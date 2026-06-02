#!/usr/bin/env python3
"""Replay an SDK-style lowcmd trace directly to Unitree ROS/Gazebo controllers.

This is intentionally not the pure torque bridge. It publishes MotorCmd with the
recorded q/dq/tau fields plus configurable Kp/Kd, matching the Gazebo controller
law:

    tau_applied = Kp * (q_cmd - q_actual) + Kd * (dq_cmd - dq_actual) + tau_cmd
"""

from __future__ import annotations

import argparse
import csv
import signal
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import rospy
    from sensor_msgs.msg import JointState
    from unitree_legged_msgs.msg import MotorCmd, MotorState
except ImportError as exc:  # pragma: no cover - requires sourced ROS env
    raise SystemExit(f"failed to import ROS Python modules; source your ROS workspace first: {exc}") from exc

NDOF = 6
PMSM_MODE = 0x0A
DEFAULT_JOINT_NAMES = [f"joint{i}" for i in range(1, NDOF + 1)]
DEFAULT_CONTROLLER_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/command" for i in range(1, NDOF + 1)]
DEFAULT_STATE_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/state" for i in range(1, NDOF + 1)]
DEFAULT_KP = "20 30 30 20 15 10"
DEFAULT_KD = "2000 2000 2000 2000 2000 2000"


def parse_vec(text: str, name: str) -> np.ndarray:
    values = np.fromstring(text.replace(",", " "), sep=" ", dtype=float)
    if values.size == 1:
        return np.repeat(values.item(), NDOF)
    if values.size != NDOF:
        raise ValueError(f"{name} must contain 1 or {NDOF} values, got {values.size}")
    return values


def parse_list(text: str) -> list[str]:
    return [part for part in text.replace(",", " ").split() if part]


def field(row: Dict[str, str], key: str) -> str:
    value = row.get(key, "")
    if value == "":
        raise KeyError(f"missing required column {key!r}")
    return value


def read_trace(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t_values: list[float] = []
    q_values: list[np.ndarray] = []
    dq_values: list[np.ndarray] = []
    tau_values: list[np.ndarray] = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_values.append(float(field(row, "t")))
            q_values.append(
                np.array(
                    [
                        float(field(row, f"q_cmd_{i}"))
                        for i in range(1, NDOF + 1)
                    ],
                    dtype=float,
                )
            )
            dq_values.append(
                np.array(
                    [
                        float(field(row, f"dq_cmd_{i}"))
                        for i in range(1, NDOF + 1)
                    ],
                    dtype=float,
                )
            )
            tau_values.append(
                np.array(
                    [
                        float(field(row, f"tau_{i}"))
                        for i in range(1, NDOF + 1)
                    ],
                    dtype=float,
                )
            )

    if not t_values:
        raise ValueError(f"no usable rows found in {path}")

    t = np.array(t_values, dtype=float)
    t -= t[0]
    return t, np.vstack(q_values), np.vstack(dq_values), np.vstack(tau_values)


def interp_matrix(t_src: np.ndarray, values: np.ndarray, t_query: float) -> np.ndarray:
    t_query = float(np.clip(t_query, t_src[0], t_src[-1]))
    out = np.empty(values.shape[1], dtype=float)
    for i in range(values.shape[1]):
        out[i] = np.interp(t_query, t_src, values[:, i])
    return out


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t"]
        + [f"q_cmd_{i + 1}" for i in range(NDOF)]
        + [f"dq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"tau_cmd_{i + 1}" for i in range(NDOF)]
        + [f"kp_{i + 1}" for i in range(NDOF)]
        + [f"kd_{i + 1}" for i in range(NDOF)]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + [f"tau_est_{i + 1}" for i in range(NDOF)]
    )


class JointStateBuffer:
    def __init__(self, joint_names: list[str]):
        self.joint_names = joint_names
        self.stamp = 0.0
        self.q: Optional[np.ndarray] = None
        self.dq: Optional[np.ndarray] = None
        self.tau: Optional[np.ndarray] = None

    def callback(self, msg: JointState) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        try:
            indices = [name_to_index[name] for name in self.joint_names]
        except KeyError:
            return

        self.q = np.array([msg.position[i] if i < len(msg.position) else 0.0 for i in indices], dtype=float)
        self.dq = np.array([msg.velocity[i] if i < len(msg.velocity) else 0.0 for i in indices], dtype=float)
        self.tau = np.array([msg.effort[i] if i < len(msg.effort) else 0.0 for i in indices], dtype=float)
        self.q = np.nan_to_num(self.q, nan=0.0, posinf=0.0, neginf=0.0)
        self.dq = np.nan_to_num(self.dq, nan=0.0, posinf=0.0, neginf=0.0)
        self.tau = np.nan_to_num(self.tau, nan=0.0, posinf=0.0, neginf=0.0)
        self.stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.monotonic()


class ControllerStateBuffer:
    def __init__(self):
        self.stamp = 0.0
        self.q = np.zeros(NDOF)
        self.dq = np.zeros(NDOF)
        self.tau = np.zeros(NDOF)
        self.seen = np.zeros(NDOF, dtype=bool)

    def callback(self, idx: int, msg: MotorState) -> None:
        self.q[idx] = float(msg.q)
        self.dq[idx] = float(msg.dq)
        self.tau[idx] = float(msg.tauEst)
        self.seen[idx] = True
        self.stamp = time.monotonic()

    @property
    def ready(self) -> bool:
        return bool(np.all(self.seen))


def publish_cmd(publishers, q_cmd: np.ndarray, dq_cmd: np.ndarray, tau_cmd: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> None:
    for i, pub in enumerate(publishers):
        msg = MotorCmd()
        msg.mode = PMSM_MODE
        msg.q = float(q_cmd[i])
        msg.dq = float(dq_cmd[i])
        msg.tau = float(tau_cmd[i])
        msg.Kp = float(kp[i])
        msg.Kd = float(kd[i])
        pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay SDK-style q/dq/tau lowcmd trace to ROS/Gazebo")
    parser.add_argument("input_csv")
    parser.add_argument("--csv-log", default=str(Path("logs") / f"sdk_lowcmd_replay_{time.strftime('%Y%m%d_%H%M%S')}.csv"))
    parser.add_argument("--state-source", choices=["controller_states", "joint_states"], default="controller_states")
    parser.add_argument("--joint-state-topic", default="/z1_gazebo/joint_states")
    parser.add_argument("--state-topics", default=" ".join(DEFAULT_STATE_TOPICS))
    parser.add_argument("--joint-names", default=" ".join(DEFAULT_JOINT_NAMES))
    parser.add_argument("--command-topics", default=" ".join(DEFAULT_CONTROLLER_TOPICS))
    parser.add_argument("--kp", default=DEFAULT_KP)
    parser.add_argument("--kd", default=DEFAULT_KD)
    parser.add_argument("--rate-hz", type=float, default=500.0)
    parser.add_argument("--hold-final-sec", type=float, default=0.2)
    parser.add_argument("--zero-on-exit", action="store_true", help="send zero torque/gains on exit")
    args = parser.parse_args()

    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if Path(args.csv_log).exists() and Path(args.csv_log).is_dir():
        raise IsADirectoryError(f"--csv-log must be a CSV file path, not a directory: {args.csv_log}")
    if Path(args.input_csv).resolve() == Path(args.csv_log).resolve():
        raise ValueError("--csv-log must be different from input_csv")

    kp = parse_vec(args.kp, "--kp")
    kd = parse_vec(args.kd, "--kd")
    joint_names = parse_list(args.joint_names)
    command_topics = parse_list(args.command_topics)
    state_topics = parse_list(args.state_topics)
    if len(joint_names) != NDOF or len(command_topics) != NDOF:
        raise ValueError("expected six joint names and six command topics")
    if args.state_source == "controller_states" and len(state_topics) != NDOF:
        raise ValueError("expected six controller state topics")

    t_src, q_src, dq_src, tau_src = read_trace(args.input_csv)
    rospy.init_node("z1_sdk_lowcmd_replay", anonymous=False)
    if args.state_source == "joint_states":
        state = JointStateBuffer(joint_names)
        rospy.Subscriber(args.joint_state_topic, JointState, state.callback, queue_size=1)
        state_ready = lambda: state.q is not None
    else:
        state = ControllerStateBuffer()
        for idx, topic in enumerate(state_topics):
            rospy.Subscriber(topic, MotorState, lambda msg, idx=idx: state.callback(idx, msg), queue_size=1)
        state_ready = lambda: state.ready
    publishers = [rospy.Publisher(topic, MotorCmd, queue_size=1) for topic in command_topics]
    rospy.sleep(0.5)

    deadline = time.monotonic() + 5.0
    while not state_ready() and not rospy.is_shutdown():
        if time.monotonic() > deadline:
            raise TimeoutError(f"timed out waiting for {args.state_source} feedback")
        rospy.sleep(0.01)

    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print("input =", args.input_csv)
    print("duration =", f"{t_src[-1]:.6f}", "samples =", len(t_src))
    print("kp =", np.array2string(kp, precision=6, suppress_small=False))
    print("kd =", np.array2string(kd, precision=6, suppress_small=False))
    print("start q =", np.array2string(state.q, precision=6, suppress_small=False))

    Path(args.csv_log).parent.mkdir(parents=True, exist_ok=True)
    max_error = np.zeros(NDOF)
    final_error = np.zeros(NDOF)
    final_q = state.q.copy()
    steps = 0

    try:
        with open(args.csv_log, "w", newline="") as f:
            writer = csv.writer(f)
            write_header(writer)
            t0 = time.perf_counter()
            next_tick = t0
            dt = 1.0 / args.rate_hz

            while not rospy.is_shutdown() and not stop_requested:
                elapsed = time.perf_counter() - t0
                if elapsed > t_src[-1]:
                    break

                q_cmd = interp_matrix(t_src, q_src, elapsed)
                dq_cmd = interp_matrix(t_src, dq_src, elapsed)
                tau_cmd = interp_matrix(t_src, tau_src, elapsed)
                publish_cmd(publishers, q_cmd, dq_cmd, tau_cmd, kp, kd)

                if state.q is not None:
                    error = q_cmd - state.q
                    max_error = np.maximum(max_error, np.abs(error))
                    final_error = error
                    final_q = state.q.copy()
                    writer.writerow([elapsed, *q_cmd, *dq_cmd, *tau_cmd, *kp, *kd, *state.q, *state.dq, *state.tau])
                    steps += 1

                next_tick += dt
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0.0:
                    rospy.sleep(sleep_time)
                else:
                    next_tick = time.perf_counter()

            if args.hold_final_sec > 0.0 and not stop_requested:
                q_cmd = q_src[-1]
                dq_cmd = np.zeros(NDOF)
                tau_cmd = tau_src[-1]
                hold_end = time.perf_counter() + args.hold_final_sec
                while time.perf_counter() < hold_end and not rospy.is_shutdown():
                    publish_cmd(publishers, q_cmd, dq_cmd, tau_cmd, kp, kd)
                    if state.q is not None:
                        elapsed = time.perf_counter() - t0
                        error = q_cmd - state.q
                        max_error = np.maximum(max_error, np.abs(error))
                        final_error = error
                        final_q = state.q.copy()
                        writer.writerow([elapsed, *q_cmd, *dq_cmd, *tau_cmd, *kp, *kd, *state.q, *state.dq, *state.tau])
                        steps += 1
                    rospy.sleep(dt)
    finally:
        if args.zero_on_exit:
            zero = np.zeros(NDOF)
            for _ in range(20):
                publish_cmd(publishers, final_q, zero, zero, zero, zero)
                rospy.sleep(1.0 / args.rate_hz)

    print("steps =", steps)
    print("final q =", np.array2string(final_q, precision=6, suppress_small=False))
    print("final error =", np.array2string(final_error, precision=6, suppress_small=False))
    print("max error =", np.array2string(max_error, precision=6, suppress_small=False))
    print("CSV path =", args.csv_log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
