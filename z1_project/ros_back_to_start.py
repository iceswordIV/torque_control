#!/usr/bin/env python3
"""SDK-style back-to-start helper for Unitree ROS/Gazebo.

This is not pure torque control. It mimics the useful part of
arm.backToStart() for Gazebo by publishing MotorCmd position/velocity targets
with nonzero Kp/Kd to the per-joint Unitree controllers.
"""

from __future__ import annotations

import argparse
import csv
import signal
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import rospy
    from sensor_msgs.msg import JointState
    from unitree_legged_msgs.msg import MotorCmd, MotorState
except ImportError as exc:  # pragma: no cover - requires sourced ROS env
    raise SystemExit(f"failed to import ROS Python modules; source your ROS workspace first: {exc}") from exc

NDOF = 6
PMSM_MODE = 0x0A
START_FLAT = "0 0 -0.005 -0.074 0 0"
GAZEBO_ZERO = "0 0 0 0 0 0"
DEFAULT_KP = "20 30 30 20 15 10"
DEFAULT_KD = "2000 2000 2000 2000 2000 2000"
DEFAULT_JOINT_NAMES = [f"joint{i}" for i in range(1, NDOF + 1)]
DEFAULT_CONTROLLER_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/command" for i in range(1, NDOF + 1)]
DEFAULT_STATE_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/state" for i in range(1, NDOF + 1)]


def parse_vec(text: str, name: str) -> np.ndarray:
    values = np.fromstring(text.replace(",", " "), sep=" ", dtype=float)
    if values.size == 1:
        return np.repeat(values.item(), NDOF)
    if values.size != NDOF:
        raise ValueError(f"{name} must contain 1 or {NDOF} values, got {values.size}")
    return values


def parse_list(text: str) -> list[str]:
    return [part for part in text.replace(",", " ").split() if part]


def scurve_trajectory(q_start: np.ndarray, q_goal: np.ndarray, t: float, T: float):
    if t <= 0.0:
        return q_start.copy(), np.zeros(NDOF)
    if t >= T:
        return q_goal.copy(), np.zeros(NDOF)

    s = min(1.0, max(0.0, float(t) / float(T)))
    b = 35.0 * s**4 - 84.0 * s**5 + 70.0 * s**6 - 20.0 * s**7
    bd = (140.0 * s**3 - 420.0 * s**4 + 420.0 * s**5 - 140.0 * s**6) / T
    delta = q_goal - q_start
    return q_start + b * delta, bd * delta


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


class JointStateBuffer:
    def __init__(self, joint_names: list[str]):
        self.joint_names = joint_names
        self.q: Optional[np.ndarray] = None
        self.dq: Optional[np.ndarray] = None

    def callback(self, msg: JointState) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        try:
            indices = [name_to_index[name] for name in self.joint_names]
        except KeyError:
            return
        self.q = np.array([msg.position[i] if i < len(msg.position) else 0.0 for i in indices], dtype=float)
        self.dq = np.array([msg.velocity[i] if i < len(msg.velocity) else 0.0 for i in indices], dtype=float)
        self.q = np.nan_to_num(self.q, nan=0.0, posinf=0.0, neginf=0.0)
        self.dq = np.nan_to_num(self.dq, nan=0.0, posinf=0.0, neginf=0.0)


class ControllerStateBuffer:
    def __init__(self):
        self.q = np.zeros(NDOF)
        self.dq = np.zeros(NDOF)
        self.seen = np.zeros(NDOF, dtype=bool)

    def callback(self, idx: int, msg: MotorState) -> None:
        self.q[idx] = float(msg.q)
        self.dq[idx] = float(msg.dq)
        self.seen[idx] = True

    @property
    def ready(self) -> bool:
        return bool(np.all(self.seen))


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t"]
        + [f"q_cmd_{i + 1}" for i in range(NDOF)]
        + [f"dq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"kp_{i + 1}" for i in range(NDOF)]
        + [f"kd_{i + 1}" for i in range(NDOF)]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
    )


def default_csv_path() -> str:
    return str(Path("logs") / f"ros_back_to_start_{time.strftime('%Y%m%d_%H%M%S')}.csv")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SDK-style ROS/Gazebo back-to-start command")
    parser.add_argument("--target", default=START_FLAT, help=f"target joint pose; default startFlat: {START_FLAT}")
    parser.add_argument("--gazebo-zero", action="store_true", help=f"use Gazebo zero pose instead of startFlat: {GAZEBO_ZERO}")
    parser.add_argument("--duration", type=float, default=None, help="trajectory duration [s]; default is max joint delta / --speed")
    parser.add_argument("--speed", type=float, default=1.0, help="joint speed used when --duration is omitted [rad/s]")
    parser.add_argument("--hold-final-sec", type=float, default=0.5)
    parser.add_argument("--kp", default=DEFAULT_KP)
    parser.add_argument("--kd", default=DEFAULT_KD)
    parser.add_argument("--tau", default="0", help="optional feedforward torque, scalar or 6 values")
    parser.add_argument("--rate-hz", type=float, default=500.0)
    parser.add_argument("--state-source", choices=["joint_states", "controller_states"], default="joint_states")
    parser.add_argument("--joint-state-topic", default="/z1_gazebo/joint_states")
    parser.add_argument("--state-topics", default=" ".join(DEFAULT_STATE_TOPICS))
    parser.add_argument("--joint-names", default=" ".join(DEFAULT_JOINT_NAMES))
    parser.add_argument("--command-topics", default=" ".join(DEFAULT_CONTROLLER_TOPICS))
    parser.add_argument("--csv-log", default=default_csv_path())
    parser.add_argument("--zero-on-exit", action="store_true", help="send one zero-gain command at final measured pose on exit")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if args.speed <= 0.0:
        raise ValueError("--speed must be positive")
    if args.hold_final_sec < 0.0:
        raise ValueError("--hold-final-sec must be non-negative")

    target_text = GAZEBO_ZERO if args.gazebo_zero else args.target
    q_goal = parse_vec(target_text, "--target")
    kp = parse_vec(args.kp, "--kp")
    kd = parse_vec(args.kd, "--kd")
    tau = parse_vec(args.tau, "--tau")
    joint_names = parse_list(args.joint_names)
    command_topics = parse_list(args.command_topics)
    state_topics = parse_list(args.state_topics)
    if len(joint_names) != NDOF or len(command_topics) != NDOF:
        raise ValueError("expected six joint names and six command topics")
    if args.state_source == "controller_states" and len(state_topics) != NDOF:
        raise ValueError("expected six controller state topics")

    rospy.init_node("z1_ros_back_to_start", anonymous=False)
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

    q_start = state.q.copy()
    duration = float(args.duration) if args.duration is not None else max(0.5, float(np.max(np.abs(q_goal - q_start))) / args.speed)
    if duration <= 0.0:
        raise ValueError("--duration must be positive")

    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print("q_start =", np.array2string(q_start, precision=6, suppress_small=False))
    print("q_goal =", np.array2string(q_goal, precision=6, suppress_small=False))
    print("duration =", duration)
    print("kp =", np.array2string(kp, precision=6, suppress_small=False))
    print("kd =", np.array2string(kd, precision=6, suppress_small=False))
    print("tau =", np.array2string(tau, precision=6, suppress_small=False))
    print("warning: stop ros_torque_bridge.py while this helper is publishing to the same command topics")

    Path(args.csv_log).parent.mkdir(parents=True, exist_ok=True)
    max_error = np.zeros(NDOF)
    final_error = np.zeros(NDOF)
    final_q = q_start.copy()
    steps = 0
    dt = 1.0 / args.rate_hz

    try:
        with open(args.csv_log, "w", newline="") as f:
            writer = csv.writer(f)
            write_header(writer)
            t0 = time.perf_counter()
            next_tick = t0
            while not rospy.is_shutdown() and not stop_requested:
                elapsed = time.perf_counter() - t0
                if elapsed > duration:
                    break
                q_cmd, dq_cmd = scurve_trajectory(q_start, q_goal, elapsed, duration)
                publish_cmd(publishers, q_cmd, dq_cmd, tau, kp, kd)
                if state.q is not None:
                    error = q_cmd - state.q
                    max_error = np.maximum(max_error, np.abs(error))
                    final_error = error
                    final_q = state.q.copy()
                    writer.writerow([elapsed, *q_cmd, *dq_cmd, *kp, *kd, *state.q, *state.dq])
                    steps += 1
                next_tick += dt
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0.0:
                    rospy.sleep(sleep_time)
                else:
                    next_tick = time.perf_counter()

            hold_end = time.perf_counter() + args.hold_final_sec
            while time.perf_counter() < hold_end and not rospy.is_shutdown() and not stop_requested:
                publish_cmd(publishers, q_goal, np.zeros(NDOF), tau, kp, kd)
                if state.q is not None:
                    elapsed = time.perf_counter() - t0
                    error = q_goal - state.q
                    max_error = np.maximum(max_error, np.abs(error))
                    final_error = error
                    final_q = state.q.copy()
                    writer.writerow([elapsed, *q_goal, *np.zeros(NDOF), *kp, *kd, *state.q, *state.dq])
                    steps += 1
                rospy.sleep(dt)
    finally:
        if args.zero_on_exit and not rospy.is_shutdown():
            publish_cmd(publishers, final_q, np.zeros(NDOF), np.zeros(NDOF), np.zeros(NDOF), np.zeros(NDOF))

    print("steps =", steps)
    print("final q =", np.array2string(final_q, precision=6, suppress_small=False))
    print("final error =", np.array2string(final_error, precision=6, suppress_small=False))
    print("max error =", np.array2string(max_error, precision=6, suppress_small=False))
    print("CSV path =", args.csv_log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
