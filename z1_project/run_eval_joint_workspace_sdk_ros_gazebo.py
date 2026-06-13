#!/usr/bin/env python3
"""Run the Unitree SDK LOWCMD Gazebo baseline directly on ROS topics.

Unitree's ``z1_controller/build/sim_ctrl`` receives SDK LOWCMD packets and then
publishes ROS ``MotorCmd`` messages. This script publishes the same command
shape directly:

    q_cmd, dq_cmd = desired trajectory
    tau_sdk_cmd   = SDK armModel.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, 0)
    MotorCmd.Kp   = SDK lowcmd kp * 25.6
    MotorCmd.Kd   = SDK lowcmd kd * 0.0128

It avoids the SDK UDP connection, but still uses Unitree's SDK arm model for the
feedforward dynamics and Unitree's ROS/Gazebo controller law for feedback.
"""

from __future__ import annotations

import argparse
import ctypes
import csv
import math
import signal
import sys
import time
from pathlib import Path
from typing import Iterable

import numpy as np

from trajectory import NDOF, scurve_trajectory

try:
    import rospy
    from sensor_msgs.msg import JointState
    from unitree_legged_msgs.msg import MotorCmd, MotorState
except ImportError as exc:  # pragma: no cover - requires sourced ROS env
    raise SystemExit(f"failed to import ROS Python modules; source your ROS workspace first: {exc}") from exc

ROOT = Path(__file__).resolve().parents[1]
SDK_LIB = ROOT / "z1_sdk" / "lib"

PMSM_MODE = 0x0A
DEFAULT_JOINT_NAMES = [f"joint{i}" for i in range(1, NDOF + 1)]
DEFAULT_CONTROLLER_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/command" for i in range(1, NDOF + 1)]
DEFAULT_STATE_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/state" for i in range(1, NDOF + 1)]

DEFAULT_TESTS: dict[int, list[float]] = {
    1: [5.0, 10.0, 30.0],
    2: [5.0, 10.0, 30.0],
    3: [-5.0, -10.0, -30.0],
    4: [-5.0, -10.0, -30.0],
    5: [-5.0, -10.0, -30.0, 5.0, 10.0, 30.0],
    6: [-5.0, -10.0, -30.0, 5.0, 10.0, 30.0],
}

JOINT_LIMITS_RAD = np.array(
    [
        [-math.radians(150.0), math.radians(150.0)],
        [0.0, math.radians(170.0)],
        [-math.radians(165.0), 0.0],
        [-math.radians(87.0), math.radians(87.0)],
        [-math.radians(77.0), math.radians(77.0)],
        [-math.radians(160.0), math.radians(160.0)],
    ],
    dtype=float,
)

DEFAULT_SDK_KP = np.array([20.0, 30.0, 30.0, 20.0, 15.0, 10.0], dtype=float)
DEFAULT_SDK_KD = np.array([2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0], dtype=float)
SDK_TO_ROS_KP_SCALE = 25.6
SDK_TO_ROS_KD_SCALE = 0.0128

SDK_GRIPPER_END_POS_LOCAL = np.array([0.0382, 0.0, 0.0], dtype=float)
SDK_GRIPPER_END_EFFECTOR_MASS = 0.80225
SDK_GRIPPER_END_EFFECTOR_COM = np.array([0.0037, 0.0014, -0.0003], dtype=float)
SDK_GRIPPER_END_EFFECTOR_INERTIA = np.diag([0.00057593, 0.00099960, 0.00106337]).astype(float)


def load_unitree_interface():
    if str(SDK_LIB) not in sys.path:
        sys.path.insert(0, str(SDK_LIB))
    sdk_core = SDK_LIB / "libZ1_SDK_x86_64.so"
    if sdk_core.exists():
        ctypes.CDLL(str(sdk_core), mode=ctypes.RTLD_GLOBAL)
    try:
        import unitree_arm_interface
    except ImportError as exc:
        raise SystemExit(f"failed to import Unitree SDK Python module from {SDK_LIB}: {exc}") from exc
    return unitree_arm_interface


def make_standalone_model(unitree_arm_interface, *, has_gripper: bool):
    if not has_gripper:
        return unitree_arm_interface.Z1Model(np.zeros(3), 0.0, np.zeros(3), np.zeros((3, 3)))
    return unitree_arm_interface.Z1Model(
        SDK_GRIPPER_END_POS_LOCAL,
        SDK_GRIPPER_END_EFFECTOR_MASS,
        SDK_GRIPPER_END_EFFECTOR_COM,
        SDK_GRIPPER_END_EFFECTOR_INERTIA,
    )


def vec6(value, name: str) -> np.ndarray:
    arr = np.array(value, dtype=float, copy=True).reshape(-1)
    if arr.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {arr.size}")
    return arr


def parse_list(text: str) -> list[str]:
    return [part for part in text.replace(",", " ").split() if part]


def angle_label(angle_deg: float) -> str:
    value = int(abs(angle_deg)) if float(angle_deg).is_integer() else abs(angle_deg)
    return f"neg{value}deg" if angle_deg < 0.0 else f"pos{value}deg"


def csv_name(joint: int, angle_deg: float) -> str:
    return f"workspace_j{joint}_{angle_label(angle_deg)}_sdk_lowcmd.csv"


def iter_requested_tests(args) -> Iterable[tuple[int, float]]:
    if args.joint is not None:
        if args.angles_deg is None:
            raise ValueError("--angles-deg is required when --joint is provided")
        angles = [float(part) for part in args.angles_deg.replace(",", " ").split() if part]
        if not angles:
            raise ValueError("--angles-deg must contain at least one value")
        for angle in angles:
            yield int(args.joint), angle
        return

    for joint, angles in DEFAULT_TESTS.items():
        for angle in angles:
            yield joint, angle


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Direct ROS/Gazebo Unitree SDK LOWCMD joint-workspace baseline", allow_abbrev=False)
    parser.add_argument("--log-dir", default="logs/eval_joint_workspace_sdk")
    parser.add_argument("--move-time", type=float, default=5.0)
    parser.add_argument("--hold-time", type=float, default=3.0)
    parser.add_argument("--return-time", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--pause-sec", type=float, default=0.5)
    parser.add_argument("--joint", type=int, default=None)
    parser.add_argument("--angles-deg", default=None)
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--skip-limit-violations", action="store_true")
    parser.add_argument("--state-source", choices=["controller_states", "joint_states"], default="controller_states")
    parser.add_argument("--joint-state-topic", default="/z1_gazebo/joint_states")
    parser.add_argument("--joint-names", default=" ".join(DEFAULT_JOINT_NAMES))
    parser.add_argument("--command-topics", default=" ".join(DEFAULT_CONTROLLER_TOPICS))
    parser.add_argument("--state-topics", default=" ".join(DEFAULT_STATE_TOPICS))
    parser.add_argument("--rate-hz", type=float, default=500.0)
    parser.add_argument("--zero-on-exit", action="store_true")
    return parser


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t", "phase", "joint", "angle_deg"]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + [f"tau_state_{i + 1}" for i in range(NDOF)]
        + [f"q_cmd_{i + 1}" for i in range(NDOF)]
        + [f"dq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"ddq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"tau_sdk_cmd_{i + 1}" for i in range(NDOF)]
        + [f"q_error_{i + 1}" for i in range(NDOF)]
        + [f"kp_sdk_default_{i + 1}" for i in range(NDOF)]
        + [f"kd_sdk_default_{i + 1}" for i in range(NDOF)]
        + [f"kp_ros_command_{i + 1}" for i in range(NDOF)]
        + [f"kd_ros_command_{i + 1}" for i in range(NDOF)]
        + ["controller_type"]
    )


def write_summary_header(writer: csv.writer) -> None:
    writer.writerow(
        [
            "file",
            "joint",
            "angle_deg",
            "steps",
            "outbound_steps",
            "effective_loop_rate_hz",
            "desired_delta_deg",
            "actual_delta_deg",
            "achieved_pct",
            "final_error_deg",
            "max_abs_error_deg",
            "rms_error_deg",
            "max_abs_tau_sdk_cmd",
            "max_abs_tau_state",
            "max_abs_dq_actual",
        ]
    )


class JointStateBuffer:
    def __init__(self, joint_names: list[str]):
        self.joint_names = joint_names
        self.stamp = 0.0
        self.q: np.ndarray | None = None
        self.dq: np.ndarray | None = None
        self.tau: np.ndarray | None = None

    def callback(self, msg: JointState) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        try:
            indices = [name_to_index[name] for name in self.joint_names]
        except KeyError:
            return
        self.q = np.nan_to_num(np.array([msg.position[i] if i < len(msg.position) else 0.0 for i in indices], dtype=float))
        self.dq = np.nan_to_num(np.array([msg.velocity[i] if i < len(msg.velocity) else 0.0 for i in indices], dtype=float))
        self.tau = np.nan_to_num(np.array([msg.effort[i] if i < len(msg.effort) else 0.0 for i in indices], dtype=float))
        self.stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.monotonic()

    @property
    def ready(self) -> bool:
        return self.q is not None and self.dq is not None and self.tau is not None


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


def publish_cmd(publishers, q_cmd: np.ndarray, dq_cmd: np.ndarray, tau_cmd: np.ndarray, kp_ros: np.ndarray, kd_ros: np.ndarray) -> None:
    for i, pub in enumerate(publishers):
        msg = MotorCmd()
        msg.mode = PMSM_MODE
        msg.q = float(q_cmd[i])
        msg.dq = float(dq_cmd[i])
        msg.tau = float(tau_cmd[i])
        msg.Kp = float(kp_ros[i])
        msg.Kd = float(kd_ros[i])
        pub.publish(msg)


def desired_for_time(q_start: np.ndarray, q_goal: np.ndarray, elapsed: float, args) -> tuple[str, np.ndarray, np.ndarray, np.ndarray]:
    if elapsed <= args.move_time:
        q_cmd, dq_cmd, ddq_cmd = scurve_trajectory(q_start, q_goal, elapsed, args.move_time)
        return "outbound", q_cmd, dq_cmd, ddq_cmd
    if elapsed <= args.move_time + args.hold_time:
        return "outbound_hold", q_goal.copy(), np.zeros(NDOF), np.zeros(NDOF)
    q_cmd, dq_cmd, ddq_cmd = scurve_trajectory(
        q_goal,
        q_start,
        elapsed - args.move_time - args.hold_time,
        args.return_time,
    )
    return "return", q_cmd, dq_cmd, ddq_cmd


def run_one_test(model, state, publishers, args, log_dir: Path, joint: int, angle_deg: float, stop_requested_fn) -> dict | None:
    joint_index = joint - 1
    q_start = vec6(state.q, "q_start")
    q_goal = q_start.copy()
    q_goal[joint_index] += math.radians(angle_deg)

    lower, upper = JOINT_LIMITS_RAD[joint_index]
    if not lower <= q_goal[joint_index] <= upper:
        message = (
            f"skipping J{joint} {angle_deg:g} deg: target "
            f"{math.degrees(q_goal[joint_index]):.3f} deg violates "
            f"[{math.degrees(lower):.1f}, {math.degrees(upper):.1f}] deg"
        )
        if args.skip_limit_violations:
            print(message)
            return None
        raise ValueError(message)

    path = log_dir / csv_name(joint, angle_deg)
    print(
        f"Running {path.name}: J{joint} {angle_deg:+g} deg, "
        f"q_start={math.degrees(q_start[joint_index]):.3f} deg, "
        f"q_goal={math.degrees(q_goal[joint_index]):.3f} deg"
    )

    kp_ros = DEFAULT_SDK_KP * SDK_TO_ROS_KP_SCALE
    kd_ros = DEFAULT_SDK_KD * SDK_TO_ROS_KD_SCALE

    outbound_sum_sq_error_joint = 0.0
    outbound_final_error = np.zeros(NDOF)
    outbound_final_q = q_start.copy()
    outbound_max_abs_error = np.zeros(NDOF)
    outbound_max_abs_tau_sdk_cmd = np.zeros(NDOF)
    outbound_max_abs_tau_state = np.zeros(NDOF)
    outbound_max_abs_dq_actual = np.zeros(NDOF)
    outbound_steps = 0
    steps = 0

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        write_header(writer)

        wall_start = time.perf_counter()
        next_tick = wall_start
        dt = 1.0 / args.rate_hz
        while not rospy.is_shutdown() and not stop_requested_fn():
            elapsed = time.perf_counter() - wall_start
            if elapsed > args.duration:
                break

            phase, q_cmd, dq_cmd, ddq_cmd = desired_for_time(q_start, q_goal, elapsed, args)
            tau_sdk_cmd = vec6(model.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, np.zeros(NDOF)), "tau_sdk_cmd")
            publish_cmd(publishers, q_cmd, dq_cmd, tau_sdk_cmd, kp_ros, kd_ros)

            q_actual = vec6(state.q, "q_actual")
            dq_actual = vec6(state.dq, "dq_actual")
            tau_state = vec6(state.tau, "tau_state")
            error = q_cmd - q_actual

            writer.writerow(
                [
                    elapsed,
                    phase,
                    joint,
                    angle_deg,
                    *q_actual.tolist(),
                    *dq_actual.tolist(),
                    *tau_state.tolist(),
                    *q_cmd.tolist(),
                    *dq_cmd.tolist(),
                    *ddq_cmd.tolist(),
                    *tau_sdk_cmd.tolist(),
                    *error.tolist(),
                    *DEFAULT_SDK_KP.tolist(),
                    *DEFAULT_SDK_KD.tolist(),
                    *kp_ros.tolist(),
                    *kd_ros.tolist(),
                    "unitree_sdk_lowcmd_ros_direct",
                ]
            )

            if phase == "outbound":
                outbound_max_abs_error = np.maximum(outbound_max_abs_error, np.abs(error))
                outbound_max_abs_tau_sdk_cmd = np.maximum(outbound_max_abs_tau_sdk_cmd, np.abs(tau_sdk_cmd))
                outbound_max_abs_tau_state = np.maximum(outbound_max_abs_tau_state, np.abs(tau_state))
                outbound_max_abs_dq_actual = np.maximum(outbound_max_abs_dq_actual, np.abs(dq_actual))
                outbound_sum_sq_error_joint += float(error[joint_index] ** 2)
                outbound_final_error = error
                outbound_final_q = q_actual
                outbound_steps += 1
            steps += 1

            next_tick += dt
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0.0:
                rospy.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    wall_elapsed = max(time.perf_counter() - wall_start, 1e-9)
    desired_delta = q_goal[joint_index] - q_start[joint_index]
    actual_delta = outbound_final_q[joint_index] - q_start[joint_index]
    achieved_pct = 100.0 * actual_delta / desired_delta if abs(desired_delta) > 1e-12 else float("nan")
    rms_error = math.sqrt(outbound_sum_sq_error_joint / max(outbound_steps, 1))

    print(
        f"  achieved={achieved_pct:.2f}% "
        f"final_err={math.degrees(outbound_final_error[joint_index]):+.3f} deg "
        f"max_err={math.degrees(outbound_max_abs_error[joint_index]):.3f} deg "
        f"max_tau_cmd={outbound_max_abs_tau_sdk_cmd[joint_index]:.3f} "
        f"max_tau_state={outbound_max_abs_tau_state[joint_index]:.3f}"
    )

    if args.pause_sec > 0.0:
        rospy.sleep(args.pause_sec)

    return {
        "file": str(path),
        "joint": joint,
        "angle_deg": angle_deg,
        "steps": steps,
        "outbound_steps": outbound_steps,
        "effective_loop_rate_hz": steps / wall_elapsed,
        "desired_delta_deg": math.degrees(desired_delta),
        "actual_delta_deg": math.degrees(actual_delta),
        "achieved_pct": achieved_pct,
        "final_error_deg": math.degrees(outbound_final_error[joint_index]),
        "max_abs_error_deg": math.degrees(outbound_max_abs_error[joint_index]),
        "rms_error_deg": math.degrees(rms_error),
        "max_abs_tau_sdk_cmd": outbound_max_abs_tau_sdk_cmd[joint_index],
        "max_abs_tau_state": outbound_max_abs_tau_state[joint_index],
        "max_abs_dq_actual": outbound_max_abs_dq_actual[joint_index],
    }


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.move_time <= 0.0 or args.return_time <= 0.0:
        raise ValueError("--move-time and --return-time must be positive")
    if args.hold_time < 0.0:
        raise ValueError("--hold-time must be non-negative")
    if args.duration < args.move_time + args.hold_time + args.return_time:
        raise ValueError("--duration must be at least --move-time + --hold-time + --return-time")
    if args.pause_sec < 0.0:
        raise ValueError("--pause-sec must be non-negative")
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if args.joint is not None and not 1 <= int(args.joint) <= NDOF:
        raise ValueError("--joint must be in 1..6")

    unitree_arm_interface = load_unitree_interface()
    model = make_standalone_model(unitree_arm_interface, has_gripper=not args.no_gripper)

    rospy.init_node("z1_sdk_lowcmd_ros_direct_eval", anonymous=False)
    joint_names = parse_list(args.joint_names)
    command_topics = parse_list(args.command_topics)
    state_topics = parse_list(args.state_topics)
    if len(joint_names) != NDOF or len(command_topics) != NDOF:
        raise ValueError("expected six joint names and six command topics")
    if args.state_source == "controller_states" and len(state_topics) != NDOF:
        raise ValueError("expected six controller state topics")

    if args.state_source == "joint_states":
        state = JointStateBuffer(joint_names)
        rospy.Subscriber(args.joint_state_topic, JointState, state.callback, queue_size=1)
    else:
        state = ControllerStateBuffer()
        for idx, topic in enumerate(state_topics):
            rospy.Subscriber(topic, MotorState, lambda msg, idx=idx: state.callback(idx, msg), queue_size=1)
    publishers = [rospy.Publisher(topic, MotorCmd, queue_size=1) for topic in command_topics]
    rospy.sleep(0.5)

    deadline = time.monotonic() + 5.0
    while not state.ready and not rospy.is_shutdown():
        if time.monotonic() > deadline:
            raise TimeoutError(f"timed out waiting for {args.state_source} feedback")
        rospy.sleep(0.01)

    print("ROS direct SDK LOWCMD baseline")
    print("q_start =", np.array2string(vec6(state.q, "q_start"), precision=6, suppress_small=False))
    print("SDK lowcmd Kp =", np.array2string(DEFAULT_SDK_KP, precision=3, suppress_small=False))
    print("SDK lowcmd Kd =", np.array2string(DEFAULT_SDK_KD, precision=3, suppress_small=False))
    print("ROS MotorCmd Kp =", np.array2string(DEFAULT_SDK_KP * SDK_TO_ROS_KP_SCALE, precision=3, suppress_small=False))
    print("ROS MotorCmd Kd =", np.array2string(DEFAULT_SDK_KD * SDK_TO_ROS_KD_SCALE, precision=3, suppress_small=False))

    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    summary_path = log_dir / "summary_sdk_lowcmd.csv"

    results: list[dict] = []
    try:
        for joint, angle in iter_requested_tests(args):
            if stop_requested:
                break
            result = run_one_test(model, state, publishers, args, log_dir, joint, angle, lambda: stop_requested)
            if result is not None:
                results.append(result)
    finally:
        if args.zero_on_exit:
            zero = np.zeros(NDOF)
            q_hold = vec6(state.q, "q_hold") if state.ready else zero
            for _ in range(20):
                publish_cmd(publishers, q_hold, zero, zero, zero, zero)
                rospy.sleep(1.0 / args.rate_hz)

    with summary_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "file",
                "joint",
                "angle_deg",
                "steps",
                "outbound_steps",
                "effective_loop_rate_hz",
                "desired_delta_deg",
                "actual_delta_deg",
                "achieved_pct",
                "final_error_deg",
                "max_abs_error_deg",
                "rms_error_deg",
                "max_abs_tau_sdk_cmd",
                "max_abs_tau_state",
                "max_abs_dq_actual",
            ],
        )
        writer.writeheader()
        writer.writerows(results)

    print()
    print("Finished direct ROS SDK LOWCMD joint workspace evaluation.")
    print("summary =", summary_path)
    print("logs =", log_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
