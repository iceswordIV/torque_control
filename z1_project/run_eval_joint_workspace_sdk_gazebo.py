#!/usr/bin/env python3
"""Run the joint-workspace eval through the Unitree SDK/Gazebo lowcmd path.

This is the SDK reference baseline for ``run_eval_joint_workspace_gazebo.sh``.
It sends the same one-joint relative S-curve motions, but through the Unitree
SDK lowcmd interface:

    q_cmd, dq_cmd = desired trajectory
    tau_sdk_cmd   = armModel.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, 0)

The SDK/Gazebo controller also applies its LOWCMD position and velocity gains
internally. For comparison, this script records both the SDK feedforward command
and ``lowstate.getTau()`` from feedback.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from pathlib import Path
from typing import Iterable

import numpy as np

from trajectory import NDOF, scurve_trajectory

ROOT = Path(__file__).resolve().parents[1]
SDK_LIB = ROOT / "z1_sdk" / "lib"

DEFAULT_TESTS: dict[int, list[float]] = {
    1: [-30.0, -10.0, -5.0, 5.0, 10.0, 30.0],
    2: [5.0, 10.0, 30.0],
    3: [-5.0, -10.0, -30.0],
    4: [-5.0, -10.0, -30.0, 5.0, 10.0, 30.0],
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


def load_unitree_interface():
    if str(SDK_LIB) not in sys.path:
        sys.path.insert(0, str(SDK_LIB))
    try:
        import unitree_arm_interface
    except ImportError as exc:
        raise SystemExit(
            f"failed to import Unitree SDK Python module from {SDK_LIB}. "
            "Use Python 3.8 and run with the Unitree/Gazebo environment sourced: "
            f"{exc}"
        ) from exc
    return unitree_arm_interface


def vec6(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {arr.size}")
    return arr


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
    parser = argparse.ArgumentParser(description="Unitree SDK joint-workspace Gazebo baseline", allow_abbrev=False)
    parser.add_argument("--log-dir", default="logs/eval_joint_workspace_sdk")
    parser.add_argument("--move-time", type=float, default=10.0)
    parser.add_argument("--return-time", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=22.0)
    parser.add_argument("--pause-sec", type=float, default=1.0)
    parser.add_argument("--joint", type=int, default=None, help="optional 1-based joint to run instead of the full default list")
    parser.add_argument("--angles-deg", default=None, help="angles for --joint, e.g. '-30 -10 -5 5 10 30'")
    parser.add_argument("--probe-only", action="store_true", help="connect to SDK and print state without moving")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--skip-limit-violations", action="store_true")
    parser.add_argument("--passive-on-exit", action="store_true", help="switch SDK FSM to PASSIVE before exiting")
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
        + ["controller_type"]
    )


def write_summary_header(writer: csv.writer) -> None:
    writer.writerow(
        [
            "file",
            "joint",
            "angle_deg",
            "steps",
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


def desired_for_time(q_start: np.ndarray, q_goal: np.ndarray, elapsed: float, args) -> tuple[str, np.ndarray, np.ndarray, np.ndarray]:
    if elapsed <= args.move_time:
        q_cmd, dq_cmd, ddq_cmd = scurve_trajectory(q_start, q_goal, elapsed, args.move_time)
        return "outbound", q_cmd, dq_cmd, ddq_cmd
    q_cmd, dq_cmd, ddq_cmd = scurve_trajectory(q_goal, q_start, elapsed - args.move_time, args.return_time)
    return "return", q_cmd, dq_cmd, ddq_cmd


def run_one_test(arm, args, log_dir: Path, joint: int, angle_deg: float, stop_requested_fn) -> dict | None:
    joint_index = joint - 1
    q_start = vec6(arm.lowstate.getQ(), "q_start")
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
    max_abs_error = np.zeros(NDOF)
    max_abs_tau_sdk_cmd = np.zeros(NDOF)
    max_abs_tau_state = np.zeros(NDOF)
    max_abs_dq_actual = np.zeros(NDOF)
    sum_sq_error_joint = 0.0
    final_error = np.zeros(NDOF)
    final_q = q_start.copy()
    steps = 0

    print(
        f"Running {path.name}: J{joint} {angle_deg:+g} deg, "
        f"q_start={math.degrees(q_start[joint_index]):.3f} deg, "
        f"q_goal={math.degrees(q_goal[joint_index]):.3f} deg"
    )

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        write_header(writer)

        wall_start = time.perf_counter()
        next_tick = wall_start
        while not stop_requested_fn():
            elapsed = time.perf_counter() - wall_start
            if elapsed > args.duration:
                break

            phase, q_cmd, dq_cmd, ddq_cmd = desired_for_time(q_start, q_goal, elapsed, args)
            tau_sdk_cmd = vec6(
                arm._ctrlComp.armModel.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, np.zeros(NDOF)),
                "tau_sdk_cmd",
            )

            arm.setArmCmd(q_cmd, dq_cmd, tau_sdk_cmd)
            if not args.no_gripper:
                arm.setGripperCmd(arm.gripperQ, 0.0, 0.0)
            arm.sendRecv()

            q_actual = vec6(arm.lowstate.getQ(), "q_actual")
            dq_actual = vec6(arm.lowstate.getQd(), "dq_actual")
            tau_state = vec6(arm.lowstate.getTau(), "tau_state")
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
                    "unitree_sdk_lowcmd",
                ]
            )

            max_abs_error = np.maximum(max_abs_error, np.abs(error))
            max_abs_tau_sdk_cmd = np.maximum(max_abs_tau_sdk_cmd, np.abs(tau_sdk_cmd))
            max_abs_tau_state = np.maximum(max_abs_tau_state, np.abs(tau_state))
            max_abs_dq_actual = np.maximum(max_abs_dq_actual, np.abs(dq_actual))
            sum_sq_error_joint += float(error[joint_index] ** 2)
            final_error = error
            final_q = q_actual
            steps += 1

            next_tick += float(arm._ctrlComp.dt)
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    wall_elapsed = max(time.perf_counter() - wall_start, 1e-9)
    desired_delta = q_goal[joint_index] - q_start[joint_index]
    actual_delta = final_q[joint_index] - q_start[joint_index]
    achieved_pct = 100.0 * actual_delta / desired_delta if abs(desired_delta) > 1e-12 else float("nan")
    rms_error = math.sqrt(sum_sq_error_joint / max(steps, 1))

    print(
        f"  achieved={achieved_pct:.2f}% "
        f"final_err={math.degrees(final_error[joint_index]):+.3f} deg "
        f"max_err={math.degrees(max_abs_error[joint_index]):.3f} deg "
        f"max_tau_cmd={max_abs_tau_sdk_cmd[joint_index]:.3f} "
        f"max_tau_state={max_abs_tau_state[joint_index]:.3f}"
    )

    if args.pause_sec > 0.0:
        time.sleep(args.pause_sec)

    return {
        "file": str(path),
        "joint": joint,
        "angle_deg": angle_deg,
        "steps": steps,
        "effective_loop_rate_hz": steps / wall_elapsed,
        "desired_delta_deg": math.degrees(desired_delta),
        "actual_delta_deg": math.degrees(actual_delta),
        "achieved_pct": achieved_pct,
        "final_error_deg": math.degrees(final_error[joint_index]),
        "max_abs_error_deg": math.degrees(max_abs_error[joint_index]),
        "rms_error_deg": math.degrees(rms_error),
        "max_abs_tau_sdk_cmd": max_abs_tau_sdk_cmd[joint_index],
        "max_abs_tau_state": max_abs_tau_state[joint_index],
        "max_abs_dq_actual": max_abs_dq_actual[joint_index],
    }


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.move_time <= 0.0 or args.return_time <= 0.0:
        raise ValueError("--move-time and --return-time must be positive")
    if args.duration < args.move_time + args.return_time:
        raise ValueError("--duration must be at least --move-time + --return-time")
    if args.pause_sec < 0.0:
        raise ValueError("--pause-sec must be non-negative")
    if args.joint is not None and not 1 <= int(args.joint) <= NDOF:
        raise ValueError("--joint must be in 1..6")

    unitree_arm_interface = load_unitree_interface()
    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    arm = unitree_arm_interface.ArmInterface(hasGripper=not args.no_gripper)
    arm.setFsmLowcmd()

    print("SDK connected")
    print("dt =", arm._ctrlComp.dt)
    print("q_start =", np.array2string(vec6(arm.lowstate.getQ(), "q_start"), precision=6, suppress_small=False))
    print("assumed SDK LOWCMD Kp =", np.array2string(DEFAULT_SDK_KP, precision=3, suppress_small=False))
    print("assumed SDK LOWCMD Kd =", np.array2string(DEFAULT_SDK_KD, precision=3, suppress_small=False))

    if args.probe_only:
        return 0

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    summary_path = log_dir / "summary_sdk_lowcmd.csv"

    results: list[dict] = []
    try:
        for joint, angle in iter_requested_tests(args):
            if stop_requested:
                break
            result = run_one_test(arm, args, log_dir, joint, angle, lambda: stop_requested)
            if result is not None:
                results.append(result)
    finally:
        if args.passive_on_exit:
            try:
                arm.setFsm(unitree_arm_interface.ArmFSMState.PASSIVE)
            except Exception as exc:  # pragma: no cover - depends on SDK state
                print(f"warning: failed to set PASSIVE on exit: {exc}", file=sys.stderr)

    with summary_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "file",
                "joint",
                "angle_deg",
                "steps",
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
    print("Finished SDK joint workspace evaluation.")
    print("summary =", summary_path)
    print("logs =", log_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
