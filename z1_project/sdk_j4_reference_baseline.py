#!/usr/bin/env python3
"""Record a Unitree SDK lowcmd J4 reference trajectory.

The command shape follows the SDK lowcmd example:

    q_cmd, dq_cmd = planned joint trajectory
    tau_cmd       = armModel.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, 0)

The SDK/z1_controller still applies its LOWCMD joint gains internally. This
script records both the commanded feedforward torque and lowstate.getTau(), so
the log can be compared against the pure-torque controller logs.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np

from trajectory import NDOF, scurve_trajectory

ROOT = Path(__file__).resolve().parents[1]
SDK_LIB = ROOT / "z1_sdk" / "lib"

J4_INDEX = 3
J4_LIMIT_MIN = -math.radians(87.0)
J4_LIMIT_MAX = math.radians(87.0)
DEFAULT_SDK_KP = np.array([20.0, 30.0, 30.0, 20.0, 15.0, 10.0], dtype=float)
DEFAULT_SDK_KD = np.array([2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0], dtype=float)


def default_csv_path() -> str:
    return str(Path("logs") / f"sdk_j4_reference_{time.strftime('%Y%m%d_%H%M%S')}.csv")


def load_unitree_interface():
    if str(SDK_LIB) not in sys.path:
        sys.path.insert(0, str(SDK_LIB))
    try:
        import unitree_arm_interface
    except ImportError as exc:
        raise SystemExit(
            f"failed to import Unitree SDK Python module from {SDK_LIB}. "
            "Use Python 3.8 and a sourced Unitree SDK/Gazebo environment: "
            f"{exc}"
        ) from exc
    return unitree_arm_interface


def vec6(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {arr.size}")
    return arr


def parse_angles(text: str) -> list[float]:
    values = [float(part) for part in text.replace(",", " ").split() if part]
    if not values:
        raise ValueError("--angles-deg must contain at least one angle")
    return values


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t", "segment", "segment_t", "target_j4_deg"]
        + [f"q_cmd_{i + 1}" for i in range(NDOF)]
        + [f"dq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"ddq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"tau_cmd_{i + 1}" for i in range(NDOF)]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + [f"tau_state_{i + 1}" for i in range(NDOF)]
        + [f"q_error_{i + 1}" for i in range(NDOF)]
        + [f"kp_sdk_default_{i + 1}" for i in range(NDOF)]
        + [f"kd_sdk_default_{i + 1}" for i in range(NDOF)]
    )


def build_targets(q_start: np.ndarray, angles_deg: list[float], *, return_to_start: bool) -> list[tuple[str, float, np.ndarray]]:
    targets: list[tuple[str, float, np.ndarray]] = []
    for angle in angles_deg:
        q_target = q_start.copy()
        q_target[J4_INDEX] = q_start[J4_INDEX] + math.radians(angle)
        if not J4_LIMIT_MIN <= q_target[J4_INDEX] <= J4_LIMIT_MAX:
            raise ValueError(
                f"J4 target {math.degrees(q_target[J4_INDEX]):.3f} deg violates "
                f"limit [{math.degrees(J4_LIMIT_MIN):.1f}, {math.degrees(J4_LIMIT_MAX):.1f}] deg"
            )
        targets.append((f"j4_{angle:+g}deg", angle, q_target))

    if return_to_start:
        targets.append(("return_start", 0.0, q_start.copy()))
    return targets


def print_vec(label: str, value: np.ndarray, *, degrees: bool = False) -> None:
    data = np.rad2deg(value) if degrees else value
    suffix = " deg" if degrees else ""
    print(label, "=", np.array2string(data, precision=6, suppress_small=False), suffix)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unitree SDK J4 reference baseline logger", allow_abbrev=False)
    parser.add_argument("--angles-deg", default="-30 30", help="J4 targets relative to measured start; default: '-30 30'")
    parser.add_argument("--move-time", type=float, default=10.0, help="seconds per 30 deg of J4 travel")
    parser.add_argument("--hold-time", type=float, default=0.5, help="hold time after each segment")
    parser.add_argument("--csv-log", default=default_csv_path())
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--probe-only", action="store_true", help="connect and print SDK state without moving")
    parser.add_argument("--no-return-to-start", action="store_true", help="do not add a final commanded return to measured q_start")
    parser.add_argument("--passive-on-exit", action="store_true", help="switch SDK FSM to PASSIVE after the run")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.move_time <= 0.0:
        raise ValueError("--move-time must be positive")
    if args.hold_time < 0.0:
        raise ValueError("--hold-time must be non-negative")

    unitree_arm_interface = load_unitree_interface()
    np.set_printoptions(precision=6, suppress=True)

    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    arm = unitree_arm_interface.ArmInterface(hasGripper=not args.no_gripper)
    arm.setFsmLowcmd()
    dt = float(arm._ctrlComp.dt)
    q_start = vec6(arm.lowstate.getQ(), "q_start")
    dq_start = vec6(arm.lowstate.getQd(), "dq_start")
    tau_start = vec6(arm.lowstate.getTau(), "tau_start")

    print("SDK connected")
    print("dt =", dt)
    print_vec("q_start", q_start)
    print_vec("q_start", q_start, degrees=True)
    print_vec("dq_start", dq_start)
    print_vec("tau_start", tau_start)
    print("assumed SDK LOWCMD Kp =", np.array2string(DEFAULT_SDK_KP, precision=3, suppress_small=False))
    print("assumed SDK LOWCMD Kd =", np.array2string(DEFAULT_SDK_KD, precision=3, suppress_small=False))

    if args.probe_only:
        return 0

    angles = parse_angles(args.angles_deg)
    targets = build_targets(q_start, angles, return_to_start=not args.no_return_to_start)

    csv_path = Path(args.csv_log)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    print("csv_log =", csv_path)
    print("segments =", ", ".join(name for name, _, _ in targets))

    max_abs_error = np.zeros(NDOF)
    max_abs_tau_cmd = np.zeros(NDOF)
    max_abs_tau_state = np.zeros(NDOF)
    max_abs_dq = np.zeros(NDOF)
    final_error = np.zeros(NDOF)
    final_q = q_start.copy()
    steps = 0

    wall_start = time.perf_counter()
    next_tick = wall_start
    segment_start_q = q_start.copy()

    try:
        with csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            write_header(writer)

            for segment_name, target_angle, q_goal in targets:
                if stop_requested:
                    break

                delta_deg = abs(math.degrees(q_goal[J4_INDEX] - segment_start_q[J4_INDEX]))
                segment_move_time = args.move_time * max(delta_deg / 30.0, 1e-6)
                segment_total_time = segment_move_time + args.hold_time
                segment_steps = int(math.ceil(segment_total_time / dt))
                print(
                    f"running {segment_name}: q4 "
                    f"{math.degrees(segment_start_q[J4_INDEX]):.3f} -> {math.degrees(q_goal[J4_INDEX]):.3f} deg, "
                    f"T={segment_move_time:.3f}s"
                )

                for step in range(segment_steps + 1):
                    if stop_requested:
                        break

                    segment_t = min(step * dt, segment_total_time)
                    q_cmd, dq_cmd, ddq_cmd = scurve_trajectory(segment_start_q, q_goal, segment_t, segment_move_time)
                    tau_cmd = vec6(
                        arm._ctrlComp.armModel.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, np.zeros(NDOF)),
                        "tau_cmd",
                    )

                    arm.setArmCmd(q_cmd, dq_cmd, tau_cmd)
                    if not args.no_gripper:
                        arm.setGripperCmd(arm.gripperQ, 0.0, 0.0)
                    arm.sendRecv()

                    q_actual = vec6(arm.lowstate.getQ(), "q_actual")
                    dq_actual = vec6(arm.lowstate.getQd(), "dq_actual")
                    tau_state = vec6(arm.lowstate.getTau(), "tau_state")
                    error = q_cmd - q_actual
                    elapsed = time.perf_counter() - wall_start

                    writer.writerow(
                        [
                            elapsed,
                            segment_name,
                            segment_t,
                            target_angle,
                            *q_cmd.tolist(),
                            *dq_cmd.tolist(),
                            *ddq_cmd.tolist(),
                            *tau_cmd.tolist(),
                            *q_actual.tolist(),
                            *dq_actual.tolist(),
                            *tau_state.tolist(),
                            *error.tolist(),
                            *DEFAULT_SDK_KP.tolist(),
                            *DEFAULT_SDK_KD.tolist(),
                        ]
                    )

                    max_abs_error = np.maximum(max_abs_error, np.abs(error))
                    max_abs_tau_cmd = np.maximum(max_abs_tau_cmd, np.abs(tau_cmd))
                    max_abs_tau_state = np.maximum(max_abs_tau_state, np.abs(tau_state))
                    max_abs_dq = np.maximum(max_abs_dq, np.abs(dq_actual))
                    final_error = error
                    final_q = q_actual
                    steps += 1

                    next_tick += dt
                    sleep_time = next_tick - time.perf_counter()
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
                    else:
                        next_tick = time.perf_counter()

                segment_start_q = q_goal.copy()
    finally:
        if args.passive_on_exit:
            try:
                arm.setFsm(unitree_arm_interface.ArmFSMState.PASSIVE)
            except Exception as exc:  # pragma: no cover - depends on SDK state
                print(f"warning: failed to set PASSIVE on exit: {exc}", file=sys.stderr)

    wall_elapsed = max(time.perf_counter() - wall_start, 1e-9)
    print("steps =", steps)
    print("effective_loop_rate =", f"{steps / wall_elapsed:.1f} Hz")
    print_vec("final_q", final_q)
    print_vec("actual_delta", final_q - q_start)
    print_vec("actual_delta", final_q - q_start, degrees=True)
    print_vec("final_error", final_error)
    print_vec("final_error", final_error, degrees=True)
    print_vec("max_abs_error", max_abs_error)
    print_vec("max_abs_error", max_abs_error, degrees=True)
    print_vec("max_abs_tau_cmd", max_abs_tau_cmd)
    print_vec("max_abs_tau_state", max_abs_tau_state)
    print_vec("max_abs_dq", max_abs_dq)
    print("CSV path =", csv_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
