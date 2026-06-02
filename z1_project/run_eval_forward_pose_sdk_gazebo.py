#!/usr/bin/env python3
"""Run the forward-pose eval through the Unitree SDK/Gazebo lowcmd path.

This is the SDK reference baseline for ``run_eval_forward_pose_4controllers_gazebo.sh``.
It uses the same scaled forward-pose targets and S-curve timing, but sends the
trajectory through Unitree SDK LOWCMD:

    q_cmd, dq_cmd = desired trajectory
    tau_sdk_cmd   = armModel.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, 0)

The Unitree Gazebo controller applies its LOWCMD Kp/Kd internally. The CSV logs
therefore include both the SDK feedforward torque command and ``lowstate.getTau()``
feedback for comparison against the pure-torque controller logs.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import numpy as np

from trajectory import NDOF, parse_vec6, scurve_trajectory

ROOT = Path(__file__).resolve().parents[1]
SDK_LIB = ROOT / "z1_sdk" / "lib"

DEFAULT_TARGET = "0 1.5 -1 -0.54 0 0"
DEFAULT_SCALES = "0.25 0.50 0.75 1.00"
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


def parse_scales(text: str) -> list[float]:
    values = [float(part) for part in text.replace(",", " ").split() if part]
    if not values:
        raise ValueError("--scales must contain at least one value")
    for value in values:
        if value <= 0.0:
            raise ValueError("--scales values must be positive")
    return values


def scale_tag(scale: float) -> str:
    pct = int(round(scale * 100.0))
    if abs(scale * 100.0 - pct) < 1e-9:
        return f"{pct}pct"
    return f"{scale:g}".replace(".", "p")


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t", "phase", "scale", "tag"]
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


def desired_for_time(q_start: np.ndarray, q_goal: np.ndarray, elapsed: float, args):
    if elapsed <= args.move_time:
        q_cmd, dq_cmd, ddq_cmd = scurve_trajectory(q_start, q_goal, elapsed, args.move_time)
        return "outbound", q_cmd, dq_cmd, ddq_cmd
    q_cmd, dq_cmd, ddq_cmd = scurve_trajectory(q_goal, q_start, elapsed - args.move_time, args.return_time)
    return "return", q_cmd, dq_cmd, ddq_cmd


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unitree SDK forward-pose Gazebo baseline", allow_abbrev=False)
    parser.add_argument("--log-dir", default="logs/eval_forward_pose_sdk")
    parser.add_argument("--target", default=DEFAULT_TARGET)
    parser.add_argument("--scales", default=DEFAULT_SCALES)
    parser.add_argument("--move-time", type=float, default=15.0)
    parser.add_argument("--return-time", type=float, default=15.0)
    parser.add_argument("--duration", type=float, default=32.0)
    parser.add_argument("--pause-sec", type=float, default=1.0)
    parser.add_argument("--probe-only", action="store_true", help="connect to SDK and print state without moving")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--passive-on-exit", action="store_true", help="switch SDK FSM to PASSIVE before exiting")
    return parser


def run_one_scale(arm, args, log_dir: Path, q_target: np.ndarray, scale: float, stop_requested_fn) -> dict:
    tag = scale_tag(scale)
    path = log_dir / f"forward_{tag}_sdk_lowcmd.csv"
    q_start = vec6(arm.lowstate.getQ(), "q_start")
    q_goal = q_start + scale * (q_target - q_start)

    max_abs_error = np.zeros(NDOF)
    max_abs_tau_sdk_cmd = np.zeros(NDOF)
    max_abs_tau_state = np.zeros(NDOF)
    max_abs_dq_actual = np.zeros(NDOF)
    sum_sq_error = np.zeros(NDOF)
    final_error = np.zeros(NDOF)
    final_q = q_start.copy()
    steps = 0

    print(f"Running forward_{tag}_sdk_lowcmd.csv | scale={scale:g}")
    print("  q_start =", np.array2string(q_start, precision=6, suppress_small=False))
    print("  q_goal  =", np.array2string(q_goal, precision=6, suppress_small=False))

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
                    scale,
                    tag,
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
            sum_sq_error += error * error
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
    desired_delta = q_goal - q_start
    actual_delta = final_q - q_start
    rms_error = np.sqrt(sum_sq_error / max(steps, 1))

    result = {
        "file": str(path),
        "scale": scale,
        "tag": tag,
        "steps": steps,
        "effective_loop_rate_hz": steps / wall_elapsed,
        "desired_delta_norm": float(np.linalg.norm(desired_delta)),
        "actual_delta_norm": float(np.linalg.norm(actual_delta)),
        "final_error_norm": float(np.linalg.norm(final_error)),
        "max_abs_error_norm": float(np.linalg.norm(max_abs_error)),
        "rms_error_norm": float(np.linalg.norm(rms_error)),
        "max_abs_tau_sdk_cmd_norm": float(np.linalg.norm(max_abs_tau_sdk_cmd)),
        "max_abs_tau_state_norm": float(np.linalg.norm(max_abs_tau_state)),
        "max_abs_dq_actual_norm": float(np.linalg.norm(max_abs_dq_actual)),
    }
    for i in range(NDOF):
        result[f"final_error_{i + 1}"] = final_error[i]
        result[f"max_abs_error_{i + 1}"] = max_abs_error[i]
        result[f"rms_error_{i + 1}"] = rms_error[i]
        result[f"desired_delta_{i + 1}"] = desired_delta[i]
        result[f"actual_delta_{i + 1}"] = actual_delta[i]
        result[f"max_abs_tau_sdk_cmd_{i + 1}"] = max_abs_tau_sdk_cmd[i]
        result[f"max_abs_tau_state_{i + 1}"] = max_abs_tau_state[i]

    print(
        f"  final_error_norm={result['final_error_norm']:.4f} "
        f"max_error_norm={result['max_abs_error_norm']:.4f} "
        f"max_tau_cmd_norm={result['max_abs_tau_sdk_cmd_norm']:.4f} "
        f"max_tau_state_norm={result['max_abs_tau_state_norm']:.4f}"
    )

    if args.pause_sec > 0.0:
        time.sleep(args.pause_sec)
    return result


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.move_time <= 0.0 or args.return_time <= 0.0:
        raise ValueError("--move-time and --return-time must be positive")
    if args.duration < args.move_time + args.return_time:
        raise ValueError("--duration must be at least --move-time + --return-time")
    if args.pause_sec < 0.0:
        raise ValueError("--pause-sec must be non-negative")

    target = parse_vec6(args.target)
    scales = parse_scales(args.scales)
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
    print("target =", np.array2string(target, precision=6, suppress_small=False))
    print("scales =", scales)
    print("assumed SDK LOWCMD Kp =", np.array2string(DEFAULT_SDK_KP, precision=3, suppress_small=False))
    print("assumed SDK LOWCMD Kd =", np.array2string(DEFAULT_SDK_KD, precision=3, suppress_small=False))

    if args.probe_only:
        return 0

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    results = []

    try:
        for scale in scales:
            if stop_requested:
                break
            results.append(run_one_scale(arm, args, log_dir, target, scale, lambda: stop_requested))
    finally:
        if args.passive_on_exit:
            try:
                arm.setFsm(unitree_arm_interface.ArmFSMState.PASSIVE)
            except Exception as exc:  # pragma: no cover - depends on SDK state
                print(f"warning: failed to set PASSIVE on exit: {exc}", file=sys.stderr)

    summary_path = log_dir / "summary_sdk_lowcmd.csv"
    if results:
        fieldnames = list(results[0].keys())
        with summary_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

    print()
    print("Finished SDK forward-pose evaluation.")
    print("summary =", summary_path)
    print("logs =", log_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
