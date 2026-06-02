#!/usr/bin/env python3
"""Record or generate the Unitree SDK forward-pose lowcmd torque trace.

This reproduces the SDK example shape:

    q_cmd  = linear interpolation from current q to forward pose
    dq_cmd = constant velocity over the move
    tau    = SDK armModel.inverseDynamics(q_cmd, dq_cmd, 0, 0)

Normal mode sends the command to the SDK and records measured feedback. Offline
mode constructs Z1Model directly, opens no UDP connection, and writes a torque
trace that can still be replayed through replay_recorded_torque.py.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import numpy as np

from trajectory import NDOF, parse_vec6

ROOT = Path(__file__).resolve().parents[1]
SDK_LIB = ROOT / "z1_sdk" / "lib"
DEFAULT_TARGET = "0.0 1.5 -1.0 -0.54 0.0 0.0"
SDK_GRIPPER_END_POS_LOCAL = np.array([0.0382, 0.0, 0.0], dtype=float)
SDK_GRIPPER_END_EFFECTOR_MASS = 0.80225
SDK_GRIPPER_END_EFFECTOR_COM = np.array([0.0037, 0.0014, -0.0003], dtype=float)
SDK_GRIPPER_END_EFFECTOR_INERTIA = np.diag([0.00057593, 0.00099960, 0.00106337]).astype(float)


def default_csv_path() -> str:
    return str(Path("logs") / f"sdk_forward_tau_{time.strftime('%Y%m%d_%H%M%S')}.csv")


def load_unitree_interface():
    if str(SDK_LIB) not in sys.path:
        sys.path.insert(0, str(SDK_LIB))
    try:
        import unitree_arm_interface
    except ImportError as exc:
        raise SystemExit(
            f"failed to import Unitree SDK Python module from {SDK_LIB}. "
            "Run with the Python version matching the SDK .so and from a sourced SDK environment: "
            f"{exc}"
        ) from exc
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
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {arr.size}")
    return arr


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t"]
        + [f"q_cmd_{i + 1}" for i in range(NDOF)]
        + [f"dq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"ddq_cmd_{i + 1}" for i in range(NDOF)]
        + [f"tau_{i + 1}" for i in range(NDOF)]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + ["gripper_cmd", "alpha"]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record SDK inverseDynamics torque while moving to the forward pose",
        allow_abbrev=False,
    )
    parser.add_argument("--target", default=DEFAULT_TARGET, help="six joint target values in radians")
    parser.add_argument("--start-q", default=None, help="offline mode start position; default is all zeros")
    parser.add_argument("--duration-steps", type=int, default=1000)
    parser.add_argument("--dt", type=float, default=None, help="override SDK dt; default uses arm._ctrlComp.dt")
    parser.add_argument("--gripper-target", type=float, default=-1.0)
    parser.add_argument("--csv-log", default=default_csv_path())
    parser.add_argument("--offline", action="store_true", help="generate the torque CSV without opening SDK UDP or moving")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--no-back-to-start", action="store_true", help="do not call backToStart() after recording")
    return parser


def write_trace(
    *,
    args,
    arm_model,
    target_pos: np.ndarray,
    q_start: np.ndarray,
    dt: float,
    arm=None,
    stop_requested_fn=None,
) -> tuple[int, np.ndarray, np.ndarray]:
    dq_cmd = (target_pos - q_start) / (args.duration_steps * dt)
    ddq_cmd = np.zeros(NDOF)

    csv_path = Path(args.csv_log)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    print("recording SDK forward-pose torque trace" if arm is not None else "generating offline SDK-model torque trace")
    print("q_start =", np.array2string(q_start, precision=6, suppress_small=False))
    print("target_pos =", np.array2string(target_pos, precision=6, suppress_small=False))
    print("dt =", dt)
    print("duration_steps =", args.duration_steps)
    print("csv_log =", csv_path)

    max_tau = np.zeros(NDOF)
    final_q = q_start.copy()
    steps = 0

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        write_header(writer)
        t0 = time.perf_counter()
        next_tick = t0

        for i in range(args.duration_steps + 1):
            if stop_requested_fn is not None and stop_requested_fn():
                break

            alpha = i / args.duration_steps
            q_cmd = q_start * (1.0 - alpha) + target_pos * alpha
            tau = vec6(arm_model.inverseDynamics(q_cmd, dq_cmd, ddq_cmd, np.zeros(NDOF)), "tau")
            gripper_cmd = args.gripper_target * alpha

            if arm is None:
                q_actual = q_cmd
                dq_actual = dq_cmd
                t = i * dt
            else:
                arm.setArmCmd(q_cmd, dq_cmd, tau)
                if not args.no_gripper:
                    arm.setGripperCmd(gripper_cmd, arm.gripperQd, arm.gripperTau)
                arm.sendRecv()
                q_actual = vec6(arm.lowstate.getQ(), "q_actual")
                dq_actual = vec6(arm.lowstate.getQd(), "dq_actual")
                t = time.perf_counter() - t0

            writer.writerow(
                [
                    t,
                    *q_cmd.tolist(),
                    *dq_cmd.tolist(),
                    *ddq_cmd.tolist(),
                    *tau.tolist(),
                    *q_actual.tolist(),
                    *dq_actual.tolist(),
                    gripper_cmd,
                    alpha,
                ]
            )

            max_tau = np.maximum(max_tau, np.abs(tau))
            final_q = q_actual
            steps += 1

            if arm is not None:
                next_tick += dt
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.perf_counter()

    return steps, final_q, max_tau


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.duration_steps <= 0:
        raise ValueError("--duration-steps must be positive")
    if args.dt is not None and args.dt <= 0.0:
        raise ValueError("--dt must be positive")

    target_pos = parse_vec6(args.target)
    unitree_arm_interface = load_unitree_interface()

    np.set_printoptions(precision=6, suppress=True)

    if args.offline:
        dt = 0.002 if args.dt is None else float(args.dt)
        q_start = np.zeros(NDOF) if args.start_q is None else parse_vec6(args.start_q)
        model = make_standalone_model(unitree_arm_interface, has_gripper=not args.no_gripper)
        steps, final_q, max_tau = write_trace(
            args=args,
            arm_model=model,
            target_pos=target_pos,
            q_start=q_start,
            dt=dt,
        )
        print("steps =", steps)
        print("final_q =", np.array2string(final_q, precision=6, suppress_small=False))
        print("actual_delta =", np.array2string(final_q - q_start, precision=6, suppress_small=False))
        print("max |tau| =", np.array2string(max_tau, precision=6, suppress_small=False))
        print("CSV path =", args.csv_log)
        return 0

    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    arm = unitree_arm_interface.ArmInterface(hasGripper=not args.no_gripper)
    arm_model = arm._ctrlComp.armModel
    arm.setFsmLowcmd()

    dt = float(arm._ctrlComp.dt if args.dt is None else args.dt)
    q_start = vec6(arm.lowstate.getQ(), "q_start")

    try:
        steps, final_q, max_tau = write_trace(
            args=args,
            arm_model=arm_model,
            target_pos=target_pos,
            q_start=q_start,
            dt=dt,
            arm=arm,
            stop_requested_fn=lambda: stop_requested,
        )
    finally:
        if not args.no_back_to_start:
            print("calling backToStart()")
            arm.loopOn()
            arm.backToStart()
            arm.loopOff()

    print("steps =", steps)
    print("final_q =", np.array2string(final_q, precision=6, suppress_small=False))
    print("actual_delta =", np.array2string(final_q - q_start, precision=6, suppress_small=False))
    print("max |tau| =", np.array2string(max_tau, precision=6, suppress_small=False))
    print("CSV path =", args.csv_log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
