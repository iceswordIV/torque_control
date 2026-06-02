#!/usr/bin/env python3
"""Offline closed-loop simulation for the Z1 computed-torque controller."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

from controller import DEFAULT_WN, DEFAULT_ZETA, resolve_gains
from test_controller import TEST_CONTROLLER_MODES, compute_test_tau
from trajectory import NDOF, build_goal, parse_vec6, quintic_trajectory, scurve_trajectory
from z1_analytic_dynamics import dynamics_analytic

DEFAULT_TARGET = "0 1.5 -1.0 -0.54 0 0"
TEST_CONTROLLER_CHOICES = ["none", *TEST_CONTROLLER_MODES]


def parse_vec_any(text: str) -> np.ndarray:
    import re

    parts = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    return np.array([float(p) for p in parts], dtype=float)


def parse_gain_text(text):
    if text is None:
        return None
    values = parse_vec_any(text)
    if values.size == NDOF:
        return values
    if values.size == NDOF * NDOF:
        return values.reshape(NDOF, NDOF)
    raise ValueError(f"gain must contain 6 or 36 values, got {values.size}")


def default_csv_path(prefix: str) -> str:
    return str(Path("logs") / f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t"]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + [f"q_des_{i + 1}" for i in range(NDOF)]
        + [f"dq_des_{i + 1}" for i in range(NDOF)]
        + [f"ddq_des_{i + 1}" for i in range(NDOF)]
        + [f"tau_{i + 1}" for i in range(NDOF)]
    )


def print_summary(args, steps: int, q_start: np.ndarray, q_goal: np.ndarray, maxes: dict) -> None:
    print("dt =", args.dt)
    print("duration =", args.duration)
    print("move_time =", args.move_time)
    print("number of steps =", steps)
    print("trajectory mode =", args.mode)
    print("trajectory_profile =", args.trajectory_profile)
    print("q_start =", np.array2string(q_start, precision=6, suppress_small=False))
    print("q_goal =", np.array2string(q_goal, precision=6, suppress_small=False))
    print("max |q_des - q_start| =", np.array2string(maxes["q_delta"], precision=6, suppress_small=False))
    print("max |dq_des| =", np.array2string(maxes["dq_des"], precision=6, suppress_small=False))
    print("max |ddq_des| =", np.array2string(maxes["ddq_des"], precision=6, suppress_small=False))
    print("max |tau| =", np.array2string(maxes["tau"], precision=6, suppress_small=False))
    print("CSV path =", args.csv_log)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline Z1 computed-torque simulation", allow_abbrev=False)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--mode", choices=["hold_current", "one_joint_relative", "full_pose_absolute", "scaled_pose"], default="hold_current")
    parser.add_argument("--joint", type=int, default=1)
    parser.add_argument("--angle-deg", "--angle", dest="angle_deg", type=float, default=0.0)
    parser.add_argument("--target", type=str, default=DEFAULT_TARGET)
    parser.add_argument("--scale", type=float, default=0.25)
    parser.add_argument("--move-time", type=float, default=5.0)
    parser.add_argument("--hold-time", type=float, default=0.0)
    parser.add_argument("--trajectory-profile", choices=["quintic", "scurve"], default="quintic")
    parser.add_argument("--kp", type=str, default=None)
    parser.add_argument("--kd", type=str, default=None)
    parser.add_argument("--wn", type=str, default=None)
    parser.add_argument("--zeta", type=str, default=None)
    parser.add_argument("--test-controller", choices=TEST_CONTROLLER_CHOICES, default="none")
    parser.add_argument("--csv-log", type=str, default=default_csv_path("sim"))
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")

    target = parse_vec6(args.target)
    kp = parse_gain_text(args.kp)
    kd = parse_gain_text(args.kd)
    wn = DEFAULT_WN if args.wn is None else parse_vec6(args.wn)
    zeta = DEFAULT_ZETA if args.zeta is None else parse_vec6(args.zeta)
    trajectory_fn = quintic_trajectory if args.trajectory_profile == "quintic" else scurve_trajectory
    if args.test_controller == "none":
        Kp, Kd = resolve_gains(kp=kp, kd=kd, wn=wn, zeta=zeta)

    q_start = np.zeros(NDOF)
    q_goal = build_goal(q_start, args.mode, joint=args.joint, angle_deg=args.angle_deg, target=target, scale=args.scale)
    q = q_start.copy()
    dq = np.zeros(NDOF)
    damping = np.diag([0.02, 0.02, 0.02, 0.01, 0.01, 0.005])

    Path(args.csv_log).parent.mkdir(parents=True, exist_ok=True)
    steps = int(round(args.duration / args.dt))
    maxes = {
        "q_delta": np.zeros(NDOF),
        "dq_des": np.zeros(NDOF),
        "ddq_des": np.zeros(NDOF),
        "tau": np.zeros(NDOF),
    }

    with open(args.csv_log, "w", newline="") as f:
        writer = csv.writer(f)
        write_header(writer)
        for k in range(steps + 1):
            t = k * args.dt
            q_des, dq_des, ddq_des = trajectory_fn(q_start, q_goal, t - args.hold_time, args.move_time)
            M, C, N, _ = dynamics_analytic(q, dq)
            if args.test_controller == "none":
                e = q_des - q
                de = dq_des - dq
                # Computed torque puts feedback inside M(q): tau = M(q) @
                # (ddq_des + Kd @ de + Kp @ e) + C(q,dq) @ dq + N(q).
                ddq_cmd = ddq_des + Kd @ de + Kp @ e
                tau = M @ ddq_cmd + C @ dq + N
            else:
                # Diagnostic modes are intentionally separate from the main
                # computed-torque path. Augmented PD uses model feedforward plus
                # direct torque PD feedback, so the PD term is not scaled by M(q).
                tau = compute_test_tau(
                    q,
                    dq,
                    q_des,
                    dq_des,
                    ddq_des=ddq_des,
                    mode=args.test_controller,
                    kp=kp,
                    kd=kd,
                )

            writer.writerow([t, *q, *dq, *q_des, *dq_des, *ddq_des, *tau])
            maxes["q_delta"] = np.maximum(maxes["q_delta"], np.abs(q_des - q_start))
            maxes["dq_des"] = np.maximum(maxes["dq_des"], np.abs(dq_des))
            maxes["ddq_des"] = np.maximum(maxes["ddq_des"], np.abs(ddq_des))
            maxes["tau"] = np.maximum(maxes["tau"], np.abs(tau))

            if k == steps:
                break
            qdd = np.linalg.solve(M, tau - C @ dq - N - damping @ dq)
            dq = dq + qdd * args.dt
            q = q + dq * args.dt

    print_summary(args, steps + 1, q_start, q_goal, maxes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
