#!/usr/bin/env python3
"""Offline torque preview and feedforward/computed-torque comparison."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from controller import DEFAULT_WN, DEFAULT_ZETA, resolve_gains
from trajectory import NDOF, build_goal, parse_vec6, quintic_trajectory, scurve_trajectory
from z1_analytic_dynamics import dynamics_analytic

DEFAULT_TARGET = "0 1.5 -1.0 -0.54 0 0"


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


def field(row: Dict[str, str], candidates: List[str]) -> str:
    for key in candidates:
        if key in row and row[key] != "":
            return row[key]
    raise KeyError(f"missing any of columns {candidates}")


def read_actual_csv(path: str, duration: float | None = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    t_values = []
    q_values = []
    dq_values = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(field(row, ["t", "time", "timestamp"]))
            if duration is not None and t > duration:
                continue
            q = np.array(
                [float(field(row, [f"q_actual_{i}", f"q_{i}", f"q{i}"])) for i in range(1, NDOF + 1)],
                dtype=float,
            )
            dq = np.array(
                [float(field(row, [f"dq_actual_{i}", f"dq_{i}", f"dq{i}"])) for i in range(1, NDOF + 1)],
                dtype=float,
            )
            t_values.append(t)
            q_values.append(q)
            dq_values.append(dq)
    if not t_values:
        raise ValueError(f"no usable rows found in {path}")
    return np.array(t_values), np.vstack(q_values), np.vstack(dq_values)


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t"]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + [f"q_des_{i + 1}" for i in range(NDOF)]
        + [f"dq_des_{i + 1}" for i in range(NDOF)]
        + [f"ddq_des_{i + 1}" for i in range(NDOF)]
        + [f"tau_ff_{i + 1}" for i in range(NDOF)]
        + [f"tau_ct_{i + 1}" for i in range(NDOF)]
        + [f"tau_diff_{i + 1}" for i in range(NDOF)]
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
    print("max |tau_ff| =", np.array2string(maxes["tau_ff"], precision=6, suppress_small=False))
    print("max |tau_ct| =", np.array2string(maxes["tau_ct"], precision=6, suppress_small=False))
    print("max |tau_ct - tau_ff| =", np.array2string(maxes["tau_diff"], precision=6, suppress_small=False))
    print("CSV path =", args.csv_log)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Preview and compare Z1 torque commands", allow_abbrev=False)
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
    parser.add_argument("--actual-csv", type=str, default=None)
    parser.add_argument("--csv-log", type=str, default=default_csv_path("preview"))
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
    Kp, Kd = resolve_gains(kp=kp, kd=kd, wn=wn, zeta=zeta)
    trajectory_fn = quintic_trajectory if args.trajectory_profile == "quintic" else scurve_trajectory

    if args.actual_csv:
        t_values, q_actual_values, dq_actual_values = read_actual_csv(args.actual_csv, args.duration)
        q_start = q_actual_values[0].copy()
    else:
        steps = int(round(args.duration / args.dt))
        t_values = np.array([k * args.dt for k in range(steps + 1)], dtype=float)
        q_actual_values = None
        dq_actual_values = None
        q_start = np.zeros(NDOF)

    q_goal = build_goal(q_start, args.mode, joint=args.joint, angle_deg=args.angle_deg, target=target, scale=args.scale)
    Path(args.csv_log).parent.mkdir(parents=True, exist_ok=True)
    maxes = {
        "q_delta": np.zeros(NDOF),
        "dq_des": np.zeros(NDOF),
        "ddq_des": np.zeros(NDOF),
        "tau_ff": np.zeros(NDOF),
        "tau_ct": np.zeros(NDOF),
        "tau_diff": np.zeros(NDOF),
    }

    with open(args.csv_log, "w", newline="") as f:
        writer = csv.writer(f)
        write_header(writer)
        for idx, t in enumerate(t_values):
            q_des, dq_des, ddq_des = trajectory_fn(q_start, q_goal, t - args.hold_time, args.move_time)
            M_des, C_des, N_des, _ = dynamics_analytic(q_des, dq_des)
            tau_ff = M_des @ ddq_des + C_des @ dq_des + N_des

            if q_actual_values is None:
                q_actual = q_des
                dq_actual = dq_des
                M_actual, C_actual, N_actual = M_des, C_des, N_des
            else:
                q_actual = q_actual_values[idx]
                dq_actual = dq_actual_values[idx]
                M_actual, C_actual, N_actual, _ = dynamics_analytic(q_actual, dq_actual)

            e = q_des - q_actual
            de = dq_des - dq_actual
            ddq_cmd = ddq_des + Kd @ de + Kp @ e
            tau_ct = M_actual @ ddq_cmd + C_actual @ dq_actual + N_actual
            tau_diff = tau_ct - tau_ff

            writer.writerow([t, *q_actual, *dq_actual, *q_des, *dq_des, *ddq_des, *tau_ff, *tau_ct, *tau_diff])
            maxes["q_delta"] = np.maximum(maxes["q_delta"], np.abs(q_des - q_start))
            maxes["dq_des"] = np.maximum(maxes["dq_des"], np.abs(dq_des))
            maxes["ddq_des"] = np.maximum(maxes["ddq_des"], np.abs(ddq_des))
            maxes["tau_ff"] = np.maximum(maxes["tau_ff"], np.abs(tau_ff))
            maxes["tau_ct"] = np.maximum(maxes["tau_ct"], np.abs(tau_ct))
            maxes["tau_diff"] = np.maximum(maxes["tau_diff"], np.abs(tau_diff))

    print_summary(args, len(t_values), q_start, q_goal, maxes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
