#!/usr/bin/env python3
"""Offline plant simulation with the same command shape as torque_main.py.

This script integrates an idealized Z1 plant using the same computed-torque
law as the runtime controller and writes the same CSV columns as torque_main.py.
It is useful for comparing ideal model behavior against ROS/Gazebo logs.
"""

from __future__ import annotations

import argparse
import csv
import re
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from controller import DEFAULT_WN, DEFAULT_ZETA, resolve_gains
from robot_io import default_runtime_dir
from trajectory import NDOF, build_goal, parse_vec6, quintic_trajectory, scurve_trajectory
from z1_analytic_dynamics import dynamics_analytic

DEFAULT_TARGET = "0 1.5 -1.0 -0.54 0 0"
DEFAULT_DAMPING = "0.02 0.02 0.02 0.01 0.01 0.005"


def parse_vec_any(text: str) -> np.ndarray:
    parts = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    return np.array([float(p) for p in parts], dtype=float)


def parse_gain_text(text: Optional[str]):
    if text is None:
        return None
    values = parse_vec_any(text)
    if values.size == NDOF:
        return values
    if values.size == NDOF * NDOF:
        return values.reshape(NDOF, NDOF)
    raise ValueError(f"gain must contain 6 or 36 values, got {values.size}")


def parse_vec6_text(text: Optional[str], name: str) -> Optional[np.ndarray]:
    if text is None:
        return None
    values = parse_vec_any(text)
    if values.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {values.size}")
    return values


def default_csv_path(prefix: str) -> str:
    return str(Path("logs") / f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")


def ensure_parent(path: str) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)


def field(row: Dict[str, str], candidates: List[str]) -> str:
    for key in candidates:
        if key in row and row[key] != "":
            return row[key]
    raise KeyError(f"missing any of columns {candidates}")


def csv_state_from_row(row: Dict[str, str]) -> Tuple[np.ndarray, np.ndarray]:
    q = np.array(
        [float(field(row, [f"q_actual_{i}", f"q_{i}", f"q{i}"])) for i in range(1, NDOF + 1)],
        dtype=float,
    )
    dq = np.array(
        [float(field(row, [f"dq_actual_{i}", f"dq_{i}", f"dq{i}"])) for i in range(1, NDOF + 1)],
        dtype=float,
    )
    return q, dq


def read_csv_first_state(path: str) -> Tuple[np.ndarray, np.ndarray]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            return csv_state_from_row(row)
    raise ValueError(f"no usable rows found in {path}")


def read_csv_times_and_first_state(path: str, duration: Optional[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    t_values = []
    first_q = None
    first_dq = None
    first_t = None
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(field(row, ["t", "time", "timestamp"]))
            if first_t is None:
                first_t = t
                first_q, first_dq = csv_state_from_row(row)
            rel_t = t - first_t
            if duration is not None and rel_t > duration:
                continue
            t_values.append(rel_t)

    if first_q is None or first_dq is None or not t_values:
        raise ValueError(f"no usable rows found in {path}")
    return np.array(t_values, dtype=float), first_q, first_dq


def fixed_time_values(dt: float, duration: float) -> np.ndarray:
    steps = int(round(duration / dt))
    return np.array([k * dt for k in range(steps + 1)], dtype=float)


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
    joint_index = args.joint - 1
    rad_to_deg = 180.0 / np.pi
    print("dt =", args.dt)
    print("duration =", args.duration)
    print("move_time =", args.move_time)
    print("number of steps =", steps)
    print("time source =", maxes["time_source"])
    print("trajectory mode =", args.mode)
    print("trajectory_profile =", args.trajectory_profile)
    print("q_start =", np.array2string(q_start, precision=6, suppress_small=False))
    print("q_goal =", np.array2string(q_goal, precision=6, suppress_small=False))
    print("q_final =", np.array2string(maxes["q_final"], precision=6, suppress_small=False))
    print("max |q_des - q_start| =", np.array2string(maxes["q_delta"], precision=6, suppress_small=False))
    print("max |dq_des| =", np.array2string(maxes["dq_des"], precision=6, suppress_small=False))
    print("max |ddq_des| =", np.array2string(maxes["ddq_des"], precision=6, suppress_small=False))
    print("max |tau| =", np.array2string(maxes["tau"], precision=6, suppress_small=False))
    print(
        f"joint {args.joint} desired move =",
        f"{(q_goal[joint_index] - q_start[joint_index]) * rad_to_deg:.3f} deg",
    )
    print(
        f"joint {args.joint} actual move =",
        f"{(maxes['q_final'][joint_index] - q_start[joint_index]) * rad_to_deg:.3f} deg",
    )
    print(
        f"joint {args.joint} final error =",
        f"{(q_goal[joint_index] - maxes['q_final'][joint_index]) * rad_to_deg:.3f} deg",
    )
    print(
        f"joint {args.joint} max abs error =",
        f"{maxes['max_abs_error'][joint_index] * rad_to_deg:.3f} deg",
    )
    if maxes["time_span"] > 0.0:
        print("effective sample rate =", f"{(steps - 1) / maxes['time_span']:.1f} Hz")
    print("CSV path =", args.csv_log)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline Z1 computed-torque plant simulation",
        allow_abbrev=False,
    )
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--duration", type=float, default=None)
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
    parser.add_argument("--csv-log", type=str, default=default_csv_path("offline_runtime"))
    parser.add_argument("--runtime-dir", type=str, default=str(default_runtime_dir()), help="accepted for torque_main.py CLI compatibility; ignored")
    parser.add_argument("--state-timeout", type=float, default=1.0, help="accepted for torque_main.py CLI compatibility; ignored")
    parser.add_argument("--feedback-watchdog-timeout", type=float, default=0.75, help="accepted for torque_main.py CLI compatibility; ignored")
    parser.add_argument("--feedback-watchdog-command-threshold", type=float, default=0.01, help="accepted for torque_main.py CLI compatibility; ignored")
    parser.add_argument("--feedback-watchdog-state-threshold", type=float, default=1e-4, help="accepted for torque_main.py CLI compatibility; ignored")
    parser.add_argument("--disable-feedback-watchdog", action="store_true", help="accepted for torque_main.py CLI compatibility; ignored")
    parser.add_argument("--initial-q", type=str, default=None, help="six initial joint positions in radians")
    parser.add_argument("--initial-dq", type=str, default=None, help="six initial joint velocities in rad/s")
    parser.add_argument("--initial-from-csv", type=str, default=None, help="use first q/dq row from an existing torque_main.py CSV")
    parser.add_argument("--match-csv", type=str, default=None, help="use first q/dq row and time samples from an existing torque_main.py CSV")
    parser.add_argument("--plant-damping", type=str, default=DEFAULT_DAMPING, help="six viscous damping values applied in the offline plant")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.duration is not None and args.duration < 0.0:
        raise ValueError("--duration must be non-negative")

    target = parse_vec6(args.target)
    kp = parse_gain_text(args.kp)
    kd = parse_gain_text(args.kd)
    wn = DEFAULT_WN if args.wn is None else parse_vec6(args.wn)
    zeta = DEFAULT_ZETA if args.zeta is None else parse_vec6(args.zeta)
    Kp, Kd = resolve_gains(kp=kp, kd=kd, wn=wn, zeta=zeta)
    trajectory_fn = quintic_trajectory if args.trajectory_profile == "quintic" else scurve_trajectory
    damping_values = parse_vec6_text(args.plant_damping, "--plant-damping")
    damping = np.diag(damping_values)

    q_start = np.zeros(NDOF)
    dq_start = np.zeros(NDOF)
    if args.initial_from_csv:
        q_start, dq_start = read_csv_first_state(args.initial_from_csv)
    if args.initial_q is not None:
        q_start = parse_vec6_text(args.initial_q, "--initial-q")
    if args.initial_dq is not None:
        dq_start = parse_vec6_text(args.initial_dq, "--initial-dq")

    time_source = "fixed dt"
    if args.match_csv:
        time_values, q_start, dq_start = read_csv_times_and_first_state(args.match_csv, args.duration)
        time_source = args.match_csv
        args.duration = float(time_values[-1])
    else:
        if args.duration is None:
            args.duration = 6.0
        time_values = fixed_time_values(args.dt, args.duration)

    q_goal = build_goal(q_start, args.mode, joint=args.joint, angle_deg=args.angle_deg, target=target, scale=args.scale)
    ensure_parent(args.csv_log)

    q = q_start.copy()
    dq = dq_start.copy()
    maxes = {
        "q_delta": np.zeros(NDOF),
        "dq_des": np.zeros(NDOF),
        "ddq_des": np.zeros(NDOF),
        "tau": np.zeros(NDOF),
        "max_abs_error": np.zeros(NDOF),
        "q_final": q.copy(),
        "time_source": time_source,
        "time_span": float(time_values[-1] - time_values[0]) if len(time_values) > 1 else 0.0,
    }

    with open(args.csv_log, "w", newline="") as f:
        writer = csv.writer(f)
        write_header(writer)
        for idx, t in enumerate(time_values):
            q_des, dq_des, ddq_des = trajectory_fn(q_start, q_goal, t - args.hold_time, args.move_time)
            M, C, N, _ = dynamics_analytic(q, dq)
            e = q_des - q
            de = dq_des - dq
            ddq_cmd = ddq_des + Kd @ de + Kp @ e
            tau = M @ ddq_cmd + C @ dq + N

            writer.writerow([t, *q, *dq, *q_des, *dq_des, *ddq_des, *tau])
            maxes["q_delta"] = np.maximum(maxes["q_delta"], np.abs(q_des - q_start))
            maxes["dq_des"] = np.maximum(maxes["dq_des"], np.abs(dq_des))
            maxes["ddq_des"] = np.maximum(maxes["ddq_des"], np.abs(ddq_des))
            maxes["tau"] = np.maximum(maxes["tau"], np.abs(tau))
            maxes["max_abs_error"] = np.maximum(maxes["max_abs_error"], np.abs(q_des - q))
            maxes["q_final"] = q.copy()

            if idx == len(time_values) - 1:
                break

            step_dt = float(time_values[idx + 1] - t)
            if step_dt <= 0.0:
                raise ValueError(f"time samples must be strictly increasing, got dt={step_dt} at index {idx}")
            qdd = np.linalg.solve(M, tau - C @ dq - N - damping @ dq)
            dq = dq + qdd * step_dt
            q = q + dq * step_dt

    print_summary(args, len(time_values), q_start, q_goal, maxes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
