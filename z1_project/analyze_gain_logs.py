#!/usr/bin/env python3
"""Analyze Z1 gain-tuning CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

NDOF = 6


def read_csv_columns(path: str) -> Dict[str, np.ndarray]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []
    if not rows:
        raise ValueError(f"empty CSV: {path}")

    data: Dict[str, np.ndarray] = {}
    for key in fieldnames:
        if key is None:
            continue
        values: List[float] = []
        try:
            for row in rows:
                cell = row.get(key, "")
                if cell == "":
                    raise ValueError
                values.append(float(cell))
        except (TypeError, ValueError):
            continue
        data[key] = np.array(values, dtype=float)
    return data


def pick_matrix(data: Dict[str, np.ndarray], prefixes: List[str]) -> Tuple[Optional[np.ndarray], Optional[str]]:
    for prefix in prefixes:
        cols = []
        ok = True
        for i in range(1, NDOF + 1):
            found = None
            for name in (f"{prefix}_{i}", f"{prefix}{i}"):
                if name in data:
                    found = data[name]
                    break
            if found is None:
                ok = False
                break
            cols.append(found)
        if ok:
            return np.vstack(cols).T, prefix
    return None, None


def estimate_velocity(t: np.ndarray, q: np.ndarray) -> np.ndarray:
    if len(t) < 2:
        return np.zeros_like(q)
    dt = np.diff(t)
    dq = np.zeros_like(q)
    valid = dt > 0.0
    if np.any(valid):
        step_velocity = np.zeros((len(t) - 1, NDOF), dtype=float)
        step_velocity[valid] = np.diff(q, axis=0)[valid] / dt[valid, None]
        dq[1:] = step_velocity
        dq[0] = dq[1]
    return dq


def overshoot_ratio(q_actual: np.ndarray, q_des: np.ndarray, motion_threshold: float) -> np.ndarray:
    ratio = np.zeros(NDOF, dtype=float)
    for j in range(NDOF):
        start = q_des[0, j]
        target = q_des[-1, j]
        commanded = target - start
        if abs(commanded) <= motion_threshold:
            continue
        if commanded > 0.0:
            overshoot = max(0.0, float(np.max(q_actual[:, j] - target)))
        else:
            overshoot = max(0.0, float(np.max(target - q_actual[:, j])))
        ratio[j] = overshoot / abs(commanded)
    return ratio


def first_motion_index(
    q_actual: np.ndarray,
    dq_actual: np.ndarray,
    joint: int,
    motion_threshold: float,
    velocity_threshold: float,
) -> Optional[int]:
    moved = np.abs(q_actual[:, joint] - q_actual[0, joint]) >= motion_threshold
    moved |= np.abs(dq_actual[:, joint]) >= velocity_threshold
    indices = np.flatnonzero(moved)
    if indices.size == 0:
        return None
    return int(indices[0])


def is_large_error(final_error: float, command: float, args) -> bool:
    threshold = max(args.final_error_abs, args.final_error_ratio * abs(command))
    return abs(final_error) > threshold


def is_unstable(
    command: float,
    max_error: float,
    max_velocity: float,
    max_torque: float,
    overshoot: float,
    args,
) -> Tuple[bool, str]:
    reasons: List[str] = []
    values = [command, max_error, max_velocity, overshoot]
    if np.isfinite(max_torque):
        values.append(max_torque)
    if not np.all(np.isfinite(values)):
        reasons.append("nonfinite")

    if abs(command) > args.motion_threshold:
        error_limit = max(args.final_error_abs, args.unstable_error_ratio * abs(command))
        if max_error > error_limit:
            reasons.append("tracking")
    if overshoot > args.unstable_overshoot:
        reasons.append("overshoot")
    if max_velocity > args.unstable_velocity:
        reasons.append("velocity")
    if np.isfinite(max_torque) and max_torque > args.unstable_torque:
        reasons.append("torque")

    if reasons:
        return False, ",".join(reasons)
    return True, "ok"


def recommendation(
    command: float,
    actual_final_motion: float,
    max_actual_motion: float,
    final_error: float,
    max_velocity: float,
    max_torque: float,
    overshoot: float,
    tau: Optional[np.ndarray],
    q_actual: np.ndarray,
    dq_actual: np.ndarray,
    joint: int,
    args,
) -> str:
    command_abs = abs(command)
    if command_abs <= args.motion_threshold:
        return "no commanded motion"

    recs: List[str] = []
    no_motion_limit = max(args.motion_threshold, args.no_motion_ratio * command_abs)
    no_motion = abs(actual_final_motion) <= no_motion_limit and max_actual_motion <= no_motion_limit

    if no_motion:
        if tau is None or not np.isfinite(max_torque):
            recs.append("joint did not move; add torque logging to estimate breakaway torque")
        else:
            idx = first_motion_index(q_actual, dq_actual, joint, args.motion_threshold, args.small_velocity)
            if idx is None:
                recs.append(f"joint did not move; breakaway torque is above {max_torque:.3g} Nm")
            else:
                recs.append(f"breakaway torque about {abs(tau[idx, joint]):.3g} Nm")

    if is_large_error(final_error, command, args) and max_velocity < args.small_velocity:
        recs.append("increase Kp")
    if overshoot > args.high_overshoot or max_velocity > args.high_velocity:
        recs.append("increase Kd or decrease Kp")
    if np.isfinite(max_torque) and max_torque > args.high_torque:
        recs.append("reduce Kp or add torque limit")

    if not recs:
        return "no gain change indicated"
    return "; ".join(recs)


def analyze_file(path: str, args) -> bool:
    data = read_csv_columns(path)
    if "t" not in data:
        raise ValueError(f"{path}: missing t column")

    q_actual, q_actual_label = pick_matrix(data, ["q_actual", "q"])
    q_des, _ = pick_matrix(data, ["q_des"])
    dq_actual, dq_actual_label = pick_matrix(data, ["dq_actual", "dq"])
    tau, tau_label = pick_matrix(data, ["tau_total", "tau", "tau_ct"])

    if q_actual is None:
        raise ValueError(f"{path}: missing q_actual/q columns")
    if q_des is None:
        raise ValueError(f"{path}: missing q_des columns")

    t = data["t"]
    if dq_actual is None:
        dq_actual = estimate_velocity(t, q_actual)
        dq_actual_label = "estimated"

    commanded = q_des[-1] - q_des[0]
    actual_final = q_actual[-1] - q_actual[0]
    final_error = q_des[-1] - q_actual[-1]
    tracking_error = q_des - q_actual
    max_error = np.max(np.abs(tracking_error), axis=0)
    max_velocity = np.max(np.abs(dq_actual), axis=0)
    max_actual_motion = np.max(np.abs(q_actual - q_actual[0]), axis=0)
    overshoot = overshoot_ratio(q_actual, q_des, args.motion_threshold)
    if tau is None:
        max_torque = np.full(NDOF, np.nan)
    else:
        max_torque = np.max(np.abs(tau), axis=0)

    stable = []
    reason = []
    recs = []
    for j in range(NDOF):
        joint_stable, joint_reason = is_unstable(
            commanded[j],
            max_error[j],
            max_velocity[j],
            max_torque[j],
            overshoot[j],
            args,
        )
        stable.append(joint_stable)
        reason.append(joint_reason)
        recs.append(
            recommendation(
                commanded[j],
                actual_final[j],
                max_actual_motion[j],
                final_error[j],
                max_velocity[j],
                max_torque[j],
                overshoot[j],
                tau,
                q_actual,
                dq_actual,
                j,
                args,
            )
        )

    overall_stable = all(stable)
    print(f"\n== {path} ==")
    print(f"samples: {len(t)}")
    print(f"position source: {q_actual_label}")
    print(f"velocity source: {dq_actual_label}")
    print(f"torque source: {tau_label or 'none'}")
    print(f"overall: {'stable' if overall_stable else 'unstable'}")
    print(
        "joint  cmd_rad    actual_rad final_err  max_err    max_vel    max_tau    overshoot  status"
    )
    for j in range(NDOF):
        torque_text = "nan" if not np.isfinite(max_torque[j]) else f"{max_torque[j]:9.4f}"
        status = "stable" if stable[j] else f"unstable:{reason[j]}"
        print(
            f"J{j + 1:<2} "
            f"{commanded[j]:9.4f} "
            f"{actual_final[j]:10.4f} "
            f"{final_error[j]:9.4f} "
            f"{max_error[j]:9.4f} "
            f"{max_velocity[j]:9.4f} "
            f"{torque_text} "
            f"{overshoot[j]:10.3f} "
            f"{status}"
        )

    print("recommendations:")
    for j, rec in enumerate(recs, start=1):
        print(f"  J{j}: {rec}")
    return overall_stable


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze Z1 gain-tuning CSV logs", allow_abbrev=False)
    parser.add_argument("csv_paths", nargs="+")
    parser.add_argument("--motion-threshold", type=float, default=1e-3, help="minimum meaningful joint motion [rad]")
    parser.add_argument("--final-error-abs", type=float, default=math.radians(2.0), help="large final error absolute threshold [rad]")
    parser.add_argument("--final-error-ratio", type=float, default=0.25, help="large final error threshold as fraction of commanded motion")
    parser.add_argument("--small-velocity", type=float, default=0.02, help="small measured velocity threshold [rad/s]")
    parser.add_argument("--high-velocity", type=float, default=1.0, help="velocity threshold for damping recommendation [rad/s]")
    parser.add_argument("--high-overshoot", type=float, default=0.20, help="overshoot ratio threshold for damping recommendation")
    parser.add_argument("--high-torque", type=float, default=20.0, help="torque threshold for torque-limit recommendation [Nm]")
    parser.add_argument("--no-motion-ratio", type=float, default=0.05, help="actual motion below this fraction counts as no motion")
    parser.add_argument("--unstable-error-ratio", type=float, default=1.5, help="max error ratio that marks a moving test unstable")
    parser.add_argument("--unstable-overshoot", type=float, default=1.0, help="overshoot ratio that marks a test unstable")
    parser.add_argument("--unstable-velocity", type=float, default=math.pi, help="velocity that marks a test unstable [rad/s]")
    parser.add_argument("--unstable-torque", type=float, default=50.0, help="torque that marks a test unstable [Nm]")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    for path in args.csv_paths:
        analyze_file(path, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
