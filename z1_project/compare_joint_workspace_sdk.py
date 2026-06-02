#!/usr/bin/env python3
"""Compare pure-torque joint workspace logs against SDK lowcmd logs."""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

import numpy as np

from trajectory import NDOF

PURE_RE = re.compile(
    r"workspace_j(?P<joint>\d+)_(?P<angle>(?:neg|pos)?\d+(?:\.\d+)?deg)_(?P<label>augpd_nofric|augpd_fric|computed_nofric|computed_fric)\.csv$"
)
SDK_RE = re.compile(
    r"workspace_j(?P<joint>\d+)_(?P<angle>(?:neg|pos)?\d+(?:\.\d+)?deg)_sdk_lowcmd\.csv$"
)


def parse_angle(label: str) -> float:
    sign = -1.0 if label.startswith("neg") else 1.0
    value = label.replace("neg", "").replace("pos", "").replace("deg", "")
    return sign * float(value)


def matrix(rows: list[dict[str, str]], prefix: str) -> np.ndarray:
    return np.array([[float(row[f"{prefix}_{i}"]) for i in range(1, NDOF + 1)] for row in rows], dtype=float)


def first_existing_prefix(fieldnames: list[str], candidates: list[str]) -> str | None:
    names = set(fieldnames)
    for prefix in candidates:
        if all(f"{prefix}_{i}" in names for i in range(1, NDOF + 1)):
            return prefix
    return None


def analyze_csv(path: Path, *, source: str, label: str, joint: int, angle_deg: float) -> dict:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = [row for row in reader]
        fieldnames = reader.fieldnames or []

    if not rows:
        raise ValueError(f"empty CSV: {path}")

    outbound = [row for row in rows if row.get("phase", "outbound") == "outbound"]
    if not outbound:
        outbound = rows

    q_des_prefix = first_existing_prefix(fieldnames, ["q_des", "q_cmd"])
    dq_des_prefix = first_existing_prefix(fieldnames, ["dq_des", "dq_cmd"])
    tau_cmd_prefix = first_existing_prefix(fieldnames, ["tau", "tau_total", "tau_sdk_cmd", "tau_cmd"])
    tau_state_prefix = first_existing_prefix(fieldnames, ["tau_state", "tau_feedback"])
    if q_des_prefix is None or dq_des_prefix is None:
        raise ValueError(f"missing desired q/dq columns in {path}")

    j = joint - 1
    q_des = matrix(outbound, q_des_prefix)
    q_actual = matrix(outbound, "q_actual")
    dq_actual = matrix(outbound, "dq_actual")
    error = q_des - q_actual

    desired_delta = q_des[-1, j] - q_des[0, j]
    actual_delta = q_actual[-1, j] - q_actual[0, j]
    achieved_pct = 100.0 * actual_delta / desired_delta if abs(desired_delta) > 1e-12 else float("nan")
    tau_cmd_max = float("nan")
    tau_state_max = float("nan")
    if tau_cmd_prefix is not None:
        tau_cmd_max = float(np.max(np.abs(matrix(outbound, tau_cmd_prefix)[:, j])))
    if tau_state_prefix is not None:
        tau_state_max = float(np.max(np.abs(matrix(outbound, tau_state_prefix)[:, j])))

    t0 = float(outbound[0]["t"])
    t1 = float(outbound[-1]["t"])
    return {
        "source": source,
        "label": label,
        "file": str(path),
        "joint": joint,
        "angle_deg": angle_deg,
        "outbound_samples": len(outbound),
        "outbound_duration_s": t1 - t0,
        "outbound_rate_hz": len(outbound) / max(t1 - t0, 1e-9),
        "desired_delta_deg": math.degrees(desired_delta),
        "actual_delta_deg": math.degrees(actual_delta),
        "achieved_pct": achieved_pct,
        "final_error_deg": math.degrees(error[-1, j]),
        "max_abs_error_deg": math.degrees(float(np.max(np.abs(error[:, j])))),
        "rms_error_deg": math.degrees(float(np.sqrt(np.mean(error[:, j] ** 2)))),
        "max_abs_tau_cmd_or_tau_joint": tau_cmd_max,
        "max_abs_tau_state_joint": tau_state_max,
        "max_abs_dq_actual_joint": float(np.max(np.abs(dq_actual[:, j]))),
    }


def collect(pure_dir: Path, sdk_dir: Path) -> list[dict]:
    rows: list[dict] = []
    if pure_dir.exists():
        for path in sorted(pure_dir.glob("*.csv")):
            match = PURE_RE.match(path.name)
            if not match:
                continue
            joint = int(match.group("joint"))
            angle = parse_angle(match.group("angle"))
            rows.append(analyze_csv(path, source="pure_torque", label=match.group("label"), joint=joint, angle_deg=angle))

    if sdk_dir.exists():
        for path in sorted(sdk_dir.glob("*.csv")):
            if path.name.startswith("summary"):
                continue
            match = SDK_RE.match(path.name)
            if not match:
                continue
            joint = int(match.group("joint"))
            angle = parse_angle(match.group("angle"))
            rows.append(analyze_csv(path, source="sdk", label="sdk_lowcmd", joint=joint, angle_deg=angle))
    return rows


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare joint workspace pure-torque logs with SDK logs")
    parser.add_argument("--pure-dir", default="logs/eval_joint_workspace")
    parser.add_argument("--sdk-dir", default="logs/eval_joint_workspace_sdk")
    parser.add_argument("--out", default="logs/joint_workspace_sdk_compare.csv")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    rows = collect(Path(args.pure_dir), Path(args.sdk_dir))
    if not rows:
        raise SystemExit("no matching CSV files found")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("wrote", out_path)
    print("rows =", len(rows))

    # Compact console view: SDK vs best pure-torque result per joint/angle.
    index: dict[tuple[int, float], list[dict]] = {}
    for row in rows:
        index.setdefault((int(row["joint"]), float(row["angle_deg"])), []).append(row)

    print()
    print("joint angle  sdk_maxerr  sdk_achieved  best_pure(label:maxerr/achieved)")
    for key in sorted(index):
        group = index[key]
        sdk = [row for row in group if row["source"] == "sdk"]
        pure = [row for row in group if row["source"] == "pure_torque"]
        if not sdk or not pure:
            continue
        sdk_row = sdk[0]
        best = min(pure, key=lambda row: float(row["max_abs_error_deg"]))
        print(
            f"J{key[0]:d} {key[1]:+g} "
            f"{float(sdk_row['max_abs_error_deg']):9.3f} "
            f"{float(sdk_row['achieved_pct']):11.2f}% "
            f"{best['label']}:{float(best['max_abs_error_deg']):.3f}/{float(best['achieved_pct']):.2f}%"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
