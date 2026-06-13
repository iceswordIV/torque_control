#!/usr/bin/env python3
"""Compare sim controller batch logs against Unitree SDK lowcmd logs."""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

import numpy as np

from trajectory import NDOF

SIM_RE = re.compile(
    r"(?P<controller>augpd_nofric|augpd_fric2p5|computed_torque_baseline|cpid_fric2p5)"
    r"_j(?P<joint>\d+)_(?P<sign>pos|neg)(?P<angle>\d+(?:\.\d+)?)\.csv$"
)
SDK_RE = re.compile(
    r"workspace_j(?P<joint>\d+)_(?P<sign>pos|neg)(?P<angle>\d+(?:\.\d+)?)deg_sdk_lowcmd\.csv$"
)


def parse_signed_angle(sign: str, angle_text: str) -> float:
    sign_value = -1.0 if sign == "neg" else 1.0
    return sign_value * float(angle_text)


def matrix(rows: list[dict[str, str]], prefix: str) -> np.ndarray:
    return np.array([[float(row[f"{prefix}_{i}"]) for i in range(1, NDOF + 1)] for row in rows], dtype=float)


def first_existing_prefix(fieldnames: list[str], candidates: list[str]) -> str | None:
    names = set(fieldnames)
    for prefix in candidates:
        if all(f"{prefix}_{i}" in names for i in range(1, NDOF + 1)):
            return prefix
    return None


def safe_joint_max(rows: list[dict[str, str]], prefix: str | None, joint_index: int) -> float:
    if prefix is None:
        return float("nan")
    try:
        return float(np.max(np.abs(matrix(rows, prefix)[:, joint_index])))
    except (KeyError, ValueError):
        return float("nan")


def analyze_csv(path: Path, *, source: str, controller: str, joint: int, angle_deg: float) -> dict:
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
    tau_cmd_prefix = first_existing_prefix(fieldnames, ["tau_total", "tau", "tau_sdk_cmd", "tau_cmd"])
    tau_state_prefix = first_existing_prefix(fieldnames, ["tau_state", "tau_feedback"])
    if q_des_prefix is None:
        raise ValueError(f"missing desired q columns in {path}")

    joint_index = joint - 1
    q_des = matrix(outbound, q_des_prefix)
    q_actual = matrix(outbound, "q_actual")
    dq_actual = matrix(outbound, "dq_actual")
    error = q_des - q_actual

    desired_delta = q_des[-1, joint_index] - q_des[0, joint_index]
    actual_delta = q_actual[-1, joint_index] - q_actual[0, joint_index]
    achieved_pct = 100.0 * actual_delta / desired_delta if abs(desired_delta) > 1e-12 else float("nan")
    t0 = float(outbound[0]["t"])
    t1 = float(outbound[-1]["t"])

    return {
        "source": source,
        "controller": controller,
        "joint": joint,
        "angle_deg": angle_deg,
        "outbound_samples": len(outbound),
        "outbound_duration_s": t1 - t0,
        "outbound_rate_hz": len(outbound) / max(t1 - t0, 1e-9),
        "desired_delta_deg": math.degrees(desired_delta),
        "actual_delta_deg": math.degrees(actual_delta),
        "achieved_pct": achieved_pct,
        "final_error_deg": math.degrees(error[-1, joint_index]),
        "max_abs_error_deg": math.degrees(float(np.max(np.abs(error[:, joint_index])))),
        "rms_error_deg": math.degrees(float(np.sqrt(np.mean(error[:, joint_index] ** 2)))),
        "max_abs_tau_cmd_joint": safe_joint_max(outbound, tau_cmd_prefix, joint_index),
        "max_abs_tau_state_joint": safe_joint_max(outbound, tau_state_prefix, joint_index),
        "max_abs_dq_actual_joint": float(np.max(np.abs(dq_actual[:, joint_index]))),
        "q_des_prefix": q_des_prefix,
        "tau_cmd_prefix": tau_cmd_prefix or "",
        "tau_state_prefix": tau_state_prefix or "",
        "file": str(path),
    }


def collect(sim_dir: Path, sdk_dir: Path) -> list[dict]:
    rows: list[dict] = []

    for path in sorted(sim_dir.glob("*.csv")):
        match = SIM_RE.match(path.name)
        if not match:
            continue
        rows.append(
            analyze_csv(
                path,
                source="sim",
                controller=match.group("controller"),
                joint=int(match.group("joint")),
                angle_deg=parse_signed_angle(match.group("sign"), match.group("angle")),
            )
        )

    if sdk_dir.exists():
        for path in sorted(sdk_dir.glob("*.csv")):
            if path.name.startswith("summary"):
                continue
            match = SDK_RE.match(path.name)
            if not match:
                continue
            rows.append(
                analyze_csv(
                    path,
                    source="sdk",
                    controller="sdk_lowcmd",
                    joint=int(match.group("joint")),
                    angle_deg=parse_signed_angle(match.group("sign"), match.group("angle")),
                )
            )

    return rows


def print_console_summary(rows: list[dict]) -> None:
    index: dict[tuple[int, float], list[dict]] = {}
    for row in rows:
        index.setdefault((int(row["joint"]), float(row["angle_deg"])), []).append(row)

    print()
    print("joint angle  sdk_maxerr sdk_achieved sdk_tau_state   best_sim(maxerr/achieved/tau)")
    for key in sorted(index):
        group = index[key]
        sdk_rows = [row for row in group if row["source"] == "sdk"]
        sim_rows = [row for row in group if row["source"] == "sim"]
        if not sim_rows:
            continue
        best = min(sim_rows, key=lambda row: float(row["max_abs_error_deg"]))
        if not sdk_rows:
            print(
                f"J{key[0]:d} {key[1]:+g}  missing SDK logs                  "
                f"{best['controller']}:{float(best['max_abs_error_deg']):.3f}/"
                f"{float(best['achieved_pct']):.2f}%/"
                f"{float(best['max_abs_tau_cmd_joint']):.3f}"
            )
            continue
        sdk = min(sdk_rows, key=lambda row: float(row["max_abs_error_deg"]))
        print(
            f"J{key[0]:d} {key[1]:+g} "
            f"{float(sdk['max_abs_error_deg']):10.3f} "
            f"{float(sdk['achieved_pct']):11.2f}% "
            f"{float(sdk['max_abs_tau_state_joint']):13.3f}   "
            f"{best['controller']}:{float(best['max_abs_error_deg']):.3f}/"
            f"{float(best['achieved_pct']):.2f}%/"
            f"{float(best['max_abs_tau_cmd_joint']):.3f}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare sim controller logs against SDK lowcmd logs")
    parser.add_argument("--sim-dir", required=True, help="directory from run_sim_joint_controller_compare.sh")
    parser.add_argument("--sdk-dir", required=True, help="directory from run_sdk_sim_joint_controller_compare.sh")
    parser.add_argument("--out", default=None, help="output CSV path; default is <sim-dir>/sdk_vs_sim_controller_compare.csv")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    sim_dir = Path(args.sim_dir)
    sdk_dir = Path(args.sdk_dir)
    out_path = Path(args.out) if args.out else sim_dir / "sdk_vs_sim_controller_compare.csv"

    rows = collect(sim_dir, sdk_dir)
    if not rows:
        raise SystemExit("no matching sim or SDK CSV files found")

    rows.sort(key=lambda row: (int(row["joint"]), float(row["angle_deg"]), str(row["source"]), str(row["controller"])))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("wrote", out_path)
    print("rows =", len(rows))
    print_console_summary(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
