#!/usr/bin/env python3
"""Compare forward-pose pure-torque logs against SDK lowcmd logs."""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

import numpy as np

from trajectory import NDOF

PURE_RE = re.compile(
    r"forward_(?P<tag>\d+pct|[\dp.]+)_(?P<label>augpd_nofric|augpd_fric|computed_nofric|computed_fric)\.csv$"
)
SDK_RE = re.compile(r"forward_(?P<tag>\d+pct|[\dp.]+)_sdk_lowcmd\.csv$")


def scale_from_tag(tag: str) -> float:
    if tag.endswith("pct"):
        return float(tag[:-3]) / 100.0
    return float(tag.replace("p", "."))


def matrix(rows: list[dict[str, str]], prefix: str) -> np.ndarray:
    return np.array([[float(row[f"{prefix}_{i}"]) for i in range(1, NDOF + 1)] for row in rows], dtype=float)


def first_existing_prefix(fieldnames: list[str], candidates: list[str]) -> str | None:
    names = set(fieldnames)
    for prefix in candidates:
        if all(f"{prefix}_{i}" in names for i in range(1, NDOF + 1)):
            return prefix
    return None


def safe_matrix(rows: list[dict[str, str]], prefix: str | None) -> np.ndarray | None:
    if prefix is None:
        return None
    try:
        return matrix(rows, prefix)
    except (KeyError, ValueError):
        return None


def analyze_csv(path: Path, *, source: str, label: str, tag: str, scale: float) -> dict:
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

    q_des = matrix(outbound, q_des_prefix)
    q_actual = matrix(outbound, "q_actual")
    dq_actual = matrix(outbound, "dq_actual")
    error = q_des - q_actual
    desired_delta = q_des[-1] - q_des[0]
    actual_delta = q_actual[-1] - q_actual[0]
    achieved_pct = np.full(NDOF, np.nan)
    moving = np.abs(desired_delta) > 1e-12
    achieved_pct[moving] = 100.0 * actual_delta[moving] / desired_delta[moving]

    tau_cmd = safe_matrix(outbound, tau_cmd_prefix)
    tau_state = safe_matrix(outbound, tau_state_prefix)
    max_abs_tau_cmd = np.max(np.abs(tau_cmd), axis=0) if tau_cmd is not None else np.full(NDOF, np.nan)
    max_abs_tau_state = np.max(np.abs(tau_state), axis=0) if tau_state is not None else np.full(NDOF, np.nan)

    t0 = float(outbound[0]["t"])
    t1 = float(outbound[-1]["t"])
    rms_error = np.sqrt(np.mean(error * error, axis=0))
    max_abs_error = np.max(np.abs(error), axis=0)
    max_abs_dq_actual = np.max(np.abs(dq_actual), axis=0)

    result = {
        "source": source,
        "label": label,
        "file": str(path),
        "scale_tag": tag,
        "scale": scale,
        "outbound_samples": len(outbound),
        "outbound_duration_s": t1 - t0,
        "outbound_rate_hz": len(outbound) / max(t1 - t0, 1e-9),
        "q_des_prefix": q_des_prefix,
        "tau_cmd_prefix": tau_cmd_prefix or "",
        "tau_state_prefix": tau_state_prefix or "",
        "desired_delta_norm": float(np.linalg.norm(desired_delta)),
        "actual_delta_norm": float(np.linalg.norm(actual_delta)),
        "final_error_norm": float(np.linalg.norm(error[-1])),
        "max_error_norm": float(np.max(np.linalg.norm(error, axis=1))),
        "max_abs_error_norm": float(np.linalg.norm(max_abs_error)),
        "rms_error_norm": float(np.linalg.norm(rms_error)),
        "max_abs_tau_cmd_or_tau_norm": float(np.linalg.norm(max_abs_tau_cmd))
        if tau_cmd is not None
        else float("nan"),
        "max_abs_tau_state_norm": float(np.linalg.norm(max_abs_tau_state))
        if tau_state is not None
        else float("nan"),
        "max_abs_dq_actual_norm": float(np.linalg.norm(max_abs_dq_actual)),
    }

    for i in range(NDOF):
        idx = i + 1
        result[f"final_error_{idx}"] = error[-1, i]
        result[f"final_error_deg_{idx}"] = math.degrees(error[-1, i])
        result[f"max_abs_error_{idx}"] = max_abs_error[i]
        result[f"max_abs_error_deg_{idx}"] = math.degrees(max_abs_error[i])
        result[f"rms_error_{idx}"] = rms_error[i]
        result[f"rms_error_deg_{idx}"] = math.degrees(rms_error[i])
        result[f"desired_delta_{idx}"] = desired_delta[i]
        result[f"actual_delta_{idx}"] = actual_delta[i]
        result[f"achieved_pct_{idx}"] = achieved_pct[i]
        result[f"max_abs_tau_cmd_or_tau_{idx}"] = max_abs_tau_cmd[i]
        result[f"max_abs_tau_state_{idx}"] = max_abs_tau_state[i]
        result[f"max_abs_dq_actual_{idx}"] = max_abs_dq_actual[i]

    return result


def collect(pure_dir: Path, sdk_dir: Path) -> list[dict]:
    rows: list[dict] = []
    if pure_dir.exists():
        for path in sorted(pure_dir.glob("*.csv")):
            match = PURE_RE.match(path.name)
            if not match:
                continue
            tag = match.group("tag")
            rows.append(
                analyze_csv(
                    path,
                    source="pure_torque",
                    label=match.group("label"),
                    tag=tag,
                    scale=scale_from_tag(tag),
                )
            )

    if sdk_dir.exists():
        for path in sorted(sdk_dir.glob("*.csv")):
            if path.name.startswith("summary"):
                continue
            match = SDK_RE.match(path.name)
            if not match:
                continue
            tag = match.group("tag")
            rows.append(
                analyze_csv(
                    path,
                    source="sdk",
                    label="sdk_lowcmd",
                    tag=tag,
                    scale=scale_from_tag(tag),
                )
            )
    return rows


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare forward-pose pure-torque logs with SDK logs")
    parser.add_argument("--pure-dir", default="logs/eval_forward_pose_4controllers")
    parser.add_argument("--sdk-dir", default="logs/eval_forward_pose_sdk")
    parser.add_argument("--out", default="logs/forward_pose_sdk_compare.csv")
    return parser


def print_console_summary(rows: list[dict]) -> None:
    index: dict[str, list[dict]] = {}
    for row in rows:
        index.setdefault(str(row["scale_tag"]), []).append(row)

    print()
    print("scale  sdk max/final/tau_state        best pure(label:max/final/tau_cmd)")
    for tag in sorted(index, key=scale_from_tag):
        group = index[tag]
        sdk = [row for row in group if row["source"] == "sdk"]
        pure = [row for row in group if row["source"] == "pure_torque"]
        if not sdk or not pure:
            missing = "sdk" if not sdk else "pure"
            print(f"{tag:>6}  missing {missing} rows")
            continue

        sdk_row = min(sdk, key=lambda row: float(row["max_error_norm"]))
        best = min(pure, key=lambda row: float(row["max_error_norm"]))
        print(
            f"{tag:>6}  "
            f"{float(sdk_row['max_error_norm']):.4f}/"
            f"{float(sdk_row['final_error_norm']):.4f}/"
            f"{float(sdk_row['max_abs_tau_state_norm']):.4f}        "
            f"{best['label']}:"
            f"{float(best['max_error_norm']):.4f}/"
            f"{float(best['final_error_norm']):.4f}/"
            f"{float(best['max_abs_tau_cmd_or_tau_norm']):.4f}"
        )


def main() -> int:
    args = build_arg_parser().parse_args()
    rows = collect(Path(args.pure_dir), Path(args.sdk_dir))
    if not rows:
        raise SystemExit("no matching CSV files found")

    rows.sort(key=lambda row: (float(row["scale"]), str(row["source"]), str(row["label"])))
    out_path = Path(args.out)
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
