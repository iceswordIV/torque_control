#!/usr/bin/env python3
"""Compare recorded SDK feedforward torque with the Python analytic model."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from z1_analytic_dynamics import NDOF, dynamics_analytic


def read_trace(path: str):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError(f"empty CSV: {path}")

    required = ["t"] + [f"q_cmd_{i}" for i in range(1, NDOF + 1)] + [f"dq_cmd_{i}" for i in range(1, NDOF + 1)] + [
        f"tau_{i}" for i in range(1, NDOF + 1)
    ]
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise ValueError(f"{path} is not a clean SDK source trace; missing columns: {missing}")

    t = np.array([float(r["t"]) for r in rows], dtype=float)
    q = np.array([[float(r[f"q_cmd_{i}"]) for i in range(1, NDOF + 1)] for r in rows], dtype=float)
    dq = np.array([[float(r[f"dq_cmd_{i}"]) for i in range(1, NDOF + 1)] for r in rows], dtype=float)
    tau_sdk = np.array([[float(r[f"tau_{i}"]) for i in range(1, NDOF + 1)] for r in rows], dtype=float)
    if all(f"ddq_cmd_{i}" in rows[0] for i in range(1, NDOF + 1)):
        ddq = np.array([[float(r[f"ddq_cmd_{i}"]) for i in range(1, NDOF + 1)] for r in rows], dtype=float)
    else:
        ddq = np.zeros_like(q)
    return t, q, dq, ddq, tau_sdk


def write_output(path: str, t, q, dq, ddq, tau_sdk, tau_py, diff):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["t"]
            + [f"q_cmd_{i + 1}" for i in range(NDOF)]
            + [f"dq_cmd_{i + 1}" for i in range(NDOF)]
            + [f"ddq_cmd_{i + 1}" for i in range(NDOF)]
            + [f"tau_sdk_{i + 1}" for i in range(NDOF)]
            + [f"tau_python_{i + 1}" for i in range(NDOF)]
            + [f"tau_diff_{i + 1}" for i in range(NDOF)]
        )
        for i in range(len(t)):
            writer.writerow([t[i], *q[i], *dq[i], *ddq[i], *tau_sdk[i], *tau_py[i], *diff[i]])


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare SDK recorded tau_ff against Python analytic tau_ff")
    parser.add_argument("input_csv")
    parser.add_argument("--csv-log", default="logs/feedforward_compare.csv")
    parser.add_argument("--stride", type=int, default=1, help="evaluate every Nth input row")
    args = parser.parse_args()

    if args.stride <= 0:
        raise ValueError("--stride must be positive")

    t, q, dq, ddq, tau_sdk = read_trace(args.input_csv)
    idx = np.arange(0, len(t), args.stride)
    t = t[idx]
    q = q[idx]
    dq = dq[idx]
    ddq = ddq[idx]
    tau_sdk = tau_sdk[idx]

    tau_py = np.zeros_like(tau_sdk)
    for i in range(len(t)):
        M, C, N, _ = dynamics_analytic(q[i], dq[i])
        tau_py[i] = M @ ddq[i] + C @ dq[i] + N

    diff = tau_py - tau_sdk
    write_output(args.csv_log, t, q, dq, ddq, tau_sdk, tau_py, diff)

    max_abs_sdk = np.max(np.abs(tau_sdk), axis=0)
    max_abs_py = np.max(np.abs(tau_py), axis=0)
    max_abs_diff = np.max(np.abs(diff), axis=0)
    rms_diff = np.sqrt(np.mean(diff * diff, axis=0))
    denom = np.maximum(max_abs_sdk, 1e-9)
    rel_max = max_abs_diff / denom

    print("input =", args.input_csv)
    print("samples =", len(t), "stride =", args.stride)
    print("max |tau_sdk| =", np.array2string(max_abs_sdk, precision=6, suppress_small=False))
    print("max |tau_python| =", np.array2string(max_abs_py, precision=6, suppress_small=False))
    print("max |python - sdk| =", np.array2string(max_abs_diff, precision=6, suppress_small=False))
    print("rms |python - sdk| =", np.array2string(rms_diff, precision=6, suppress_small=False))
    print("relative max diff =", np.array2string(rel_max, precision=6, suppress_small=False))
    print("CSV path =", args.csv_log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
