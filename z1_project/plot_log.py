#!/usr/bin/env python3
"""Plot CSV logs produced by the Z1 torque-control scripts."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np

NDOF = 6


def read_csv_columns(path: str) -> Dict[str, np.ndarray]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty CSV: {path}")

    data: Dict[str, List[float]] = {}
    for key in rows[0].keys():
        if key is None:
            continue
        values = []
        try:
            for row in rows:
                values.append(float(row[key]))
        except (TypeError, ValueError):
            continue
        data[key] = values
    return {key: np.array(values, dtype=float) for key, values in data.items()}


def pick_matrix(data: Dict[str, np.ndarray], prefixes: List[str]) -> Optional[np.ndarray]:
    for prefix in prefixes:
        cols = []
        ok = True
        for i in range(1, NDOF + 1):
            candidates = [f"{prefix}_{i}", f"{prefix}{i}"]
            found = None
            for name in candidates:
                if name in data:
                    found = data[name]
                    break
            if found is None:
                ok = False
                break
            cols.append(found)
        if ok:
            return np.vstack(cols).T
    return None


def save_figure(fig, out_dir: Path, stem: str, suffix: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_dir / f"{stem}_{suffix}.png", dpi=160)


def plot_joint_matrix(ax, t, values, label_prefix: str, linestyle: str = "-") -> None:
    for i in range(NDOF):
        ax.plot(t, values[:, i], linestyle=linestyle, label=f"{label_prefix}{i + 1}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot a Z1 CSV log")
    parser.add_argument("csv_path")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    data = read_csv_columns(args.csv_path)
    if "t" not in data:
        raise ValueError("CSV must contain a t column")

    t = data["t"]
    q_actual = pick_matrix(data, ["q_actual", "q"])
    dq_actual = pick_matrix(data, ["dq_actual", "dq"])
    q_des = pick_matrix(data, ["q_des"])
    dq_des = pick_matrix(data, ["dq_des"])
    ddq_des = pick_matrix(data, ["ddq_des"])
    tau = pick_matrix(data, ["tau"])
    tau_ct = pick_matrix(data, ["tau_ct"])
    tau_ff = pick_matrix(data, ["tau_ff"])

    csv_path = Path(args.csv_path)
    out_dir = Path("logs") / "plots"
    stem = csv_path.stem

    if q_des is not None or q_actual is not None:
        fig, ax = plt.subplots(figsize=(10, 6))
        if q_des is not None:
            plot_joint_matrix(ax, t, q_des, "q_des", "--")
        if q_actual is not None:
            plot_joint_matrix(ax, t, q_actual, "q", "-")
        ax.set_title("Joint Position")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("q [rad]")
        ax.grid(True)
        ax.legend(ncol=2, fontsize="small")
        save_figure(fig, out_dir, stem, "q")

    if dq_des is not None or dq_actual is not None:
        fig, ax = plt.subplots(figsize=(10, 6))
        if dq_des is not None:
            plot_joint_matrix(ax, t, dq_des, "dq_des", "--")
        if dq_actual is not None:
            plot_joint_matrix(ax, t, dq_actual, "dq", "-")
        ax.set_title("Joint Velocity")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("dq [rad/s]")
        ax.grid(True)
        ax.legend(ncol=2, fontsize="small")
        save_figure(fig, out_dir, stem, "dq")

    if ddq_des is not None:
        fig, ax = plt.subplots(figsize=(10, 6))
        plot_joint_matrix(ax, t, ddq_des, "ddq_des", "-")
        ax.set_title("Desired Joint Acceleration")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("ddq_des [rad/s^2]")
        ax.grid(True)
        ax.legend(ncol=2, fontsize="small")
        save_figure(fig, out_dir, stem, "ddq_des")

    if tau is not None or tau_ct is not None or tau_ff is not None:
        fig, ax = plt.subplots(figsize=(10, 6))
        if tau is not None:
            plot_joint_matrix(ax, t, tau, "tau", "-")
        if tau_ct is not None:
            plot_joint_matrix(ax, t, tau_ct, "tau_ct", "-")
        if tau_ff is not None:
            plot_joint_matrix(ax, t, tau_ff, "tau_ff", "--")
        ax.set_title("Torque")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("tau [Nm]")
        ax.grid(True)
        ax.legend(ncol=2, fontsize="small")
        save_figure(fig, out_dir, stem, "tau")

    if q_des is not None and q_actual is not None:
        fig, ax = plt.subplots(figsize=(10, 6))
        plot_joint_matrix(ax, t, q_des - q_actual, "e", "-")
        ax.set_title("Position Error")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("q_des - q_actual [rad]")
        ax.grid(True)
        ax.legend(ncol=2, fontsize="small")
        save_figure(fig, out_dir, stem, "position_error")

    print(f"plots written to {out_dir}")
    if args.show:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
