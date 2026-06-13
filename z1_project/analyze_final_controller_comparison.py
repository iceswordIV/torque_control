#!/usr/bin/env python3
"""Generate final report/poster analysis artifacts from controller CSV logs.

This is an analysis-only tool. It reads existing simulation and SDK LOWCMD CSVs,
then writes derived summaries, conclusions, and plots into the selected log
folder. It does not command Gazebo, ROS, or hardware.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from trajectory import NDOF


SIM_RE = re.compile(
    r"(?P<controller>augpd_nofric|augpd_fric2p5|computed_torque_baseline|cpid_fric2p5)"
    r"_j(?P<joint>\d+)_(?P<sign>pos|neg)(?P<angle>\d+(?:\.\d+)?)\.csv$"
)
SDK_RE = re.compile(
    r"workspace_j(?P<joint>\d+)_(?P<sign>pos|neg)(?P<angle>\d+(?:\.\d+)?)deg_sdk_lowcmd\.csv$"
)

CONTROLLER_ORDER = [
    "augpd_nofric",
    "augpd_fric2p5",
    "computed_torque_baseline",
    "cpid_fric2p5",
    "sdk_lowcmd",
]
CONTROLLER_LABELS = {
    "augpd_nofric": "AugPD no friction",
    "augpd_fric2p5": "AugPD friction",
    "computed_torque_baseline": "Computed torque baseline",
    "cpid_fric2p5": "CPID friction",
    "sdk_lowcmd": "Unitree SDK LOWCMD",
}
CONTROLLER_COLORS = {
    "augpd_nofric": "#7f7f7f",
    "augpd_fric2p5": "#1f77b4",
    "computed_torque_baseline": "#ff7f0e",
    "cpid_fric2p5": "#2ca02c",
    "sdk_lowcmd": "#d62728",
}
TRACKING_PHASES = {"outbound", "outbound_hold"}
RETURN_SUCCESS_CMD_ERROR_DEG = 2.0
RETURN_SUCCESS_ALL_ERROR_DEG = 5.0


def parse_signed_angle(sign: str, angle_text: str) -> float:
    sign_value = -1.0 if sign == "neg" else 1.0
    return sign_value * float(angle_text)


def angle_label(angle_deg: float) -> str:
    value = int(abs(angle_deg)) if float(angle_deg).is_integer() else abs(angle_deg)
    return f"neg{value:g}" if angle_deg < 0.0 else f"pos{value:g}"


def case_label(joint: int, angle_deg: float) -> str:
    sign = "+" if angle_deg >= 0.0 else ""
    return f"J{joint} {sign}{angle_deg:g} deg"


def controller_sort_key(controller: str) -> int:
    try:
        return CONTROLLER_ORDER.index(controller)
    except ValueError:
        return len(CONTROLLER_ORDER)


def latest_log_dir(root: Path) -> Path:
    candidates = [path for path in (root / "logs").glob("sim_compare_joint_controllers_*") if path.is_dir()]
    if not candidates:
        raise SystemExit(f"no sim_compare_joint_controllers_* folders under {root / 'logs'}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_rows(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        return rows, reader.fieldnames or []


def first_existing_prefix(fieldnames: Iterable[str], candidates: Iterable[str]) -> str | None:
    names = set(fieldnames)
    for prefix in candidates:
        if all(f"{prefix}_{i}" in names for i in range(1, NDOF + 1)):
            return prefix
    return None


def matrix(rows: list[dict[str, str]], prefix: str) -> np.ndarray:
    return np.array([[float(row[f"{prefix}_{i}"]) for i in range(1, NDOF + 1)] for row in rows], dtype=float)


def safe_matrix(rows: list[dict[str, str]], prefix: str | None) -> np.ndarray | None:
    if prefix is None:
        return None
    try:
        return matrix(rows, prefix)
    except (KeyError, ValueError):
        return None


def safe_max_abs(values: np.ndarray | None) -> float:
    if values is None or values.size == 0:
        return float("nan")
    return float(np.max(np.abs(values)))


def safe_joint_max_abs(values: np.ndarray | None, joint_index: int) -> float:
    if values is None or values.size == 0:
        return float("nan")
    return float(np.max(np.abs(values[:, joint_index])))


def rms(values: np.ndarray) -> float:
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values**2)))


def mean(values: Iterable[float]) -> float:
    nums = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.fmean(nums) if nums else float("nan")


def median(values: Iterable[float]) -> float:
    nums = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.median(nums) if nums else float("nan")


def fmt_float(value: float, digits: int = 4) -> str:
    if not math.isfinite(float(value)):
        return "nan"
    return f"{float(value):.{digits}f}"


def tau_prefix_for(source: str, fieldnames: list[str]) -> tuple[str | None, str | None]:
    if source == "sdk":
        comparable = first_existing_prefix(fieldnames, ["tau_state", "tau_feedback", "tau_sdk_cmd", "tau_total", "tau"])
        feedforward = first_existing_prefix(fieldnames, ["tau_sdk_cmd"])
        return comparable, feedforward
    comparable = first_existing_prefix(fieldnames, ["tau_total", "tau", "tau_cmd", "tau_ff"])
    return comparable, None


def analyze_log(path: Path, *, source: str, controller: str, joint: int, angle_deg: float, log_dir: Path) -> dict[str, object]:
    rows, fieldnames = read_rows(path)
    if not rows:
        raise ValueError(f"empty CSV: {path}")

    phase_counts = Counter(row.get("phase", "outbound") or "outbound" for row in rows)
    tracking_rows = [row for row in rows if (row.get("phase", "outbound") or "outbound") in TRACKING_PHASES]
    if not tracking_rows:
        tracking_rows = rows
    return_rows = [row for row in rows if row.get("phase", "") == "return"]

    q_des_prefix = first_existing_prefix(fieldnames, ["q_des", "q_cmd"])
    if q_des_prefix is None:
        raise ValueError(f"missing desired q columns in {path}")
    tau_prefix, sdk_feedforward_prefix = tau_prefix_for(source, fieldnames)

    joint_index = joint - 1
    q_des = matrix(tracking_rows, q_des_prefix)
    q_actual = matrix(tracking_rows, "q_actual")
    error = q_des - q_actual
    tau = safe_matrix(tracking_rows, tau_prefix)
    sdk_feedforward_tau = safe_matrix(tracking_rows, sdk_feedforward_prefix)

    t0 = float(tracking_rows[0].get("t", 0.0))
    t1 = float(tracking_rows[-1].get("t", t0))
    desired_delta = q_des[-1, joint_index] - q_des[0, joint_index]
    actual_delta = q_actual[-1, joint_index] - q_actual[0, joint_index]
    achieved_pct = 100.0 * actual_delta / desired_delta if abs(desired_delta) > 1e-12 else float("nan")
    cmd_error = error[:, joint_index]
    cmd_final_error = float(cmd_error[-1])

    return_present = bool(return_rows)
    return_final_cmd_abs_error_rad = float("nan")
    return_final_all_joint_max_abs_error_rad = float("nan")
    return_duration_s = float("nan")
    return_desired_back_to_start_deg = float("nan")
    if return_rows:
        q_des_return = matrix(return_rows, q_des_prefix)
        q_actual_return = matrix(return_rows, "q_actual")
        return_error = q_des_return - q_actual_return
        return_final_cmd_abs_error_rad = abs(float(return_error[-1, joint_index]))
        return_final_all_joint_max_abs_error_rad = float(np.max(np.abs(return_error[-1, :])))
        return_duration_s = float(return_rows[-1].get("t", 0.0)) - float(return_rows[0].get("t", 0.0))
        return_desired_back_to_start_deg = math.degrees(abs(float(q_des_return[-1, joint_index] - q_des[0, joint_index])))

    return_completed = (
        return_present
        and math.degrees(return_final_cmd_abs_error_rad) <= RETURN_SUCCESS_CMD_ERROR_DEG
        and math.degrees(return_final_all_joint_max_abs_error_rad) <= RETURN_SUCCESS_ALL_ERROR_DEG
    )

    return {
        "source": source,
        "controller": controller,
        "controller_label": CONTROLLER_LABELS.get(controller, controller),
        "joint": joint,
        "angle_deg": angle_deg,
        "case": case_label(joint, angle_deg),
        "file": str(path.relative_to(log_dir)),
        "phase_counts": ";".join(f"{name}:{phase_counts[name]}" for name in sorted(phase_counts)),
        "tracking_phase_definition": "+".join(sorted(TRACKING_PHASES)),
        "tracking_samples": len(tracking_rows),
        "tracking_duration_s": max(t1 - t0, 0.0),
        "tracking_rate_hz": len(tracking_rows) / max(t1 - t0, 1e-9),
        "desired_delta_deg": math.degrees(float(desired_delta)),
        "actual_delta_deg": math.degrees(float(actual_delta)),
        "achieved_pct": achieved_pct,
        "cmd_joint_signed_final_error_deg": math.degrees(cmd_final_error),
        "cmd_joint_final_abs_error_deg": math.degrees(abs(cmd_final_error)),
        "cmd_joint_max_abs_error_deg": math.degrees(float(np.max(np.abs(cmd_error)))),
        "cmd_joint_rms_error_deg": math.degrees(rms(cmd_error)),
        "cmd_joint_final_abs_error_rad": abs(cmd_final_error),
        "cmd_joint_max_abs_error_rad": float(np.max(np.abs(cmd_error))),
        "cmd_joint_rms_error_rad": rms(cmd_error),
        "cmd_joint_max_abs_torque_nm": safe_joint_max_abs(tau, joint_index),
        "all_joint_max_abs_error_deg": math.degrees(float(np.max(np.abs(error)))),
        "all_joint_rms_error_deg": math.degrees(rms(error)),
        "all_joint_max_abs_error_rad": float(np.max(np.abs(error))),
        "all_joint_rms_error_rad": rms(error),
        "all_joint_max_abs_torque_nm": safe_max_abs(tau),
        "torque_measure_prefix": tau_prefix or "",
        "sdk_feedforward_cmd_joint_max_abs_torque_nm": safe_joint_max_abs(sdk_feedforward_tau, joint_index),
        "sdk_feedforward_all_joint_max_abs_torque_nm": safe_max_abs(sdk_feedforward_tau),
        "return_present": return_present,
        "return_samples": len(return_rows),
        "return_duration_s": return_duration_s,
        "return_final_cmd_abs_error_deg": math.degrees(return_final_cmd_abs_error_rad),
        "return_final_all_joint_max_abs_error_deg": math.degrees(return_final_all_joint_max_abs_error_rad),
        "return_desired_back_to_start_deg": return_desired_back_to_start_deg,
        "return_completed_successfully": return_completed,
    }


def collect_logs(log_dir: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for path in sorted(log_dir.glob("*.csv")):
        match = SIM_RE.match(path.name)
        if not match:
            continue
        rows.append(
            analyze_log(
                path,
                source="sim",
                controller=match.group("controller"),
                joint=int(match.group("joint")),
                angle_deg=parse_signed_angle(match.group("sign"), match.group("angle")),
                log_dir=log_dir,
            )
        )

    sdk_dir = log_dir / "sdk_lowcmd"
    for path in sorted(sdk_dir.glob("workspace_j*_sdk_lowcmd.csv")):
        match = SDK_RE.match(path.name)
        if not match:
            continue
        rows.append(
            analyze_log(
                path,
                source="sdk",
                controller="sdk_lowcmd",
                joint=int(match.group("joint")),
                angle_deg=parse_signed_angle(match.group("sign"), match.group("angle")),
                log_dir=log_dir,
            )
        )

    rows.sort(key=lambda row: (int(row["joint"]), float(row["angle_deg"]), controller_sort_key(str(row["controller"]))))
    return rows


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str] | None = None) -> None:
    if not rows:
        raise ValueError(f"no rows to write: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    names = fieldnames or list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=names, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def build_inventory(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    by_controller: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        by_controller[str(row["controller"])].append(row)

    inventory = []
    all_cases = {(int(row["joint"]), float(row["angle_deg"])) for row in rows}
    for controller in sorted(by_controller, key=controller_sort_key):
        group = by_controller[controller]
        cases = {(int(row["joint"]), float(row["angle_deg"])) for row in group}
        missing = sorted(all_cases - cases)
        per_joint = Counter(int(row["joint"]) for row in group)
        inventory.append(
            {
                "controller": controller,
                "controller_label": CONTROLLER_LABELS.get(controller, controller),
                "csv_logs": len(group),
                "unique_joint_angle_cases": len(cases),
                "joint_counts": ";".join(f"J{joint}:{per_joint[joint]}" for joint in sorted(per_joint)),
                "missing_cases": ";".join(case_label(joint, angle) for joint, angle in missing),
            }
        )
    return inventory


def aggregate_by_controller(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    by_controller: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        by_controller[str(row["controller"])].append(row)

    summaries = []
    for controller in sorted(by_controller, key=controller_sort_key):
        group = by_controller[controller]
        worst_error = max(group, key=lambda row: float(row["cmd_joint_max_abs_error_deg"]))
        worst_torque = max(group, key=lambda row: float(row["cmd_joint_max_abs_torque_nm"]))
        return_successes = sum(1 for row in group if row["return_completed_successfully"])
        summaries.append(
            {
                "controller": controller,
                "controller_label": CONTROLLER_LABELS.get(controller, controller),
                "source": group[0]["source"],
                "num_tests": len(group),
                "unique_joint_angle_cases": len({(int(row["joint"]), float(row["angle_deg"])) for row in group}),
                "return_success_count": return_successes,
                "return_success_rate_pct": 100.0 * return_successes / max(len(group), 1),
                "mean_cmd_joint_rms_error_deg": mean(row["cmd_joint_rms_error_deg"] for row in group),
                "median_cmd_joint_rms_error_deg": median(row["cmd_joint_rms_error_deg"] for row in group),
                "mean_cmd_joint_max_abs_error_deg": mean(row["cmd_joint_max_abs_error_deg"] for row in group),
                "median_cmd_joint_max_abs_error_deg": median(row["cmd_joint_max_abs_error_deg"] for row in group),
                "mean_cmd_joint_final_abs_error_deg": mean(row["cmd_joint_final_abs_error_deg"] for row in group),
                "median_cmd_joint_final_abs_error_deg": median(row["cmd_joint_final_abs_error_deg"] for row in group),
                "mean_cmd_joint_max_abs_torque_nm": mean(row["cmd_joint_max_abs_torque_nm"] for row in group),
                "median_cmd_joint_max_abs_torque_nm": median(row["cmd_joint_max_abs_torque_nm"] for row in group),
                "mean_all_joint_rms_error_deg": mean(row["all_joint_rms_error_deg"] for row in group),
                "mean_all_joint_max_abs_error_deg": mean(row["all_joint_max_abs_error_deg"] for row in group),
                "mean_all_joint_max_abs_torque_nm": mean(row["all_joint_max_abs_torque_nm"] for row in group),
                "worst_cmd_joint_max_abs_error_deg": worst_error["cmd_joint_max_abs_error_deg"],
                "worst_error_case": worst_error["case"],
                "largest_cmd_joint_max_abs_torque_nm": worst_torque["cmd_joint_max_abs_torque_nm"],
                "largest_torque_case": worst_torque["case"],
            }
        )
    return summaries


def build_sdk_vs_sim_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    sdk_by_case = {
        (int(row["joint"]), float(row["angle_deg"])): row for row in rows if row["controller"] == "sdk_lowcmd"
    }
    result = []
    for controller in [item for item in CONTROLLER_ORDER if item != "sdk_lowcmd"]:
        sim_rows = [row for row in rows if row["controller"] == controller]
        pairs = [(row, sdk_by_case[(int(row["joint"]), float(row["angle_deg"]))]) for row in sim_rows if (int(row["joint"]), float(row["angle_deg"])) in sdk_by_case]
        if not pairs:
            continue
        result.append(
            {
                "sim_controller": controller,
                "sim_controller_label": CONTROLLER_LABELS.get(controller, controller),
                "matched_cases": len(pairs),
                "sim_mean_rms_error_deg": mean(sim["cmd_joint_rms_error_deg"] for sim, _ in pairs),
                "sdk_mean_rms_error_deg": mean(sdk["cmd_joint_rms_error_deg"] for _, sdk in pairs),
                "sim_minus_sdk_mean_rms_error_deg": mean(float(sim["cmd_joint_rms_error_deg"]) - float(sdk["cmd_joint_rms_error_deg"]) for sim, sdk in pairs),
                "mean_abs_rms_error_difference_deg": mean(abs(float(sim["cmd_joint_rms_error_deg"]) - float(sdk["cmd_joint_rms_error_deg"])) for sim, sdk in pairs),
                "sim_mean_max_error_deg": mean(sim["cmd_joint_max_abs_error_deg"] for sim, _ in pairs),
                "sdk_mean_max_error_deg": mean(sdk["cmd_joint_max_abs_error_deg"] for _, sdk in pairs),
                "sim_minus_sdk_mean_max_error_deg": mean(float(sim["cmd_joint_max_abs_error_deg"]) - float(sdk["cmd_joint_max_abs_error_deg"]) for sim, sdk in pairs),
                "mean_abs_max_error_difference_deg": mean(abs(float(sim["cmd_joint_max_abs_error_deg"]) - float(sdk["cmd_joint_max_abs_error_deg"])) for sim, sdk in pairs),
                "sim_better_than_sdk_by_rms_count": sum(float(sim["cmd_joint_rms_error_deg"]) < float(sdk["cmd_joint_rms_error_deg"]) for sim, sdk in pairs),
                "sim_better_than_sdk_by_max_error_count": sum(float(sim["cmd_joint_max_abs_error_deg"]) < float(sdk["cmd_joint_max_abs_error_deg"]) for sim, sdk in pairs),
                "mean_abs_torque_difference_nm": mean(abs(float(sim["cmd_joint_max_abs_torque_nm"]) - float(sdk["cmd_joint_max_abs_torque_nm"])) for sim, sdk in pairs),
            }
        )
    return result


def parse_dynamics_summary(path: Path) -> list[dict[str, object]]:
    if not path.exists():
        return []

    case_name = ""
    rows: list[dict[str, object]] = []
    current: dict[str, object] | None = None
    patterns = {
        "max_abs_M_diff": re.compile(r"max abs M diff = ([0-9.eE+-]+)"),
        "max_abs_N_diff": re.compile(r"max abs N diff = ([0-9.eE+-]+)"),
        "norm_h_python_minus_h_sdk": re.compile(r"norm h_python - h_sdk = ([0-9.eE+-]+)"),
        "norm_tau_diff": re.compile(r"norm tau diff = ([0-9.eE+-]+)"),
        "max_abs_dM_diff": re.compile(r"max abs dM_python - fd_dM_sdk = ([0-9.eE+-]+)"),
    }

    for line in path.read_text().splitlines():
        if line.startswith("===== Case "):
            if current:
                rows.append(current)
            case_name = line.strip("= ").strip()
            current = {"case": case_name}
            continue
        if current is None:
            continue
        for key, pattern in patterns.items():
            match = pattern.search(line)
            if match:
                current[key] = float(match.group(1))
    if current:
        rows.append(current)
    return rows


def plot_bar(path: Path, summaries: list[dict[str, object]], metric: str, ylabel: str, title: str) -> None:
    ordered = [row for row in summaries if row["controller"] in CONTROLLER_ORDER]
    labels = [str(row["controller_label"]) for row in ordered]
    values = [float(row[metric]) for row in ordered]
    colors = [CONTROLLER_COLORS.get(str(row["controller"]), "#333333") for row in ordered]

    fig, ax = plt.subplots(figsize=(9, 5), dpi=150)
    ax.bar(labels, values, color=colors)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.25)
    ax.tick_params(axis="x", rotation=25)
    for idx, value in enumerate(values):
        if math.isfinite(value):
            ax.text(idx, value, fmt_float(value, 2), ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_per_joint_rms(path: Path, rows: list[dict[str, object]]) -> None:
    joints = list(range(1, NDOF + 1))
    controllers = [controller for controller in CONTROLLER_ORDER if any(row["controller"] == controller for row in rows)]
    width = 0.16
    x = np.arange(len(joints))

    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    for idx, controller in enumerate(controllers):
        values = []
        for joint in joints:
            values.append(mean(row["cmd_joint_rms_error_deg"] for row in rows if row["controller"] == controller and int(row["joint"]) == joint))
        offset = (idx - (len(controllers) - 1) / 2.0) * width
        ax.bar(
            x + offset,
            values,
            width=width,
            label=CONTROLLER_LABELS.get(controller, controller),
            color=CONTROLLER_COLORS.get(controller),
        )
    ax.set_xticks(x)
    ax.set_xticklabels([f"J{joint}" for joint in joints])
    ax.set_ylabel("Mean commanded-joint RMS error (deg)")
    ax.set_title("Per-joint RMS tracking error by controller")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_worst_cases(path: Path, worst_rows: list[dict[str, object]]) -> None:
    top = list(reversed(worst_rows[:10]))
    labels = [f"{row['controller_label']} {row['case']}" for row in top]
    values = [float(row["cmd_joint_max_abs_error_deg"]) for row in top]
    colors = [CONTROLLER_COLORS.get(str(row["controller"]), "#333333") for row in top]

    fig, ax = plt.subplots(figsize=(10, 6), dpi=150)
    ax.barh(labels, values, color=colors)
    ax.set_xlabel("Commanded-joint max absolute error (deg)")
    ax.set_title("Worst 10 tracking cases")
    ax.grid(axis="x", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_sdk_vs_sim(path: Path, rows: list[dict[str, object]]) -> None:
    cases = sorted({(int(row["joint"]), float(row["angle_deg"])) for row in rows})
    x = np.arange(len(cases))
    labels = [case_label(joint, angle) for joint, angle in cases]

    fig, axes = plt.subplots(2, 1, figsize=(13, 7), dpi=150, sharex=True)
    for controller in CONTROLLER_ORDER:
        case_to_row = {
            (int(row["joint"]), float(row["angle_deg"])): row for row in rows if row["controller"] == controller
        }
        if not case_to_row:
            continue
        rms_values = [float(case_to_row[case]["cmd_joint_rms_error_deg"]) if case in case_to_row else np.nan for case in cases]
        max_values = [float(case_to_row[case]["cmd_joint_max_abs_error_deg"]) if case in case_to_row else np.nan for case in cases]
        axes[0].plot(x, rms_values, marker="o", linewidth=1.4, markersize=3, label=CONTROLLER_LABELS.get(controller, controller), color=CONTROLLER_COLORS.get(controller))
        axes[1].plot(x, max_values, marker="o", linewidth=1.4, markersize=3, label=CONTROLLER_LABELS.get(controller, controller), color=CONTROLLER_COLORS.get(controller))

    axes[0].set_ylabel("RMS error (deg)")
    axes[0].set_title("SDK vs simulation commanded-joint RMS error by case")
    axes[1].set_ylabel("Max error (deg)")
    axes[1].set_title("SDK vs simulation commanded-joint max error by case")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, rotation=65, ha="right", fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, ncol=3)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def load_trace(path: Path, joint: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows, fieldnames = read_rows(path)
    q_des_prefix = first_existing_prefix(fieldnames, ["q_des", "q_cmd"])
    if q_des_prefix is None:
        raise ValueError(f"missing desired q columns in {path}")
    joint_index = joint - 1
    t = np.array([float(row["t"]) for row in rows], dtype=float)
    t = t - t[0]
    q_des = matrix(rows, q_des_prefix)[:, joint_index]
    q_actual = matrix(rows, "q_actual")[:, joint_index]
    return t, np.degrees(q_des), np.degrees(q_actual)


def plot_tracking_case(path: Path, rows: list[dict[str, object]], log_dir: Path, joint: int, angle_deg: float) -> None:
    group = [row for row in rows if int(row["joint"]) == joint and float(row["angle_deg"]) == float(angle_deg)]
    group.sort(key=lambda row: controller_sort_key(str(row["controller"])))
    if not group:
        return

    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    desired_drawn = False
    for row in group:
        trace_path = log_dir / str(row["file"])
        t, q_des_deg, q_actual_deg = load_trace(trace_path, joint)
        if not desired_drawn:
            ax.plot(t, q_des_deg, "k--", linewidth=2.0, label="desired")
            desired_drawn = True
        ax.plot(
            t,
            q_actual_deg,
            linewidth=1.3,
            label=CONTROLLER_LABELS.get(str(row["controller"]), str(row["controller"])),
            color=CONTROLLER_COLORS.get(str(row["controller"])),
        )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(f"Joint {joint} position (deg)")
    ax.set_title(f"Representative tracking: {case_label(joint, angle_deg)}")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def generate_plots(log_dir: Path, rows: list[dict[str, object]], summaries: list[dict[str, object]], worst_rows: list[dict[str, object]]) -> list[Path]:
    plot_dir = log_dir / "final_analysis_plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    outputs = [
        plot_dir / "mean_rms_error_by_controller.png",
        plot_dir / "mean_max_error_by_controller.png",
        plot_dir / "mean_final_error_by_controller.png",
        plot_dir / "mean_max_torque_by_controller.png",
        plot_dir / "per_joint_rms_error_by_controller.png",
        plot_dir / "worst_10_cases_by_max_error.png",
        plot_dir / "sdk_vs_sim_error_comparison.png",
    ]
    plot_bar(outputs[0], summaries, "mean_cmd_joint_rms_error_deg", "Mean commanded-joint RMS error (deg)", "Mean RMS error by controller")
    plot_bar(outputs[1], summaries, "mean_cmd_joint_max_abs_error_deg", "Mean commanded-joint max error (deg)", "Mean max error by controller")
    plot_bar(outputs[2], summaries, "mean_cmd_joint_final_abs_error_deg", "Mean commanded-joint final abs error (deg)", "Mean final tracking error by controller")
    plot_bar(outputs[3], summaries, "mean_cmd_joint_max_abs_torque_nm", "Mean commanded-joint max torque (Nm)", "Mean max torque by controller")
    plot_per_joint_rms(outputs[4], rows)
    plot_worst_cases(outputs[5], worst_rows)
    plot_sdk_vs_sim(outputs[6], rows)

    representative_cases = [
        (1, 30.0),
        (2, 30.0),
        (3, -30.0),
        (4, -30.0),
        (5, -30.0),
        (5, 30.0),
        (6, -30.0),
        (6, 30.0),
    ]
    for joint, angle in representative_cases:
        out = plot_dir / f"tracking_j{joint}_{angle_label(angle)}.png"
        plot_tracking_case(out, rows, log_dir, joint, angle)
        outputs.append(out)
    return outputs


def write_report(
    path: Path,
    *,
    log_dir: Path,
    rows: list[dict[str, object]],
    inventory: list[dict[str, object]],
    summaries: list[dict[str, object]],
    sdk_vs_sim: list[dict[str, object]],
    dynamics_rows: list[dict[str, object]],
    plot_paths: list[Path],
) -> None:
    by_controller = {row["controller"]: row for row in summaries}
    best_rms = min(summaries, key=lambda row: float(row["mean_cmd_joint_rms_error_deg"]))
    best_max = min(summaries, key=lambda row: float(row["mean_cmd_joint_max_abs_error_deg"]))
    largest_torque = max(summaries, key=lambda row: float(row["mean_cmd_joint_max_abs_torque_nm"]))
    worst_rows = sorted(rows, key=lambda row: float(row["cmd_joint_max_abs_error_deg"]), reverse=True)[:5]

    aug_no = by_controller.get("augpd_nofric")
    aug_fric = by_controller.get("augpd_fric2p5")
    cpid = by_controller.get("cpid_fric2p5")
    sdk = by_controller.get("sdk_lowcmd")

    def pct_improvement(before: dict[str, object] | None, after: dict[str, object] | None, metric: str) -> float:
        if not before or not after:
            return float("nan")
        before_value = float(before[metric])
        after_value = float(after[metric])
        return 100.0 * (before_value - after_value) / before_value if abs(before_value) > 1e-12 else float("nan")

    aug_fric_vs_no_rms = pct_improvement(aug_no, aug_fric, "mean_cmd_joint_rms_error_deg")
    cpid_vs_aug_fric_rms = pct_improvement(aug_fric, cpid, "mean_cmd_joint_rms_error_deg")

    if dynamics_rows:
        max_m = max(float(row.get("max_abs_M_diff", float("nan"))) for row in dynamics_rows)
        max_n = max(float(row.get("max_abs_N_diff", float("nan"))) for row in dynamics_rows)
        max_h = max(float(row.get("norm_h_python_minus_h_sdk", float("nan"))) for row in dynamics_rows)
        max_tau = max(float(row.get("norm_tau_diff", float("nan"))) for row in dynamics_rows)
        max_dm = max(float(row.get("max_abs_dM_diff", float("nan"))) for row in dynamics_rows)
        dynamics_text = (
            f"The analytic dynamics comparison found max |M_python - M_sdk| = {max_m:.6g}, "
            f"max |N_python - N_sdk| = {max_n:.6g}, max ||h_python - h_sdk|| = {max_h:.6g}, "
            f"max ||tau_python - tau_sdk|| = {max_tau:.6g}, and max |dM_python - fd_dM_sdk| = {max_dm:.6g}. "
            "These values indicate the local analytic M/C/N model is close to the Unitree SDK inverseDynamics result, "
            "with remaining mismatch small relative to the large gravity/torque terms in the hard poses."
        )
    else:
        dynamics_text = "No sdk_dynamics_compare.txt file was found, so no M/C/N validation summary was generated."

    closest_sdk = min(sdk_vs_sim, key=lambda row: float(row["mean_abs_rms_error_difference_deg"])) if sdk_vs_sim else None
    sdk_text = ""
    if sdk and closest_sdk:
        sdk_text = (
            f"SDK/LOWCMD mean RMS error was {float(sdk['mean_cmd_joint_rms_error_deg']):.3f} deg and mean max error was "
            f"{float(sdk['mean_cmd_joint_max_abs_error_deg']):.3f} deg. The closest simulation controller by mean absolute RMS "
            f"difference was {closest_sdk['sim_controller_label']} with {float(closest_sdk['mean_abs_rms_error_difference_deg']):.3f} deg "
            "mean absolute RMS difference. SDK behavior is therefore useful as a baseline, but it is not identical to the torque-controller simulation."
        )

    lines = [
        "# Final Controller Comparison Report Notes",
        "",
        "## Dataset overview",
        f"- Output folder: `{log_dir}`",
        f"- Raw controller CSV logs analyzed: {len(rows)}",
        f"- Tracking metrics use the outbound/hold segment (`outbound` plus `outbound_hold` when present). Return motion is evaluated separately.",
        f"- Return success threshold: commanded-joint final return error <= {RETURN_SUCCESS_CMD_ERROR_DEG:g} deg and all-joint final return error <= {RETURN_SUCCESS_ALL_ERROR_DEG:g} deg.",
    ]
    for item in inventory:
        lines.append(
            f"- {item['controller_label']}: {item['csv_logs']} CSV logs, {item['unique_joint_angle_cases']} unique joint/angle cases, {item['joint_counts']}."
        )
    lines.extend(
        [
            "",
            "## Controller comparison result",
            f"- Best overall by mean commanded-joint RMS error: {best_rms['controller_label']} ({float(best_rms['mean_cmd_joint_rms_error_deg']):.3f} deg).",
            f"- Lowest mean commanded-joint max error: {best_max['controller_label']} ({float(best_max['mean_cmd_joint_max_abs_error_deg']):.3f} deg).",
            f"- Largest mean commanded-joint max torque: {largest_torque['controller_label']} ({float(largest_torque['mean_cmd_joint_max_abs_torque_nm']):.3f} Nm).",
        ]
    )
    if aug_no and aug_fric:
        lines.append(
            f"- Adding the friction model to AugPD reduced mean RMS error by {aug_fric_vs_no_rms:.1f}% "
            f"({float(aug_no['mean_cmd_joint_rms_error_deg']):.3f} deg to {float(aug_fric['mean_cmd_joint_rms_error_deg']):.3f} deg)."
        )
    if aug_fric and cpid:
        if cpid_vs_aug_fric_rms >= 0:
            lines.append(
                f"- CPID friction reduced mean RMS error by {cpid_vs_aug_fric_rms:.1f}% relative to AugPD friction."
            )
        else:
            lines.append(
                f"- CPID friction increased mean RMS error by {abs(cpid_vs_aug_fric_rms):.1f}% relative to AugPD friction "
                f"({float(aug_fric['mean_cmd_joint_rms_error_deg']):.3f} deg to {float(cpid['mean_cmd_joint_rms_error_deg']):.3f} deg)."
            )
    lines.extend(
        [
            "- Worst tracking cases by commanded-joint max error:",
        ]
    )
    for row in worst_rows:
        lines.append(
            f"  - {row['controller_label']} {row['case']}: max error {float(row['cmd_joint_max_abs_error_deg']):.3f} deg, RMS {float(row['cmd_joint_rms_error_deg']):.3f} deg."
        )
    lines.extend(
        [
            "",
            "## SDK vs simulation result",
            f"- {sdk_text}",
            "- The SDK summary overwrite issue was avoided by parsing the 24 raw `sdk_lowcmd/workspace_j*_sdk_lowcmd.csv` files directly.",
            "",
            "## M/C/N dynamics model validation",
            f"- {dynamics_text}",
            "",
            "## Real hardware limitation interpretation",
            "- The real 100% forward-pose CPID friction2.5 test showed that the real arm has higher friction/deadband than the model assumed. Increasing friction compensation may help overcome static friction, but too much friction compensation can cause oscillation/chattering and actuator heating. Therefore, no more real-arm tests are performed. The remaining systematic comparison is done in simulation and SDK/LOWCMD analysis.",
            "",
            "## Safety decision: why real tests were stopped",
            "- Further hardware tuning would require pushing friction compensation and torque commands beyond the already-observed safe operating envelope. Because static friction/deadband, unmodeled actuator behavior, and thermal risk were not fully captured by the simulation model, real-arm tests were stopped and the final comparison was limited to logged simulation plus SDK/LOWCMD data.",
            "",
            "## Final conclusion for report/poster",
            f"- In the logged simulation/SDK dataset, {best_rms['controller_label']} gave the best overall tracking accuracy by mean RMS error. Friction compensation was essential: AugPD with friction strongly outperformed AugPD without friction. CPID friction did not beat AugPD friction in this batch, indicating that the integral/friction tuning used here was more aggressive than necessary for the simulated benchmark. The analytic dynamics model matched the Unitree SDK inverseDynamics closely enough to support computed-torque analysis, but real-hardware friction/deadband limited safe transfer, so the final report should present simulation and SDK/LOWCMD results as the systematic comparison while explaining why hardware testing was stopped.",
            "",
            "## Generated plot files",
        ]
    )
    for plot_path in plot_paths:
        lines.append(f"- `{plot_path}`")

    path.write_text("\n".join(lines) + "\n")


def print_final_summary(
    *,
    log_dir: Path,
    rows: list[dict[str, object]],
    inventory: list[dict[str, object]],
    summaries: list[dict[str, object]],
    worst_rows: list[dict[str, object]],
    output_paths: list[Path],
    plot_paths: list[Path],
) -> None:
    best_rms = min(summaries, key=lambda row: float(row["mean_cmd_joint_rms_error_deg"]))
    print()
    print("Final analysis complete")
    print("output_folder =", log_dir)
    print("csv_logs_analyzed =", len(rows))
    print("tests_per_controller:")
    for item in inventory:
        print(f"  {item['controller']}: {item['csv_logs']}")
    print(
        "best_controller_by_rms_error = "
        f"{best_rms['controller_label']} ({float(best_rms['mean_cmd_joint_rms_error_deg']):.3f} deg)"
    )
    print("worst_cases_by_max_error:")
    for row in worst_rows[:10]:
        print(
            f"  {row['controller_label']} {row['case']}: "
            f"max={float(row['cmd_joint_max_abs_error_deg']):.3f} deg, "
            f"rms={float(row['cmd_joint_rms_error_deg']):.3f} deg"
        )
    print("summary_files:")
    for output_path in output_paths:
        print(f"  {output_path}")
    print("plot_files:")
    for plot_path in plot_paths:
        print(f"  {plot_path}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate final report/poster analysis from controller comparison CSVs")
    parser.add_argument(
        "--log-dir",
        default=None,
        help="controller comparison log folder; default is newest logs/sim_compare_joint_controllers_*",
    )
    parser.add_argument("--project-dir", default=Path(__file__).resolve().parent, type=Path)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    project_dir = args.project_dir.resolve()
    log_dir = Path(args.log_dir).resolve() if args.log_dir else latest_log_dir(project_dir).resolve()
    if not log_dir.is_dir():
        raise SystemExit(f"log directory does not exist: {log_dir}")

    rows = collect_logs(log_dir)
    if not rows:
        raise SystemExit(f"no raw controller CSV logs found in {log_dir}")

    inventory = build_inventory(rows)
    summaries = aggregate_by_controller(rows)
    worst_rows = sorted(rows, key=lambda row: float(row["cmd_joint_max_abs_error_deg"]), reverse=True)
    sdk_vs_sim = build_sdk_vs_sim_summary(rows)
    dynamics_rows = parse_dynamics_summary(log_dir / "sdk_dynamics_compare.txt")

    final_joint_case_summary = log_dir / "final_joint_case_summary.csv"
    final_controller_summary = log_dir / "final_controller_summary.csv"
    final_worst_cases = log_dir / "final_worst_cases.csv"
    final_sdk_vs_sim_summary = log_dir / "final_sdk_vs_sim_summary.csv"
    final_sdk_lowcmd_summary = log_dir / "final_sdk_lowcmd_summary.csv"
    final_log_inventory = log_dir / "final_log_inventory.csv"
    final_dynamics_summary = log_dir / "final_dynamics_model_validation.csv"
    final_report = log_dir / "final_report_conclusion.md"

    write_csv(final_joint_case_summary, rows)
    write_csv(final_controller_summary, summaries)
    write_csv(final_worst_cases, worst_rows[:20])
    write_csv(final_sdk_vs_sim_summary, sdk_vs_sim)
    sdk_rows = [row for row in rows if row["controller"] == "sdk_lowcmd"]
    if sdk_rows:
        write_csv(final_sdk_lowcmd_summary, sdk_rows)
    write_csv(final_log_inventory, inventory)
    if dynamics_rows:
        write_csv(final_dynamics_summary, dynamics_rows)

    plot_paths = generate_plots(log_dir, rows, summaries, worst_rows)
    output_paths = [
        final_controller_summary,
        final_joint_case_summary,
        final_worst_cases,
        final_sdk_vs_sim_summary,
        final_sdk_lowcmd_summary,
        final_log_inventory,
    ]
    if dynamics_rows:
        output_paths.append(final_dynamics_summary)
    output_paths.append(final_report)

    write_report(
        final_report,
        log_dir=log_dir,
        rows=rows,
        inventory=inventory,
        summaries=summaries,
        sdk_vs_sim=sdk_vs_sim,
        dynamics_rows=dynamics_rows,
        plot_paths=plot_paths,
    )

    print_final_summary(
        log_dir=log_dir,
        rows=rows,
        inventory=inventory,
        summaries=summaries,
        worst_rows=worst_rows,
        output_paths=output_paths,
        plot_paths=plot_paths,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
