#!/usr/bin/env python3
"""Compare analytic dM dynamics against finite-difference dM dynamics."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

from compare_torque import DEFAULT_TARGET, parse_gain_text, read_actual_csv
from controller import DEFAULT_WN, DEFAULT_ZETA, resolve_gains
from trajectory import NDOF, build_goal, parse_vec6, quintic_trajectory, scurve_trajectory
from z1_analytic_dynamics import dynamics_analytic, dynamics_finite_difference


def parse_float_list(text: str) -> list[float]:
    import re

    values = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    return [float(v) for v in values]


def parse_methods(text: str) -> list[str]:
    import re

    return [p for p in re.split(r"[\s,]+", text.strip()) if p]


def evenly_spaced_indices(size: int, count: int) -> np.ndarray:
    if count <= 0 or count >= size:
        return np.arange(size)
    return np.unique(np.linspace(0, size - 1, count, dtype=int))


def sample_states(args, Kp: np.ndarray, Kd: np.ndarray):
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

    target = parse_vec6(args.target)
    q_goal = build_goal(q_start, args.mode, joint=args.joint, angle_deg=args.angle_deg, target=target, scale=args.scale)
    indices = evenly_spaced_indices(len(t_values), args.samples)

    samples = []
    for idx in indices:
        t = float(t_values[idx])
        q_des, dq_des, ddq_des = trajectory_fn(q_start, q_goal, t - args.hold_time, args.move_time)
        if q_actual_values is None:
            q_dyn = q_des
            dq_dyn = dq_des
        else:
            q_dyn = q_actual_values[idx]
            dq_dyn = dq_actual_values[idx]
        ddq_cmd = ddq_des + Kd @ (dq_des - dq_dyn) + Kp @ (q_des - q_dyn)
        samples.append((t, q_dyn, dq_dyn, ddq_cmd))
    return q_start, q_goal, samples


def average_time_us(fn, samples, repeats: int) -> float:
    for _, q, dq, _ in samples[: min(3, len(samples))]:
        fn(q, dq)
    start = time.perf_counter()
    calls = 0
    for _ in range(repeats):
        for _, q, dq, _ in samples:
            fn(q, dq)
            calls += 1
    elapsed = time.perf_counter() - start
    return elapsed * 1e6 / max(calls, 1)


def compare_for_step(samples, step: float, method: str):
    max_dM_err = 0.0
    max_C_err = 0.0
    max_tau_err = np.zeros(NDOF)
    tau_err_sq = np.zeros(NDOF)
    rows = []

    for sample_index, (t, q, dq, ddq_cmd) in enumerate(samples):
        M_a, C_a, N_a, dM_a = dynamics_analytic(q, dq)
        M_fd, C_fd, N_fd, dM_fd = dynamics_finite_difference(q, dq, step=step, method=method)

        tau_a = M_a @ ddq_cmd + C_a @ dq + N_a
        tau_fd = M_fd @ ddq_cmd + C_fd @ dq + N_fd
        tau_err = tau_fd - tau_a

        dM_err = float(np.max(np.abs(dM_fd - dM_a)))
        C_err = float(np.max(np.abs(C_fd - C_a)))
        max_dM_err = max(max_dM_err, dM_err)
        max_C_err = max(max_C_err, C_err)
        max_tau_err = np.maximum(max_tau_err, np.abs(tau_err))
        tau_err_sq += tau_err * tau_err
        rows.append((sample_index, t, q, dq, tau_a, tau_fd, tau_err, dM_err, C_err))

    rms_tau_err = np.sqrt(tau_err_sq / max(len(samples), 1))
    return {
        "max_dM_err": max_dM_err,
        "max_C_err": max_C_err,
        "max_tau_err": max_tau_err,
        "rms_tau_err": rms_tau_err,
        "rows": rows,
    }


def write_csv(path: str, step_results) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["finite_diff_method", "finite_diff_step", "sample_index", "t"]
            + [f"q_{i + 1}" for i in range(NDOF)]
            + [f"dq_{i + 1}" for i in range(NDOF)]
            + [f"tau_analytic_{i + 1}" for i in range(NDOF)]
            + [f"tau_fd_{i + 1}" for i in range(NDOF)]
            + [f"tau_error_{i + 1}" for i in range(NDOF)]
            + ["max_dM_error", "max_C_error"]
        )
        for method, step, result in step_results:
            for sample_index, t, q, dq, tau_a, tau_fd, tau_err, dM_err, C_err in result["rows"]:
                writer.writerow([method, step, sample_index, t, *q, *dq, *tau_a, *tau_fd, *tau_err, dM_err, C_err])


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare analytic and finite-difference Z1 dynamics", allow_abbrev=False)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument(
        "--mode",
        choices=["hold_current", "one_joint_relative", "full_pose_absolute", "scaled_pose"],
        default="full_pose_absolute",
    )
    parser.add_argument("--joint", type=int, default=1)
    parser.add_argument("--angle-deg", "--angle", dest="angle_deg", type=float, default=5.0)
    parser.add_argument("--target", type=str, default=DEFAULT_TARGET)
    parser.add_argument("--scale", type=float, default=0.25)
    parser.add_argument("--move-time", type=float, default=6.0)
    parser.add_argument("--hold-time", type=float, default=0.0)
    parser.add_argument("--trajectory-profile", choices=["quintic", "scurve"], default="scurve")
    parser.add_argument("--kp", type=str, default=None)
    parser.add_argument("--kd", type=str, default=None)
    parser.add_argument("--wn", type=str, default=None)
    parser.add_argument("--zeta", type=str, default=None)
    parser.add_argument("--actual-csv", type=str, default=None)
    parser.add_argument("--finite-diff-steps", type=str, default="1e-3 1e-4 1e-5 1e-6")
    parser.add_argument("--finite-diff-methods", type=str, default="central forward")
    parser.add_argument("--samples", type=int, default=200, help="number of trajectory/log samples to compare")
    parser.add_argument("--timing-repeats", type=int, default=5)
    parser.add_argument("--csv-log", type=str, default=None)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.duration < 0.0:
        raise ValueError("--duration must be non-negative")
    if args.samples <= 0:
        raise ValueError("--samples must be positive")
    if args.timing_repeats <= 0:
        raise ValueError("--timing-repeats must be positive")

    kp = parse_gain_text(args.kp)
    kd = parse_gain_text(args.kd)
    wn = DEFAULT_WN if args.wn is None else parse_vec6(args.wn)
    zeta = DEFAULT_ZETA if args.zeta is None else parse_vec6(args.zeta)
    Kp, Kd = resolve_gains(kp=kp, kd=kd, wn=wn, zeta=zeta)
    steps = parse_float_list(args.finite_diff_steps)
    if any(step <= 0.0 for step in steps):
        raise ValueError("--finite-diff-steps must all be positive")
    methods = [str(method) for method in parse_methods(args.finite_diff_methods)]
    if any(method not in ("central", "forward") for method in methods):
        raise ValueError("--finite-diff-methods may contain only 'central' and 'forward'")

    q_start, q_goal, samples = sample_states(args, Kp, Kd)
    print("samples =", len(samples))
    print("trajectory mode =", args.mode)
    print("trajectory_profile =", args.trajectory_profile)
    print("q_start =", np.array2string(q_start, precision=6, suppress_small=False))
    print("q_goal =", np.array2string(q_goal, precision=6, suppress_small=False))

    analytic_us = average_time_us(dynamics_analytic, samples, args.timing_repeats)
    print(f"analytic dynamics avg = {analytic_us:.2f} us ({1e6 / analytic_us:.1f} Hz equivalent)")
    print("finite-difference comparison:")
    print("method   step        avg_us    equiv_Hz  max_dM_err   max_C_err    max_tau_err  rms_tau_err")

    step_results = []
    for method in methods:
        for step in steps:
            fn = lambda q, dq, step=step, method=method: dynamics_finite_difference(q, dq, step=step, method=method)
            fd_us = average_time_us(fn, samples, args.timing_repeats)
            result = compare_for_step(samples, step, method)
            step_results.append((method, step, result))
            max_tau_scalar = float(np.max(result["max_tau_err"]))
            rms_tau_scalar = float(np.max(result["rms_tau_err"]))
            print(
                f"{method:<8} {step:<10.1e} {fd_us:8.2f} {1e6 / fd_us:10.1f} "
                f"{result['max_dM_err']:11.3e} {result['max_C_err']:11.3e} "
                f"{max_tau_scalar:12.3e} {rms_tau_scalar:12.3e}"
            )
            print("  max tau err by joint =", np.array2string(result["max_tau_err"], precision=3, suppress_small=False))

    if args.csv_log:
        write_csv(args.csv_log, step_results)
        print("CSV path =", args.csv_log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
