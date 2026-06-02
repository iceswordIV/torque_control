#!/usr/bin/env python3
"""Replay a recorded torque trace through the file/ROS torque bridge.

The input CSV is expected to contain either:
- `tau_effort_1..6` columns, as recorded from Gazebo joint states, or
- `tau_1..6` / `tau_total_1..6` columns.

The script interpolates the recorded torque trace in real time, sends it to
`robot_io.FileRobotIO`, and logs the replay response using the bridge sensor
file. It prefers `q_cmd`/`dq_cmd` or `q_des`/`dq_des` columns for the reference
trajectory, then falls back to measured `q_actual`/`dq_actual`.
"""

from __future__ import annotations

import argparse
import csv
import signal
import time
from pathlib import Path

import numpy as np

from robot_io import FileRobotIO, NDOF, default_runtime_dir


def field(row: dict[str, str], candidates: list[str]) -> str:
    for key in candidates:
        value = row.get(key, "")
        if value != "":
            return value
    raise KeyError(f"missing any of columns {candidates}")


def read_trace(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t_values: list[float] = []
    q_values: list[np.ndarray] = []
    dq_values: list[np.ndarray] = []
    tau_values: list[np.ndarray] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t_values.append(float(field(row, ["t", "time", "timestamp"])))
            q_values.append(
                np.array(
                    [
                        float(field(row, [f"q_cmd_{i}", f"q_des_{i}", f"q_actual_{i}", f"q_{i}", f"q{i}"]))
                        for i in range(1, NDOF + 1)
                    ],
                    dtype=float,
                )
            )
            dq_values.append(
                np.array(
                    [
                        float(field(row, [f"dq_cmd_{i}", f"dq_des_{i}", f"dq_actual_{i}", f"dq_{i}", f"dq{i}"]))
                        for i in range(1, NDOF + 1)
                    ],
                    dtype=float,
                )
            )
            tau_values.append(
                np.array(
                    [
                        float(
                            field(
                                row,
                                [
                                    f"tau_cmd_{i}",
                                    f"tau_effort_{i}",
                                    f"tau_total_{i}",
                                    f"tau_{i}",
                                ],
                            )
                        )
                        for i in range(1, NDOF + 1)
                    ],
                    dtype=float,
                )
            )

    if not t_values:
        raise ValueError(f"no usable rows found in {path}")

    t = np.array(t_values, dtype=float)
    t = t - t[0]
    return t, np.vstack(q_values), np.vstack(dq_values), np.vstack(tau_values)


def interp_matrix(t_src: np.ndarray, values: np.ndarray, t_query: float) -> np.ndarray:
    t_query = float(np.clip(t_query, t_src[0], t_src[-1]))
    out = np.empty(values.shape[1], dtype=float)
    for i in range(values.shape[1]):
        out[i] = np.interp(t_query, t_src, values[:, i])
    return out


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t"]
        + [f"q_des_{i + 1}" for i in range(NDOF)]
        + [f"dq_des_{i + 1}" for i in range(NDOF)]
        + [f"tau_cmd_{i + 1}" for i in range(NDOF)]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + [f"tau_feedback_{i + 1}" for i in range(NDOF)]
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a recorded torque trace through FileRobotIO")
    parser.add_argument("input_csv", help="recorded torque trace CSV")
    parser.add_argument("--csv-log", default=str(Path("logs") / f"replay_{time.strftime('%Y%m%d_%H%M%S')}.csv"))
    parser.add_argument("--runtime-dir", default=str(default_runtime_dir()))
    parser.add_argument("--rate-hz", type=float, default=500.0)
    parser.add_argument("--hold-zero-sec", type=float, default=0.2, help="send zero torque this long after replay")
    args = parser.parse_args()

    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if args.hold_zero_sec < 0.0:
        raise ValueError("--hold-zero-sec must be non-negative")
    if not Path(args.input_csv).is_file():
        raise FileNotFoundError(
            f"input CSV does not exist: {args.input_csv}. "
            "Create it first with record_sdk_forward_torque.py, or pass an existing torque log."
        )
    if Path(args.csv_log).exists() and Path(args.csv_log).is_dir():
        raise IsADirectoryError(f"--csv-log must be a CSV file path, not a directory: {args.csv_log}")
    if Path(args.input_csv).resolve() == Path(args.csv_log).resolve():
        raise ValueError(f"--csv-log must be different from input_csv; refusing to overwrite {args.input_csv}")

    t_src, q_ref, dq_ref, tau_ref = read_trace(args.input_csv)
    dt = 1.0 / args.rate_hz
    io = FileRobotIO(args.runtime_dir)

    print(f"loading replay trace from {args.input_csv}")
    print(f"trace duration = {t_src[-1]:.6f} s, samples = {len(t_src)}")
    print(f"waiting for bridge sensor file at {io.sensor_path}")
    sensor_stamp, q_start, dq_start, _ = io.read_state(timeout=5.0)
    print("bridge start q =", np.array2string(q_start, precision=6, suppress_small=False))
    print("bridge start dq =", np.array2string(dq_start, precision=6, suppress_small=False))
    print("reference start q =", np.array2string(q_ref[0], precision=6, suppress_small=False))
    print("reference start dq =", np.array2string(dq_ref[0], precision=6, suppress_small=False))
    print("initial q error =", np.array2string(q_start - q_ref[0], precision=6, suppress_small=False))
    print("initial dq error =", np.array2string(dq_start - dq_ref[0], precision=6, suppress_small=False))

    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    Path(args.csv_log).parent.mkdir(parents=True, exist_ok=True)
    with open(args.csv_log, "w", newline="") as f:
        writer = csv.writer(f)
        write_header(writer)

        max_error = np.zeros(NDOF)
        final_error = np.zeros(NDOF)
        final_q = q_start.copy()
        final_sensor_stamp = sensor_stamp

        t0 = time.perf_counter()
        next_tick = t0
        step = 0
        while not stop_requested:
            now = time.perf_counter()
            t_elapsed = now - t0
            if t_elapsed > t_src[-1]:
                break

            tau_cmd = interp_matrix(t_src, tau_ref, t_elapsed)
            q_des = interp_matrix(t_src, q_ref, t_elapsed)
            dq_des = interp_matrix(t_src, dq_ref, t_elapsed)
            io.send_torque(tau_cmd)

            sensor_stamp, q_actual, dq_actual, tau_feedback = io.read_state(
                timeout=1.0,
                newer_than_timestamp=final_sensor_stamp,
            )
            final_sensor_stamp = sensor_stamp

            error = q_des - q_actual
            max_error = np.maximum(max_error, np.abs(error))
            final_error = error
            final_q = q_actual

            writer.writerow(
                [
                    t_elapsed,
                    *q_des.tolist(),
                    *dq_des.tolist(),
                    *tau_cmd.tolist(),
                    *q_actual.tolist(),
                    *dq_actual.tolist(),
                    *tau_feedback.tolist(),
                ]
            )
            step += 1

            next_tick += dt
            sleep_sec = next_tick - time.perf_counter()
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
            else:
                next_tick = time.perf_counter()

        if args.hold_zero_sec > 0.0:
            zero_end = time.perf_counter() + args.hold_zero_sec
            zero = np.zeros(NDOF)
            while time.perf_counter() < zero_end and not stop_requested:
                io.send_torque(zero)
                try:
                    sensor_stamp, q_actual, dq_actual, tau_feedback = io.read_state(
                        timeout=1.0,
                        newer_than_timestamp=final_sensor_stamp,
                    )
                    final_sensor_stamp = sensor_stamp
                    writer.writerow(
                        [
                            time.perf_counter() - t0,
                            *q_ref[-1].tolist(),
                            *dq_ref[-1].tolist(),
                            *zero.tolist(),
                            *q_actual.tolist(),
                            *dq_actual.tolist(),
                            *tau_feedback.tolist(),
                        ]
                    )
                except TimeoutError:
                    break
                time.sleep(dt)

    print("replay steps =", step)
    print("final q =", np.array2string(final_q, precision=6, suppress_small=False))
    print("final error =", np.array2string(final_error, precision=6, suppress_small=False))
    print("max error =", np.array2string(max_error, precision=6, suppress_small=False))
    print("CSV path =", args.csv_log)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
