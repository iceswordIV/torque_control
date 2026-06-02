#!/usr/bin/env python3
"""Simple file IPC between Python torque control and the C++ Unitree bridge."""

from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

NDOF = 6


def default_runtime_dir() -> Path:
    return Path(f"/tmp/z1_torque_{os.getuid()}")


class FileRobotIO:
    def __init__(self, runtime_dir: Optional[str | os.PathLike[str]] = None, poll_sleep: float = 0.001):
        self.runtime_dir = Path(runtime_dir) if runtime_dir is not None else default_runtime_dir()
        self.poll_sleep = float(poll_sleep)
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.cmd_path = self.runtime_dir / "z1_torque_cmd.txt"
        self.sensor_path = self.runtime_dir / "z1_sensor.txt"
        self.stop_path = self.runtime_dir / "z1_stop.txt"

    def _atomic_write_text(self, path: Path, text: str) -> None:
        fd, tmp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent), text=True)
        try:
            with os.fdopen(fd, "w") as f:
                f.write(text)
            os.replace(tmp_name, path)
        finally:
            if os.path.exists(tmp_name):
                os.unlink(tmp_name)

    def read_state(
        self,
        timeout: Optional[float] = None,
        newer_than_timestamp: Optional[float] = None,
        newer_than_mtime_ns: Optional[int] = None,
    ) -> Tuple[float, np.ndarray, np.ndarray, np.ndarray]:
        deadline = None if timeout is None else time.monotonic() + float(timeout)
        last_error: Optional[Exception] = None
        while True:
            try:
                stat = self.sensor_path.stat()
                if newer_than_mtime_ns is not None and stat.st_mtime_ns <= newer_than_mtime_ns:
                    raise TimeoutError(
                        f"sensor file has not been updated since startup check: {self.sensor_path}"
                    )
                text = self.sensor_path.read_text().strip()
                parts = text.split()
                if len(parts) < 1 + 3 * NDOF:
                    raise ValueError(f"sensor file has {len(parts)} values, expected {1 + 3 * NDOF}")
                values = [float(x) for x in parts[: 1 + 3 * NDOF]]
                timestamp = values[0]
                if newer_than_timestamp is not None and timestamp <= newer_than_timestamp:
                    raise TimeoutError(
                        f"sensor timestamp did not advance past {newer_than_timestamp:g}: {timestamp:g}"
                    )
                q = np.array(values[1:7], dtype=float)
                dq = np.array(values[7:13], dtype=float)
                tau_feedback = np.array(values[13:19], dtype=float)
                if not np.all(np.isfinite(q)):
                    raise ValueError(f"sensor q contains non-finite values: {q}")
                if not np.all(np.isfinite(dq)):
                    raise ValueError(f"sensor dq contains non-finite values: {dq}")
                if not np.all(np.isfinite(tau_feedback)):
                    raise ValueError(f"sensor tau_feedback contains non-finite values: {tau_feedback}")
                return timestamp, q, dq, tau_feedback
            except Exception as exc:
                last_error = exc
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError(f"timed out waiting for valid sensor state at {self.sensor_path}: {last_error}") from exc
                time.sleep(self.poll_sleep)

    def send_torque(self, tau) -> None:
        tau = np.asarray(tau, dtype=float).reshape(-1)
        if tau.size != NDOF:
            raise ValueError(f"tau must contain {NDOF} values, got {tau.size}")
        self._atomic_write_text(self.cmd_path, " ".join(f"{x:.17g}" for x in tau) + "\n")

    def send_zero_torque(self) -> None:
        self.send_torque(np.zeros(NDOF))

    def close(self) -> None:
        pass


_DEFAULT_IO: Optional[FileRobotIO] = None


def _default_io() -> FileRobotIO:
    global _DEFAULT_IO
    if _DEFAULT_IO is None:
        _DEFAULT_IO = FileRobotIO()
    return _DEFAULT_IO


def read_state() -> Tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    return _default_io().read_state()


def send_torque(tau) -> None:
    _default_io().send_torque(tau)


def send_zero_torque() -> None:
    _default_io().send_zero_torque()


def close() -> None:
    _default_io().close()
