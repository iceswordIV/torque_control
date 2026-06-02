#!/usr/bin/env python3
"""ROS/Gazebo file bridge for torque_main.py.

This bridge matches the file IPC used by robot_io.py:

- reads commanded torque from z1_torque_cmd.txt
- writes measured q/dq and active torque to z1_sensor.txt
- publishes torque-only MotorCmd messages to the Unitree Gazebo joint controllers
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

try:
    import rospy
    from sensor_msgs.msg import JointState
    from unitree_legged_msgs.msg import MotorCmd, MotorState
except ImportError as exc:  # pragma: no cover - depends on sourced ROS env
    raise SystemExit(f"failed to import ROS Python modules; source your ROS workspace first: {exc}") from exc

NDOF = 6
PMSM_MODE = 0x0A
DEFAULT_JOINT_NAMES = [f"joint{i}" for i in range(1, NDOF + 1)]
DEFAULT_CONTROLLER_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/command" for i in range(1, NDOF + 1)]
DEFAULT_STATE_TOPICS = [f"/z1_gazebo/Joint{i:02d}_controller/state" for i in range(1, NDOF + 1)]


def default_runtime_dir() -> Path:
    return Path(f"/tmp/z1_torque_{os.getuid()}")


def parse_list(text: str) -> list[str]:
    return [part for part in text.replace(",", " ").split() if part]


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent), text=True)
    try:
        with os.fdopen(fd, "w") as f:
            f.write(text)
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def read_torque_command(path: Path) -> np.ndarray:
    try:
        parts = path.read_text().split()
        if len(parts) < NDOF:
            return np.zeros(NDOF)
        return np.array([float(x) for x in parts[:NDOF]], dtype=float)
    except Exception:
        return np.zeros(NDOF)


class RosTorqueBridge:
    def __init__(
        self,
        runtime_dir: Path,
        state_source: str,
        joint_state_topic: str,
        joint_names: list[str],
        command_topics: list[str],
        state_topics: list[str],
        rate_hz: float,
    ):
        if len(joint_names) != NDOF:
            raise ValueError(f"expected {NDOF} joint names, got {len(joint_names)}")
        if len(command_topics) != NDOF:
            raise ValueError(f"expected {NDOF} command topics, got {len(command_topics)}")
        if state_source == "controller_states" and len(state_topics) != NDOF:
            raise ValueError(f"expected {NDOF} controller state topics, got {len(state_topics)}")
        if rate_hz <= 0.0:
            raise ValueError("--rate-hz must be positive")
        if state_source not in ("joint_states", "controller_states"):
            raise ValueError(f"unknown state source: {state_source}")

        self.runtime_dir = runtime_dir
        self.cmd_path = runtime_dir / "z1_torque_cmd.txt"
        self.sensor_path = runtime_dir / "z1_sensor.txt"
        self.stop_path = runtime_dir / "z1_stop.txt"
        self.joint_names = joint_names
        self.rate_hz = rate_hz
        self._lock = threading.Lock()
        self._state: Optional[Tuple[float, np.ndarray, np.ndarray]] = None
        self._controller_q = np.zeros(NDOF)
        self._controller_dq = np.zeros(NDOF)
        self._controller_seen = np.zeros(NDOF, dtype=bool)
        self._running = True

        runtime_dir.mkdir(parents=True, exist_ok=True)
        if self.stop_path.exists():
            self.stop_path.unlink()

        self._publishers = [rospy.Publisher(topic, MotorCmd, queue_size=1) for topic in command_topics]
        if state_source == "joint_states":
            self._subscribers = [rospy.Subscriber(joint_state_topic, JointState, self._joint_state_callback, queue_size=1)]
        else:
            self._subscribers = [
                rospy.Subscriber(topic, MotorState, lambda msg, idx=idx: self._controller_state_callback(idx, msg), queue_size=1)
                for idx, topic in enumerate(state_topics)
            ]

    def _joint_state_callback(self, msg: JointState) -> None:
        name_to_index: Dict[str, int] = {name: idx for idx, name in enumerate(msg.name)}
        try:
            indices = [name_to_index[name] for name in self.joint_names]
        except KeyError as exc:
            rospy.logwarn_throttle(2.0, "joint_states missing expected joint %s; names=%s", exc, list(msg.name))
            return

        q = np.array([msg.position[i] if i < len(msg.position) else 0.0 for i in indices], dtype=float)
        # Some Gazebo Z1 joint_states messages publish NaN velocity values even
        # when the per-joint controller state has valid dq. Missing velocities
        # are treated as zero, and non-finite q/dq are sanitized before writing
        # the file IPC state consumed by torque_main.py.
        dq = np.array([msg.velocity[i] if i < len(msg.velocity) else 0.0 for i in indices], dtype=float)
        if not np.all(np.isfinite(q)):
            rospy.logwarn_throttle(2.0, "joint_states position contains NaN/inf; replacing non-finite q with 0.0")
            q = np.nan_to_num(q, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        if not np.all(np.isfinite(dq)):
            rospy.logwarn_throttle(2.0, "joint_states velocity contains NaN/inf; replacing non-finite dq with 0.0")
            dq = np.nan_to_num(dq, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        if stamp <= 0.0:
            stamp = time.monotonic()

        with self._lock:
            self._state = (stamp, q, dq)

    def _controller_state_callback(self, idx: int, msg: MotorState) -> None:
        with self._lock:
            self._controller_q[idx] = float(msg.q)
            self._controller_dq[idx] = float(msg.dq)
            self._controller_seen[idx] = True
            if np.all(self._controller_seen):
                q = np.nan_to_num(self._controller_q.copy(), nan=0.0, posinf=0.0, neginf=0.0)
                dq = np.nan_to_num(self._controller_dq.copy(), nan=0.0, posinf=0.0, neginf=0.0)
                self._state = (time.monotonic(), q, dq)

    def _publish_torque(self, tau: np.ndarray, q: np.ndarray, dq: np.ndarray) -> None:
        tau = np.asarray(tau, dtype=float).reshape(-1)
        q = np.asarray(q, dtype=float).reshape(-1)
        dq = np.asarray(dq, dtype=float).reshape(-1)
        if tau.size != NDOF:
            raise ValueError(f"tau must contain {NDOF} values, got {tau.size}")
        if q.size != NDOF:
            raise ValueError(f"q must contain {NDOF} values, got {q.size}")
        if dq.size != NDOF:
            raise ValueError(f"dq must contain {NDOF} values, got {dq.size}")

        for i, (pub, torque) in enumerate(zip(self._publishers, tau)):
            msg = MotorCmd()
            msg.mode = PMSM_MODE
            msg.q = float(q[i])
            msg.dq = float(dq[i])
            msg.tau = float(torque)
            msg.Kp = 0.0
            msg.Kd = 0.0
            try:
                pub.publish(msg)
            except rospy.ROSException:
                if rospy.is_shutdown() or not self._running:
                    return
                raise

    def _write_sensor(self, stamp: float, q: np.ndarray, dq: np.ndarray, tau: np.ndarray) -> None:
        values = [stamp, *q.tolist(), *dq.tolist(), *tau.tolist()]
        atomic_write_text(self.sensor_path, " ".join(f"{x:.17g}" for x in values) + "\n")

    def shutdown(self) -> None:
        self._running = False
        zero = np.zeros(NDOF)
        with self._lock:
            state = self._state
        if state is None:
            q = np.zeros(NDOF)
            dq = np.zeros(NDOF)
        else:
            _, q, dq = state
        for _ in range(20):
            self._publish_torque(zero, q, dq)
            rospy.sleep(1.0 / self.rate_hz)

    def run(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        last_print = time.monotonic()
        loops = 0

        while not rospy.is_shutdown() and self._running:
            if self.stop_path.exists():
                break

            with self._lock:
                state = self._state

            if state is None:
                rospy.loginfo_throttle(1.0, "waiting for robot state feedback")
                rate.sleep()
                continue

            stamp, q, dq = state
            tau = read_torque_command(self.cmd_path)
            self._write_sensor(stamp, q, dq, tau)
            self._publish_torque(tau, q, dq)

            loops += 1
            now = time.monotonic()
            if now - last_print >= 1.0:
                rospy.loginfo(
                    "rate %.1f Hz, command q/dq are measured actual state, q %s, dq %s, tau %s",
                    loops / (now - last_print),
                    np.array2string(q, precision=4),
                    np.array2string(dq, precision=4),
                    np.array2string(tau, precision=4),
                )
                loops = 0
                last_print = now

            rate.sleep()

        self.shutdown()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS/Gazebo file bridge for torque_main.py")
    parser.add_argument("--runtime-dir", type=Path, default=default_runtime_dir())
    parser.add_argument("--state-source", choices=["joint_states", "controller_states"], default="joint_states")
    parser.add_argument("--joint-state-topic", default="/z1_gazebo/joint_states")
    parser.add_argument("--joint-names", default=" ".join(DEFAULT_JOINT_NAMES))
    parser.add_argument("--command-topics", default=" ".join(DEFAULT_CONTROLLER_TOPICS))
    parser.add_argument("--state-topics", default=" ".join(DEFAULT_STATE_TOPICS))
    parser.add_argument("--rate-hz", type=float, default=500.0)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    rospy.init_node("z1_ros_torque_bridge", anonymous=False)

    bridge = RosTorqueBridge(
        runtime_dir=args.runtime_dir,
        state_source=args.state_source,
        joint_state_topic=args.joint_state_topic,
        joint_names=parse_list(args.joint_names),
        command_topics=parse_list(args.command_topics),
        state_topics=parse_list(args.state_topics),
        rate_hz=args.rate_hz,
    )

    def stop(signum, frame):  # noqa: ARG001
        bridge.shutdown()
        rospy.signal_shutdown(f"signal {signum}")

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    bridge.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
