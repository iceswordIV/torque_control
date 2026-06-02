#!/usr/bin/env python3
"""Trajectory helpers for the Z1 torque-control examples."""

from __future__ import annotations

import math
import re
from typing import Tuple

import numpy as np

NDOF = 6


def _vec6(values, name: str = "vector") -> np.ndarray:
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {arr.size}")
    return arr


def parse_vec6(text: str) -> np.ndarray:
    parts = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    if len(parts) != NDOF:
        raise ValueError(f"expected {NDOF} values, got {len(parts)} from {text!r}")
    return np.array([float(p) for p in parts], dtype=float)


def quintic_trajectory(q_start, q_goal, t: float, T: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    q_start = _vec6(q_start, "q_start")
    q_goal = _vec6(q_goal, "q_goal")
    t = float(t)
    T = float(T)
    if T <= 0.0:
        raise ValueError("T must be positive")

    if t <= 0.0:
        return q_start.copy(), np.zeros(NDOF), np.zeros(NDOF)
    if t >= T:
        return q_goal.copy(), np.zeros(NDOF), np.zeros(NDOF)

    s = min(1.0, max(0.0, t / T))
    b = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    bd = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / T
    bdd = (60.0 * s - 180.0 * s**2 + 120.0 * s**3) / (T * T)

    delta = q_goal - q_start
    q_des = q_start + b * delta
    dq_des = bd * delta
    ddq_des = bdd * delta
    return q_des, dq_des, ddq_des


# quintic_trajectory is the original profile. scurve_trajectory is a
# 7th-order smooth profile with analytic q, dq, and ddq, which matters because
# torque control depends directly on ddq_des.
def scurve_trajectory(q_start, q_goal, t: float, T: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    q_start = _vec6(q_start, "q_start")
    q_goal = _vec6(q_goal, "q_goal")
    t = float(t)
    T = float(T)
    if T <= 0.0:
        raise ValueError("T must be positive")

    if t <= 0.0:
        return q_start.copy(), np.zeros(NDOF), np.zeros(NDOF)
    if t >= T:
        return q_goal.copy(), np.zeros(NDOF), np.zeros(NDOF)

    s = min(1.0, max(0.0, t / T))
    b = 35.0 * s**4 - 84.0 * s**5 + 70.0 * s**6 - 20.0 * s**7
    bd = (140.0 * s**3 - 420.0 * s**4 + 420.0 * s**5 - 140.0 * s**6) / T
    bdd = (420.0 * s**2 - 1680.0 * s**3 + 2100.0 * s**4 - 840.0 * s**5) / (T * T)

    delta = q_goal - q_start
    q_des = q_start + b * delta
    dq_des = bd * delta
    ddq_des = bdd * delta
    return q_des, dq_des, ddq_des


def make_one_joint_goal(q_start, joint_index_1_based: int, angle_deg: float) -> np.ndarray:
    q_goal = _vec6(q_start, "q_start").copy()
    joint_index = int(joint_index_1_based)
    if not 1 <= joint_index <= NDOF:
        raise ValueError(f"joint_index_1_based must be in 1..{NDOF}, got {joint_index_1_based}")
    q_goal[joint_index - 1] += math.radians(float(angle_deg))
    return q_goal


def make_one_joint_absolute_goal(q_start, joint_index_1_based: int, target) -> np.ndarray:
    q_goal = _vec6(q_start, "q_start").copy()
    joint_index = int(joint_index_1_based)
    if not 1 <= joint_index <= NDOF:
        raise ValueError(f"joint_index_1_based must be in 1..{NDOF}, got {joint_index_1_based}")
    target_values = np.asarray(target, dtype=float).reshape(-1)
    if target_values.size == 1:
        q_goal[joint_index - 1] = float(target_values[0])
    elif target_values.size == NDOF:
        q_goal[joint_index - 1] = float(target_values[joint_index - 1])
    else:
        raise ValueError(f"one_joint_absolute target must contain 1 or {NDOF} values, got {target_values.size}")
    return q_goal


def make_absolute_goal(values6) -> np.ndarray:
    return _vec6(values6, "values6").copy()


def make_scaled_goal(q_start, q_target, scale: float) -> np.ndarray:
    q_start = _vec6(q_start, "q_start")
    q_target = _vec6(q_target, "q_target")
    return q_start + float(scale) * (q_target - q_start)


def build_goal(
    q_start,
    mode: str,
    joint: int = 1,
    angle_deg: float = 0.0,
    target=None,
    scale: float = 1.0,
) -> np.ndarray:
    q_start = _vec6(q_start, "q_start")
    mode = str(mode)
    if mode == "hold_current":
        return q_start.copy()
    if mode == "one_joint_relative":
        return make_one_joint_goal(q_start, joint, angle_deg)
    if mode == "one_joint_absolute":
        if target is None:
            raise ValueError("one_joint_absolute requires target")
        return make_one_joint_absolute_goal(q_start, joint, target)
    if mode == "full_pose_absolute":
        if target is None:
            raise ValueError("full_pose_absolute requires target")
        return make_absolute_goal(target)
    if mode == "scaled_pose":
        if target is None:
            raise ValueError("scaled_pose requires target")
        return make_scaled_goal(q_start, target, scale)
    raise ValueError(f"unknown trajectory mode: {mode}")
