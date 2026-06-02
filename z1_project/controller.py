#!/usr/bin/env python3
"""Computed-torque control for Unitree Z1 joint-space commands."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from z1_analytic_dynamics import NDOF, dynamics_analytic, dynamics_finite_difference

DEFAULT_WN = np.array([2.0, 2.0, 1.5, 1.5, 1.2, 1.2], dtype=float)
DEFAULT_ZETA = np.ones(NDOF, dtype=float)
DYNAMICS_MODES = ("analytic", "finite_difference")


def _vec6(values, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {arr.size}")
    return arr


def _gain_matrix(values, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.shape == (NDOF, NDOF):
        return arr.copy()
    flat = arr.reshape(-1)
    if flat.size == NDOF:
        return np.diag(flat)
    if flat.size == NDOF * NDOF:
        return flat.reshape(NDOF, NDOF)
    raise ValueError(f"{name} must be a 6-vector or 6x6 matrix, got shape {arr.shape}")


def gains_from_wn_zeta(wn, zeta) -> Tuple[np.ndarray, np.ndarray]:
    wn = _vec6(wn, "wn")
    zeta = _vec6(zeta, "zeta")
    kp = wn**2
    kd = 2.0 * zeta * wn
    return np.diag(kp), np.diag(kd)


def resolve_gains(
    kp: Optional[np.ndarray] = None,
    kd: Optional[np.ndarray] = None,
    wn: Optional[np.ndarray] = None,
    zeta: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    if kp is not None or kd is not None:
        if kp is None or kd is None:
            raise ValueError("kp and kd must be provided together")
        return _gain_matrix(kp, "kp"), _gain_matrix(kd, "kd")
    wn = DEFAULT_WN if wn is None else _vec6(wn, "wn")
    zeta = DEFAULT_ZETA if zeta is None else _vec6(zeta, "zeta")
    return gains_from_wn_zeta(wn, zeta)


def dynamics_for_mode(
    q,
    dq,
    mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
):
    mode = str(mode)
    if mode == "analytic":
        return dynamics_analytic(q, dq)
    if mode == "finite_difference":
        return dynamics_finite_difference(q, dq, step=finite_diff_step, method=finite_diff_method)
    raise ValueError(f"unknown dynamics mode: {mode}")


def compute_tau(
    q,
    dq,
    q_des,
    dq_des,
    ddq_des,
    kp: Optional[np.ndarray] = None,
    kd: Optional[np.ndarray] = None,
    wn: Optional[np.ndarray] = None,
    zeta: Optional[np.ndarray] = None,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> np.ndarray:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    ddq_des = _vec6(ddq_des, "ddq_des")
    Kp, Kd = resolve_gains(kp=kp, kd=kd, wn=wn, zeta=zeta)

    e = q_des - q
    de = dq_des - dq
    ddq_cmd = ddq_des + Kd @ de + Kp @ e

    M, C, N, _ = dynamics_for_mode(
        q,
        dq,
        mode=dynamics_mode,
        finite_diff_step=finite_diff_step,
        finite_diff_method=finite_diff_method,
    )
    tau = M @ ddq_cmd + C @ dq + N
    return tau
