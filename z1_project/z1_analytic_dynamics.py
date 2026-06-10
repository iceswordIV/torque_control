#!/usr/bin/env python3
"""Analytic Unitree Z1 dynamics.

This module is intentionally pure math. It contains no robot communication or
trajectory generation.

Twist convention: xi = [v; w].
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Optional, Tuple

import numpy as np

NDOF = 6

try:
    if os.environ.get("Z1_DISABLE_FAST_DYNAMICS"):
        _FAST_DYNAMICS = None
    else:
        import z1_analytic_dynamics_fast as _FAST_DYNAMICS
except ImportError:  # pragma: no cover - optional compiled accelerator
    _FAST_DYNAMICS = None


def skew3(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a).reshape(3)
    return np.array(
        [
            [0.0, -a[2], a[1]],
            [a[2], 0.0, -a[0]],
            [-a[1], a[0], 0.0],
        ],
        dtype=np.result_type(a, float),
    )


def inertia_from_urdf(ixx, ixy, ixz, iyy, iyz, izz) -> np.ndarray:
    return np.array(
        [
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ],
        dtype=float,
    )


def parallel_axis_theorem(I_body: np.ndarray, m_body: float, displacement: np.ndarray) -> np.ndarray:
    d = np.asarray(displacement, dtype=float).reshape(3)
    return I_body + m_body * ((d @ d) * np.eye(3) - np.outer(d, d))


def make_g(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    dtype = np.result_type(R, p, float)
    g = np.eye(4, dtype=dtype)
    g[:3, :3] = R
    g[:3, 3] = np.asarray(p).reshape(3)
    return g


def adjoint(g: np.ndarray) -> np.ndarray:
    """Adjoint for twist convention xi=[v;w]."""
    dtype = np.result_type(g, float)
    R = g[:3, :3]
    p = g[:3, 3]
    Ad = np.zeros((6, 6), dtype=dtype)
    Ad[:3, :3] = R
    Ad[:3, 3:6] = skew3(p) @ R
    Ad[3:6, 3:6] = R
    return Ad


def adjoint_inverse(g: np.ndarray) -> np.ndarray:
    """Inverse adjoint for twist convention xi=[v;w]."""
    dtype = np.result_type(g, float)
    R = g[:3, :3]
    p = g[:3, 3]
    Adi = np.zeros((6, 6), dtype=dtype)
    Adi[:3, :3] = R.T
    Adi[:3, 3:6] = -R.T @ skew3(p)
    Adi[3:6, 3:6] = R.T
    return Adi


def rodrigues(w: np.ndarray, theta: float | complex) -> np.ndarray:
    w = np.asarray(w)
    dtype = np.result_type(w, theta, float)
    w = w.astype(dtype).reshape(3)
    norm_w = np.linalg.norm(w.astype(complex if np.iscomplexobj(w) else float))
    I3 = np.eye(3, dtype=dtype)
    W = skew3(w)
    if abs(norm_w - 1.0) < 1e-12:
        return I3 + W * np.sin(theta) + (W @ W) * (1.0 - np.cos(theta))
    return I3 + (W / norm_w) * np.sin(norm_w * theta) + (W @ W / (norm_w * norm_w)) * (1.0 - np.cos(norm_w * theta))


def twist_exp(w: np.ndarray, v: np.ndarray, theta: float | complex) -> np.ndarray:
    w = np.asarray(w)
    v = np.asarray(v)
    dtype = np.result_type(w, v, theta, float)
    w = w.astype(dtype).reshape(3)
    v = v.astype(dtype).reshape(3)
    R = rodrigues(w, theta)
    p = (np.eye(3, dtype=dtype) - R) @ (skew3(w) @ v) + w * (w.T @ v) * theta
    return make_g(R, p)


def lie_bracket_vw(xi1: np.ndarray, xi2: np.ndarray) -> np.ndarray:
    """Lie bracket for twists xi=[v;w]."""
    xi1 = np.asarray(xi1, dtype=float)
    xi2 = np.asarray(xi2, dtype=float)
    v1x, v1y, v1z = xi1[0], xi1[1], xi1[2]
    w1x, w1y, w1z = xi1[3], xi1[4], xi1[5]
    v2x, v2y, v2z = xi2[0], xi2[1], xi2[2]
    w2x, w2y, w2z = xi2[3], xi2[4], xi2[5]
    return np.array(
        [
            w1y * v2z - w1z * v2y + v1y * w2z - v1z * w2y,
            w1z * v2x - w1x * v2z + v1z * w2x - v1x * w2z,
            w1x * v2y - w1y * v2x + v1x * w2y - v1y * w2x,
            w1y * w2z - w1z * w2y,
            w1z * w2x - w1x * w2z,
            w1x * w2y - w1y * w2x,
        ],
        dtype=float,
    )


@dataclass
class Z1Params:
    g_const: float
    m_vec: np.ndarray
    I: Tuple[np.ndarray, ...]
    w: np.ndarray
    q_axis: np.ndarray
    c: np.ndarray
    gsl0: Tuple[np.ndarray, ...]


_PARAMS_CACHE: Optional[Z1Params] = None


def z1_parameters() -> Z1Params:
    """Return the Z1 inertial and kinematic constants used by the reference model."""
    global _PARAMS_CACHE
    if _PARAMS_CACHE is not None:
        return _PARAMS_CACHE

    g_const = 9.80665
    m_vec = np.array([0.67332551, 1.19132258, 0.83940874, 0.56404563, 0.38938492, 0.0], dtype=float)

    I1 = inertia_from_urdf(0.00128328, -6e-08, -4e-07, 0.00071931, 5e-07, 0.00083936)
    I2 = inertia_from_urdf(0.00102138, 0.00062358, 5.13e-06, 0.02429457, -2.1e-06, 0.02466114)
    I3 = inertia_from_urdf(0.00108061, -8.669e-05, -0.00208102, 0.00954238, -1.332e-05, 0.00886621)
    I4 = inertia_from_urdf(0.00031576, 8.13e-05, 4.091e-05, 0.00092996, -5.96e-06, 0.00097912)
    I5 = inertia_from_urdf(0.00017605, 4e-07, 5.689e-05, 0.00055896, -1.3e-07, 0.00053860)

    # Equivalent final-link parameters identified from Unitree SDK
    # ArmInterface(hasGripper=True).armModel.inverseDynamics(). The SDK does
    # not expose its link inertias directly, so this is fitted from recovered
    # M(q) over multiple poses and keeps this Python model comparable to SDK.
    m_vec[5] = 1.09473306147355
    c6_sdk = np.array([0.023210304542, -0.000363250494, 0.002026681669])
    I6 = inertia_from_urdf(
        0.003367542789986,
        -1.884722331677e-05,
        2.829437074001e-04,
        0.002529152661291,
        2.211249714817e-05,
        0.002713697251443,
    )

    w = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 1.0, 1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        ],
        dtype=float,
    )

    q_axis = np.array(
        [
            [0.0, 0.0, -0.35, -0.132, -0.062, -0.0128],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0585, 0.1035, 0.1035, 0.1605, 0.1605, 0.1605],
        ],
        dtype=float,
    )

    c = np.array(
        [
            [2.47e-06, -0.11012601, 0.10609208, 0.04366681, 0.03121533, c6_sdk[0]],
            [-0.00025198, 0.00240029, -0.00541815, 0.00364738, 0.0, c6_sdk[1]],
            [0.02317169, 0.00158266, 0.03476383, -0.00170192, 0.00646316, c6_sdk[2]],
        ],
        dtype=float,
    )

    I = (I1, I2, I3, I4, I5, I6)
    gsl0 = tuple(make_g(np.eye(3), q_axis[:, i] + c[:, i]) for i in range(NDOF))
    _PARAMS_CACHE = Z1Params(g_const=g_const, m_vec=m_vec, I=I, w=w, q_axis=q_axis, c=c, gsl0=gsl0)
    return _PARAMS_CACHE


def _as_vec6(x: np.ndarray, name: str) -> np.ndarray:
    arr = np.asarray(x, dtype=float).reshape(-1)
    if arr.size != NDOF:
        raise ValueError(f"{name} must contain {NDOF} values, got {arr.size}")
    return arr


def z1_twists(p: Optional[Z1Params] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    p = z1_parameters() if p is None else p
    w_list = p.w.copy()
    v_list = np.zeros((3, NDOF), dtype=float)
    twists = np.zeros((6, NDOF), dtype=float)
    for i in range(NDOF):
        w = w_list[:, i]
        q0 = p.q_axis[:, i]
        v = -np.cross(w, q0)
        v_list[:, i] = v
        twists[:, i] = np.r_[v, w]
    return twists, w_list, v_list


def compute_A_and_Gp(q: np.ndarray, p: Optional[Z1Params] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    p = z1_parameters() if p is None else p
    q = _as_vec6(q, "q")
    xi, w_list, v_list = z1_twists(p)
    E = [twist_exp(w_list[:, i], v_list[:, i], q[i]) for i in range(NDOF)]

    A = np.zeros((NDOF, NDOF, 6, 6), dtype=float)
    for l in range(NDOF):
        for j in range(l + 1):
            if l == j:
                A[l, j] = np.eye(6)
            else:
                g = np.eye(4)
                for r in range(j + 1, l + 1):
                    g = g @ E[r]
                A[l, j] = adjoint_inverse(g)

    Gp = np.zeros((NDOF, 6, 6), dtype=float)
    for l in range(NDOF):
        G = np.zeros((6, 6), dtype=float)
        G[:3, :3] = p.m_vec[l] * np.eye(3)
        G[3:6, 3:6] = p.I[l]
        A0 = adjoint_inverse(p.gsl0[l])
        Gp[l] = A0.T @ G @ A0
    return A, Gp, xi


def dJ_column_analytic(l: int, j: int, k: int, A: np.ndarray, xi: np.ndarray) -> np.ndarray:
    """d/dq_k of J_lj = A_lj xi_j. Indices are 0-based."""
    if not (l >= k >= j):
        return np.zeros(6)
    return A[l, k] @ lie_bracket_vw(A[k, j] @ xi[:, j], xi[:, k])


def mass_and_dM_analytic(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return M(q) and analytic dM/dq using the Lie-bracket dJ formula."""
    q = _as_vec6(q, "q")
    if _FAST_DYNAMICS is not None:
        return _FAST_DYNAMICS.mass_and_dM(q)

    A, Gp, xi = compute_A_and_Gp(q)

    J = np.zeros((NDOF, NDOF, 6), dtype=float)
    for l in range(NDOF):
        for j in range(l + 1):
            J[l, j] = A[l, j] @ xi[:, j]

    GJ = np.einsum("lab,ljb->lja", Gp, J)

    M = np.zeros((NDOF, NDOF), dtype=float)
    for i in range(NDOF):
        for j in range(NDOF):
            val = 0.0
            for l in range(max(i, j), NDOF):
                val += J[l, i] @ GJ[l, j]
            M[i, j] = val
    M = 0.5 * (M + M.T)

    dJ = np.zeros((NDOF, NDOF, NDOF, 6), dtype=float)
    for l in range(NDOF):
        for j in range(l + 1):
            for k in range(j, l + 1):
                dJ[l, j, k] = dJ_column_analytic(l, j, k, A, xi)

    GdJ = np.einsum("lab,ljkb->ljka", Gp, dJ)

    dM = np.zeros((NDOF, NDOF, NDOF), dtype=float)
    for k in range(NDOF):
        for i in range(NDOF):
            for j in range(NDOF):
                val = 0.0
                for l in range(max(i, j), NDOF):
                    val += dJ[l, i, k] @ GJ[l, j] + J[l, i] @ GdJ[l, j, k]
                dM[i, j, k] = val
    return M, dM


def _mass_matrix_python(q: np.ndarray) -> np.ndarray:
    """Return M(q) using the pure-Python product-of-exponentials path."""
    q = _as_vec6(q, "q")

    A, Gp, xi = compute_A_and_Gp(q)
    J = np.zeros((NDOF, NDOF, 6), dtype=float)
    for l in range(NDOF):
        for j in range(l + 1):
            J[l, j] = A[l, j] @ xi[:, j]

    GJ = np.einsum("lab,ljb->lja", Gp, J)
    M = np.zeros((NDOF, NDOF), dtype=float)
    for i in range(NDOF):
        for j in range(NDOF):
            val = 0.0
            for l in range(max(i, j), NDOF):
                val += J[l, i] @ GJ[l, j]
            M[i, j] = val
    return 0.5 * (M + M.T)


def mass_matrix_analytic(q: np.ndarray) -> np.ndarray:
    """Return M(q) without computing dM/dq when the backend exposes it."""
    q = _as_vec6(q, "q")
    if _FAST_DYNAMICS is not None:
        mass_matrix = getattr(_FAST_DYNAMICS, "mass_matrix", None)
        if mass_matrix is not None:
            return mass_matrix(q)
        M, _ = _FAST_DYNAMICS.mass_and_dM(q)
        return M
    return _mass_matrix_python(q)


def mass_and_dM_finite_difference(
    q: np.ndarray,
    step: float = 1e-5,
    method: str = "central",
) -> Tuple[np.ndarray, np.ndarray]:
    """Return M(q) and finite-difference dM/dq.

    The central method costs 2*NDOF mass-matrix calls and has O(step^2)
    truncation error. The forward method costs NDOF+1 mass-matrix calls and is
    useful when raw speed matters more than accuracy.
    """
    q = _as_vec6(q, "q")
    h = float(step)
    if h <= 0.0:
        raise ValueError("finite-difference step must be positive")
    method = str(method)
    if method not in ("central", "forward"):
        raise ValueError("finite-difference method must be 'central' or 'forward'")

    M0 = mass_matrix_analytic(q)
    dM = np.zeros((NDOF, NDOF, NDOF), dtype=float)
    for k in range(NDOF):
        q_plus = q.copy()
        q_plus[k] += h
        M_plus = mass_matrix_analytic(q_plus)
        if method == "central":
            q_minus = q.copy()
            q_minus[k] -= h
            M_minus = mass_matrix_analytic(q_minus)
            dM[:, :, k] = (M_plus - M_minus) / (2.0 * h)
        else:
            dM[:, :, k] = (M_plus - M0) / h
    return M0, dM


def coriolis_from_dM(dM: np.ndarray, dq: np.ndarray) -> np.ndarray:
    dq = _as_vec6(dq, "dq")
    dM = np.asarray(dM, dtype=float)
    if dM.shape != (NDOF, NDOF, NDOF):
        raise ValueError(f"dM must have shape {(NDOF, NDOF, NDOF)}, got {dM.shape}")

    C = np.zeros((NDOF, NDOF), dtype=float)
    for i in range(NDOF):
        for j in range(NDOF):
            C[i, j] = 0.5 * sum((dM[i, j, k] + dM[i, k, j] - dM[k, j, i]) * dq[k] for k in range(NDOF))
    return C


def _link_com_transforms_and_space_jacobian(
    q: np.ndarray,
    p: Optional[Z1Params] = None,
) -> Tuple[Tuple[np.ndarray, ...], np.ndarray]:
    p = z1_parameters() if p is None else p
    q = _as_vec6(q, "q")
    xi, w_list, v_list = z1_twists(p)
    E = [twist_exp(w_list[:, i], v_list[:, i], q[i]) for i in range(NDOF)]

    g_coms = []
    g = np.eye(4)
    for l in range(NDOF):
        g = g @ E[l]
        g_coms.append(g @ p.gsl0[l])

    J_space = np.zeros((6, NDOF), dtype=float)
    T_prev = np.eye(4)
    for j in range(NDOF):
        J_space[:, j] = adjoint(T_prev) @ xi[:, j]
        T_prev = T_prev @ E[j]

    return tuple(g_coms), J_space


def link_com_positions(q: np.ndarray) -> np.ndarray:
    """Return the world-frame COM positions as a 3x6 matrix."""
    g_coms, _ = _link_com_transforms_and_space_jacobian(q)
    return np.column_stack([g[:3, 3] for g in g_coms])


def potential_energy(q: np.ndarray) -> float:
    """Return gravitational potential energy V(q)=sum_i m_i*g*z_i(q)."""
    p = z1_parameters()
    p_com = link_com_positions(q)
    return float(np.sum(p.m_vec * p.g_const * p_com[2, :]))


def gravity_vector_finite_difference(q: np.ndarray, step: float = 1e-6) -> np.ndarray:
    """Return dV/dq from a central finite difference of potential_energy(q)."""
    q = _as_vec6(q, "q")
    h = float(step)
    if h <= 0.0:
        raise ValueError("finite-difference step must be positive")

    N = np.zeros(NDOF, dtype=float)
    for j in range(NDOF):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[j] += h
        q_minus[j] -= h
        N[j] = (potential_energy(q_plus) - potential_energy(q_minus)) / (2.0 * h)
    return N


def gravity_vector_virtual_work(q: np.ndarray) -> np.ndarray:
    """Return the PDF-style gravity gradient using body Jacobians.

    The calculation uses each link COM frame, a body-frame gravity wrench, and
    the convention dV/dq = -J_body.T @ F_gravity_body.
    """
    q = _as_vec6(q, "q")
    p = z1_parameters()
    g_coms, J_space = _link_com_transforms_and_space_jacobian(q, p=p)
    gravity_force_space = np.array([0.0, 0.0, -p.g_const], dtype=float)

    N = np.zeros(NDOF, dtype=float)
    for l, g_com in enumerate(g_coms):
        J_body = adjoint_inverse(g_com) @ J_space
        force_body = g_com[:3, :3].T @ (p.m_vec[l] * gravity_force_space)
        gravity_wrench_body = np.r_[force_body, np.zeros(3)]
        N[: l + 1] += -(J_body[:, : l + 1].T @ gravity_wrench_body)
    return N


def gravity_vector(q: np.ndarray) -> np.ndarray:
    q = _as_vec6(q, "q")
    if _FAST_DYNAMICS is not None:
        return _FAST_DYNAMICS.gravity_vector(q)

    p = z1_parameters()
    xi, w_list, v_list = z1_twists(p)
    E = [twist_exp(w_list[:, i], v_list[:, i], q[i]) for i in range(NDOF)]

    p_com = np.zeros((3, NDOF), dtype=float)
    for l in range(NDOF):
        g = np.eye(4)
        for r in range(l + 1):
            g = g @ E[r]
        g_com = g @ p.gsl0[l]
        p_com[:, l] = g_com[:3, 3]

    J_space = np.zeros((6, NDOF), dtype=float)
    T_prev = np.eye(4)
    for j in range(NDOF):
        J_space[:, j] = adjoint(T_prev) @ xi[:, j]
        T_prev = T_prev @ E[j]

    N = np.zeros(NDOF, dtype=float)
    for j in range(NDOF):
        vj = J_space[:3, j]
        wj = J_space[3:6, j]
        for l in range(j, NDOF):
            dp = vj + np.cross(wj, p_com[:, l])
            N[j] += p.m_vec[l] * p.g_const * dp[2]
    return N


def dynamics_analytic(q: np.ndarray, dq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    q = _as_vec6(q, "q")
    dq = _as_vec6(dq, "dq")
    if _FAST_DYNAMICS is not None:
        return _FAST_DYNAMICS.dynamics(q, dq)

    M, dM = mass_and_dM_analytic(q)
    C = coriolis_from_dM(dM, dq)
    N = gravity_vector(q)
    return M, C, N, dM


def dynamics_finite_difference(
    q: np.ndarray,
    dq: np.ndarray,
    step: float = 1e-5,
    method: str = "central",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return dynamics with dM/dq approximated by finite differences of M(q)."""
    q = _as_vec6(q, "q")
    dq = _as_vec6(dq, "dq")
    M, dM = mass_and_dM_finite_difference(q, step=step, method=method)
    C = coriolis_from_dM(dM, dq)
    N = gravity_vector(q)
    return M, C, N, dM
