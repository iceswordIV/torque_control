#!/usr/bin/env python3
r"""
Unitree Z1-style dynamics demo using analytic dM/dq from the Lie-bracket formula.

No symbolic math.
No finite difference for dM.

Run:
    python z1_analytic_dm_python.py --check
    python z1_analytic_dm_python.py --simulate
    python z1_analytic_dm_python.py --simulate --no-plot

This script calculates:
    M(q)
    dM/dq analytically using dJ/dq = A_lk [A_kj xi_j, xi_k]
    C(q,dq) from dM/dq
    N(q) from COM Jacobians
    qdd = M \ (tau - C dq - N - B dq)

Twist convention used here is xi = [v; w].
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from typing import Tuple, Dict, Any

import numpy as np

NDOF = 6


def skew3(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a).reshape(3)
    return np.array([
        [0.0, -a[2], a[1]],
        [a[2], 0.0, -a[0]],
        [-a[1], a[0], 0.0],
    ], dtype=np.result_type(a, float))


def inertia_from_urdf(ixx, ixy, ixz, iyy, iyz, izz) -> np.ndarray:
    return np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ], dtype=float)


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
    """Adjoint for twist convention xi=[v;w], matching Ad_g.m."""
    dtype = np.result_type(g, float)
    R = g[:3, :3]
    p = g[:3, 3]
    Ad = np.zeros((6, 6), dtype=dtype)
    Ad[:3, :3] = R
    Ad[:3, 3:6] = skew3(p) @ R
    Ad[3:6, 3:6] = R
    return Ad


def adjoint_inverse(g: np.ndarray) -> np.ndarray:
    """Inverse adjoint for twist convention xi=[v;w], matching Ad_g_inv.m."""
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
    xi1 = np.asarray(xi1)
    xi2 = np.asarray(xi2)
    dtype = np.result_type(xi1, xi2, float)
    v1, w1 = xi1[:3].astype(dtype), xi1[3:6].astype(dtype)
    v2, w2 = xi2[:3].astype(dtype), xi2[3:6].astype(dtype)
    return np.r_[np.cross(w1, v2) + np.cross(v1, w2), np.cross(w1, w2)].astype(dtype)


@dataclass
class Z1Params:
    g_const: float
    m_vec: np.ndarray
    I: Tuple[np.ndarray, ...]
    w: np.ndarray       # 3x6
    q_axis: np.ndarray  # 3x6 joint-axis points at home
    c: np.ndarray       # 3x6 COM offsets relative to joint home frame points
    gsl0: Tuple[np.ndarray, ...]


def z1_parameters() -> Z1Params:
    g_const = 9.80665
    m_vec = np.array([0.67332551, 1.19132258, 0.83940874, 0.56404563, 0.38938492, 0.0], dtype=float)

    I1 = inertia_from_urdf(0.00128328, -6e-08,  -4e-07, 0.00071931,  5e-07, 0.00083936)
    I2 = inertia_from_urdf(0.00102138,  0.00062358, 5.13e-06, 0.02429457, -2.1e-06, 0.02466114)
    I3 = inertia_from_urdf(0.00108061, -8.669e-05, -0.00208102, 0.00954238, -1.332e-05, 0.00886621)
    I4 = inertia_from_urdf(0.00031576,  8.13e-05,  4.091e-05, 0.00092996, -5.96e-06, 0.00097912)
    I5 = inertia_from_urdf(0.00017605,  4e-07,    5.689e-05, 0.00055896, -1.3e-07, 0.00053860)

    m_link06 = 0.28875807
    c_link06 = np.array([0.0241569, -0.00017355, -0.00143876])
    I_link06 = inertia_from_urdf(0.00018328, 1.22e-06, 5.4e-07, 0.0001475, 8e-08, 0.0001468)

    m_gripper_stator = 0.52603655
    c_gripper_stator = np.array([0.04764427, -0.00035819, -0.00249162])
    I_gripper_stator = inertia_from_urdf(0.00038683, -0.00000359, 0.00007662, 0.00068614, 0.00000209, 0.00066293)

    m_gripper_mover = 0.27621302
    c_gripper_mover = np.array([0.01320633, 0.00476708, 0.00380534])
    I_gripper_mover = inertia_from_urdf(0.00017716, 0.00001683, -0.00001786, 0.00026787, 0.00000262, 0.00035728)

    gripper_joint_origin = np.array([0.051, 0.0, 0.0])
    m_total = m_link06 + m_gripper_stator + m_gripper_mover
    c_gripper_stator_in_link06 = gripper_joint_origin + c_gripper_stator
    c_gripper_mover_in_link06 = gripper_joint_origin + c_gripper_mover
    c_combined = (
        m_link06 * c_link06
        + m_gripper_stator * c_gripper_stator_in_link06
        + m_gripper_mover * c_gripper_mover_in_link06
    ) / m_total

    I_link06_at_combined = parallel_axis_theorem(I_link06, m_link06, c_link06 - c_combined)
    I_gripper_stator_at_combined = parallel_axis_theorem(I_gripper_stator, m_gripper_stator, c_gripper_stator_in_link06 - c_combined)
    I_gripper_mover_at_combined = parallel_axis_theorem(I_gripper_mover, m_gripper_mover, c_gripper_mover_in_link06 - c_combined)
    I6 = I_link06_at_combined + I_gripper_stator_at_combined + I_gripper_mover_at_combined
    m_vec[5] = m_total

    w = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 1.0, 1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    ], dtype=float)

    q_axis = np.array([
        [0.0,     0.0,   -0.35,  -0.132, -0.062, -0.0128],
        [0.0,     0.0,    0.0,    0.0,    0.0,    0.0],
        [0.0585,  0.1035, 0.1035, 0.1605, 0.1605, 0.1605],
    ], dtype=float)

    c = np.array([
        [ 2.47e-06,   -0.11012601,  0.10609208,  0.04366681, 0.03121533, c_combined[0]],
        [-0.00025198,  0.00240029, -0.00541815,  0.00364738, 0.0,        c_combined[1]],
        [ 0.02317169,  0.00158266,  0.03476383, -0.00170192, 0.00646316, c_combined[2]],
    ], dtype=float)

    I = (I1, I2, I3, I4, I5, I6)
    gsl0 = tuple(make_g(np.eye(3), q_axis[:, i] + c[:, i]) for i in range(NDOF))
    return Z1Params(g_const=g_const, m_vec=m_vec, I=I, w=w, q_axis=q_axis, c=c, gsl0=gsl0)


def z1_twists(p: Z1Params) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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


def compute_A_and_Gp(q: np.ndarray, p: Z1Params) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = np.asarray(q).reshape(NDOF)
    xi, w_list, v_list = z1_twists(p)
    E = [twist_exp(w_list[:, i], v_list[:, i], q[i]) for i in range(NDOF)]

    A = np.zeros((NDOF, NDOF, 6, 6), dtype=float)  # A[l,j]
    for l in range(NDOF):
        for j in range(NDOF):
            if l < j:
                continue
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
    """d/dq_k of J_lj = A_lj xi_j.  Indices are 0-based."""
    if not (l >= k >= j):
        return np.zeros(6)
    return A[l, k] @ lie_bracket_vw(A[k, j] @ xi[:, j], xi[:, k])


def mass_and_dM_analytic(q: np.ndarray, p: Z1Params) -> Tuple[np.ndarray, np.ndarray]:
    A, Gp, xi = compute_A_and_Gp(q, p)

    # J[l,j] = A_lj xi_j
    J = np.zeros((NDOF, NDOF, 6), dtype=float)
    for l in range(NDOF):
        for j in range(l + 1):
            J[l, j] = A[l, j] @ xi[:, j]

    GJ = np.einsum("lab,ljb->lja", Gp, J)  # Gp[l] @ J[l,j]

    M = np.zeros((NDOF, NDOF), dtype=float)
    for i in range(NDOF):
        for j in range(NDOF):
            val = 0.0
            for l in range(max(i, j), NDOF):
                val += J[l, i] @ GJ[l, j]
            M[i, j] = val
    M = 0.5 * (M + M.T)

    # Precompute dJ[l,j,k]
    dJ = np.zeros((NDOF, NDOF, NDOF, 6), dtype=float)
    for l in range(NDOF):
        for j in range(l + 1):
            for k in range(j, l + 1):
                dJ[l, j, k] = dJ_column_analytic(l, j, k, A, xi)

    GdJ = np.einsum("lab,ljkb->ljka", Gp, dJ)  # Gp[l] @ dJ[l,j,k]

    dM = np.zeros((NDOF, NDOF, NDOF), dtype=float)
    for k in range(NDOF):
        for i in range(NDOF):
            for j in range(NDOF):
                val = 0.0
                for l in range(max(i, j), NDOF):
                    val += dJ[l, i, k] @ GJ[l, j] + J[l, i] @ GdJ[l, j, k]
                dM[i, j, k] = val
    return M, dM


def mass_matrix_analytic(q: np.ndarray, p: Z1Params) -> np.ndarray:
    M, _ = mass_and_dM_analytic(q, p)
    return M


def coriolis_from_dM(dM: np.ndarray, dq: np.ndarray) -> np.ndarray:
    dq = np.asarray(dq).reshape(NDOF)
    C = np.zeros((NDOF, NDOF), dtype=float)
    for i in range(NDOF):
        for j in range(NDOF):
            # sum_k 0.5*(dM_ij/dqk + dM_ik/dqj - dM_kj/dqi)*dqk
            C[i, j] = 0.5 * sum((dM[i, j, k] + dM[i, k, j] - dM[k, j, i]) * dq[k] for k in range(NDOF))
    return C


def gravity_vector_analytic(q: np.ndarray, p: Z1Params) -> np.ndarray:
    q = np.asarray(q).reshape(NDOF)
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


def dynamics_analytic(q: np.ndarray, dq: np.ndarray, p: Z1Params) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    M, dM = mass_and_dM_analytic(q, p)
    C = coriolis_from_dM(dM, dq)
    N = gravity_vector_analytic(q, p)
    return M, C, N, dM


def desired_trajectory(
    t: float,
    q_start: np.ndarray | None = None,
    joint: int = 1,
    angle_deg: float = 5.0,
    move_start: float = 0.5,
    move_duration: float = 5.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """One-joint quintic test trajectory.

    joint is 1-based: joint=1 moves q1, joint=6 moves q6.
    angle_deg is relative to q_start, not absolute.

    Timeline:
        0 ... move_start: hold q_start
        move_start ... move_start+move_duration: smooth quintic move
        after move: hold q_goal
    """
    if q_start is None:
        q_start = np.zeros(NDOF, dtype=float)
    else:
        q_start = np.asarray(q_start, dtype=float).reshape(NDOF)

    if not 1 <= int(joint) <= NDOF:
        raise ValueError(f"joint must be in 1..{NDOF}, got {joint}")

    q_goal = q_start.copy()
    q_goal[int(joint) - 1] += math.radians(float(angle_deg))

    elapsed = float(t) - float(move_start)
    if elapsed <= 0.0:
        return q_start.copy(), np.zeros(NDOF), np.zeros(NDOF)

    T = max(float(move_duration), 1e-9)
    if elapsed >= T:
        return q_goal.copy(), np.zeros(NDOF), np.zeros(NDOF)

    s = elapsed / T
    b = 10*s**3 - 15*s**4 + 6*s**5
    bd = (30*s**2 - 60*s**3 + 30*s**4) / T
    bdd = (60*s - 180*s**2 + 120*s**3) / (T*T)

    delta = q_goal - q_start
    return q_start + b * delta, bd * delta, bdd * delta


def control_logic(
    t: float,
    q: np.ndarray,
    dq: np.ndarray,
    p: Z1Params,
    joint: int = 1,
    angle_deg: float = 5.0,
    move_start: float = 0.5,
    move_duration: float = 5.0,
) -> np.ndarray:
    qd, dqd, ddqd = desired_trajectory(
        t,
        q_start=np.zeros(NDOF),
        joint=joint,
        angle_deg=angle_deg,
        move_start=move_start,
        move_duration=move_duration,
    )
    Kp = np.diag([80, 80, 70, 55, 35, 25])
    Kd = np.diag([16, 16, 14, 11, 7, 5])
    e = qd - q
    de = dqd - dq

    # Computed-torque style: tau = M*(ddq_des + Kd*de + Kp*e) + C*dq + N
    M, C, N, _ = dynamics_analytic(q, dq, p)
    tau = M @ (ddqd + Kd @ de + Kp @ e) + C @ dq + N
    return tau


def forward_dynamics(q: np.ndarray, dq: np.ndarray, tau: np.ndarray, p: Z1Params, damping: np.ndarray) -> np.ndarray:
    M, C, N, _ = dynamics_analytic(q, dq, p)
    return np.linalg.solve(M, tau - C @ dq - N - damping @ dq)


def simulate(
    t_end: float = 7.0,
    dt: float = 0.01,
    csv_path: str = "z1_python_sim_log.csv",
    joint: int = 1,
    angle_deg: float = 5.0,
    move_start: float = 0.5,
    move_duration: float = 5.0,
) -> Dict[str, np.ndarray]:
    p = z1_parameters()
    damping = np.diag([0.02, 0.02, 0.02, 0.01, 0.01, 0.005])
    steps = int(round(t_end / dt))
    q = np.zeros(NDOF)
    dq = np.zeros(NDOF)

    t_log = np.zeros(steps + 1)
    q_log = np.zeros((steps + 1, NDOF))
    dq_log = np.zeros((steps + 1, NDOF))
    tau_log = np.zeros((steps + 1, NDOF))
    qd_log = np.zeros((steps + 1, NDOF))
    dqd_log = np.zeros((steps + 1, NDOF))
    ddqd_log = np.zeros((steps + 1, NDOF))

    qd0, dqd0, ddqd0 = desired_trajectory(
        0.0, q_start=np.zeros(NDOF), joint=joint, angle_deg=angle_deg,
        move_start=move_start, move_duration=move_duration,
    )
    q_log[0] = q
    dq_log[0] = dq
    qd_log[0] = qd0
    dqd_log[0] = dqd0
    ddqd_log[0] = ddqd0

    start = time.perf_counter()
    for k in range(steps):
        t = k * dt

        # Calculate dynamics ONCE per controller step.
        # This is faster than calling control_logic() and forward_dynamics() separately.
        M, C, N, _ = dynamics_analytic(q, dq, p)
        qd, dqd, ddqd = desired_trajectory(
            t,
            q_start=np.zeros(NDOF),
            joint=joint,
            angle_deg=angle_deg,
            move_start=move_start,
            move_duration=move_duration,
        )
        Kp = np.diag([80, 80, 70, 55, 35, 25])
        Kd = np.diag([16, 16, 14, 11, 7, 5])
        tau = M @ (ddqd + Kd @ (dqd - dq) + Kp @ (qd - q)) + C @ dq + N
        qdd = np.linalg.solve(M, tau - C @ dq - N - damping @ dq)

        # Semi-implicit Euler. Fast and stable enough for this learning demo.
        dq = dq + qdd * dt
        q = q + dq * dt

        t_log[k + 1] = t + dt
        q_log[k + 1] = q
        dq_log[k + 1] = dq
        tau_log[k + 1] = tau
        qd_log[k + 1] = qd
        dqd_log[k + 1] = dqd
        ddqd_log[k + 1] = ddqd
    elapsed = time.perf_counter() - start

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["t"]
            + [f"q_des{i+1}" for i in range(NDOF)]
            + [f"dq_des{i+1}" for i in range(NDOF)]
            + [f"ddq_des{i+1}" for i in range(NDOF)]
            + [f"q{i+1}" for i in range(NDOF)]
            + [f"dq{i+1}" for i in range(NDOF)]
            + [f"tau{i+1}" for i in range(NDOF)]
        )
        for i in range(steps + 1):
            writer.writerow([t_log[i], *qd_log[i], *dqd_log[i], *ddqd_log[i], *q_log[i], *dq_log[i], *tau_log[i]])

    print(f"Trajectory: joint {joint}, angle {angle_deg:.3f} deg, move_start {move_start:.3f} s, move_duration {move_duration:.3f} s")
    print("Max |q_des| by joint [deg] =", np.rad2deg(np.max(np.abs(qd_log), axis=0)))
    print("Max |dq_des| by joint [rad/s] =", np.max(np.abs(dqd_log), axis=0))
    print("Max |ddq_des| by joint [rad/s^2] =", np.max(np.abs(ddqd_log), axis=0))
    print("Max |tau| by joint [Nm] =", np.max(np.abs(tau_log), axis=0))
    print(f"Simulation finished: {steps} steps in {elapsed:.2f} s")
    print(f"CSV log written: {csv_path}")
    return {"t": t_log, "q": q_log, "dq": dq_log, "tau": tau_log, "q_des": qd_log, "dq_des": dqd_log, "ddq_des": ddqd_log}


def finite_difference_dM(q: np.ndarray, p: Z1Params, eps: float = 1e-6) -> np.ndarray:
    q = np.asarray(q).reshape(NDOF)
    dM = np.zeros((NDOF, NDOF, NDOF), dtype=float)
    for k in range(NDOF):
        e = np.zeros(NDOF)
        e[k] = eps
        Mp = mass_matrix_analytic(q + e, p)
        Mm = mass_matrix_analytic(q - e, p)
        dM[:, :, k] = (Mp - Mm) / (2 * eps)
    return dM


def check_against_finite_difference() -> None:
    p = z1_parameters()
    q = np.array([0.20, 0.25, -0.25, 0.15, 0.10, 0.10])
    dq = np.array([0.50, -0.30, 0.20, -0.10, 0.40, -0.20])
    M, dM = mass_and_dM_analytic(q, p)
    C = coriolis_from_dM(dM, dq)
    N = gravity_vector_analytic(q, p)
    dM_fd = finite_difference_dM(q, p, eps=1e-6)
    C_fd = coriolis_from_dM(dM_fd, dq)

    print("Check analytic dM against finite-difference dM at one point")
    print("q  =", q)
    print("dq =", dq)
    print("M condition number =", np.linalg.cond(M))
    print("max abs dM error =", np.max(np.abs(dM - dM_fd)))
    print("fro dM error     =", np.linalg.norm((dM - dM_fd).reshape(-1)))
    print("C*dq analytic    =", C @ dq)
    print("C*dq finite diff =", C_fd @ dq)
    print("C*dq error norm  =", np.linalg.norm(C @ dq - C_fd @ dq))
    print("N(q)             =", N)


def plot_logs(log: Dict[str, np.ndarray]) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib is not available: {exc}")
        return
    t, q, dq, tau = log["t"], log["q"], log["dq"], log["tau"]
    q_des, dq_des = log["q_des"], log["dq_des"]

    plt.figure()
    plt.plot(t, q_des, linestyle="--")
    plt.plot(t, q)
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("q [rad]")
    plt.title("Z1 Python analytic dM: desired vs simulated joint position")
    plt.legend([f"q_des{i+1}" for i in range(NDOF)] + [f"q{i+1}" for i in range(NDOF)])

    plt.figure()
    plt.plot(t, dq_des, linestyle="--")
    plt.plot(t, dq)
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("dq [rad/s]")
    plt.title("Z1 Python analytic dM: desired vs simulated joint velocity")
    plt.legend([f"dq_des{i+1}" for i in range(NDOF)] + [f"dq{i+1}" for i in range(NDOF)])

    plt.figure()
    plt.plot(t, tau)
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("tau [N m]")
    plt.title("Z1 Python analytic dM: commanded torque")
    plt.legend([f"tau{i+1}" for i in range(NDOF)])
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Unitree Z1 analytic dM dynamics demo in Python")
    parser.add_argument("--check", action="store_true", help="check analytic dM against finite difference")
    parser.add_argument("--simulate", action="store_true", help="run the torque-to-q simulation")
    parser.add_argument("--tend", type=float, default=7.0, help="simulation duration [s]")
    parser.add_argument("--dt", type=float, default=0.01, help="controller/integration step [s]")
    parser.add_argument("--csv", type=str, default="z1_python_sim_log.csv", help="CSV output path")
    parser.add_argument("--no-plot", action="store_true", help="do not show matplotlib plots")
    parser.add_argument("--joint", type=int, default=1, help="1-based joint index to move, from 1 to 6")
    parser.add_argument("--angle-deg", type=float, default=5.0, help="relative target angle for that joint [deg]")
    parser.add_argument("--move-start", type=float, default=0.5, help="hold time before motion starts [s]")
    parser.add_argument("--move-duration", type=float, default=5.0, help="quintic move duration [s]")
    args = parser.parse_args()

    if not args.check and not args.simulate:
        args.check = True
        args.simulate = True

    if args.check:
        check_against_finite_difference()
    if args.simulate:
        log = simulate(
            t_end=args.tend,
            dt=args.dt,
            csv_path=args.csv,
            joint=args.joint,
            angle_deg=args.angle_deg,
            move_start=args.move_start,
            move_duration=args.move_duration,
        )
        if not args.no_plot:
            plot_logs(log)


if __name__ == "__main__":
    main()
