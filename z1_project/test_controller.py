#!/usr/bin/env python3
"""Diagnostic torque controllers for Gazebo/debug testing only.

The final project controller remains computed torque in controller.py.
Use this file only to compare simple control behavior against the main
computed-torque path.
"""

from __future__ import annotations

import numpy as np

from z1_analytic_dynamics import NDOF, dynamics_analytic, dynamics_finite_difference, gravity_vector

DEFAULT_TEST_KP = np.array([20.0, 20.0, 60.0, 20.0, 5.0, 5.0], dtype=float)
DEFAULT_TEST_KD = np.array([5.0, 5.0, 15.0, 5.0, 1.0, 1.0], dtype=float)
DEFAULT_TEST_KI = np.zeros(NDOF, dtype=float)
GAZEBO_MODEL_DAMPING = np.array([1.0, 2.0, 1.0, 1.0, 1.0, 1.0], dtype=float)
GAZEBO_MODEL_FRICTION = np.array([1.0, 2.0, 1.0, 1.0, 1.0, 1.0], dtype=float)
FRICTION_VELOCITY_EPS = 1e-4
DEFAULT_FRICTION_DEADBAND = 0.002
TEST_CONTROLLER_MODES = (
    "gravity_only",
    "pd_only",
    "pd_gravity",
    "augmented_pd",
    "augmented_pd_friction_model",
    "augmented_pid_friction_model",
    "computed_pid_friction_model",
    "gazebo_friction_model",
    "feedforward_friction_model",
)
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


def _resolve_test_gains(kp=None, kd=None) -> tuple[np.ndarray, np.ndarray]:
    kp_values = DEFAULT_TEST_KP if kp is None else kp
    kd_values = DEFAULT_TEST_KD if kd is None else kd
    return _gain_matrix(kp_values, "kp"), _gain_matrix(kd_values, "kd")


def _resolve_pid_gains(kp=None, kd=None, ki=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    Kp, Kd = _resolve_test_gains(kp=kp, kd=kd)
    ki_values = DEFAULT_TEST_KI if ki is None else ki
    return Kp, Kd, _gain_matrix(ki_values, "ki")


def _dynamics_for_mode(
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


def compute_test_tau(
    q,
    dq,
    q_des,
    dq_des,
    mode="pd_gravity",
    kp=None,
    kd=None,
    ddq_des=None,
    ki=None,
    e_int=None,
    model_damping=None,
    model_friction=None,
    friction_deadband: float = DEFAULT_FRICTION_DEADBAND,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> np.ndarray:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    mode = str(mode)

    if mode not in TEST_CONTROLLER_MODES:
        raise ValueError(f"unknown test controller mode: {mode}")

    if mode == "augmented_pd":
        _, _, tau = compute_augmented_pd_components(
            q,
            dq,
            q_des,
            dq_des,
            ddq_des,
            kp=kp,
            kd=kd,
            dynamics_mode=dynamics_mode,
            finite_diff_step=finite_diff_step,
            finite_diff_method=finite_diff_method,
        )
        return tau

    if mode == "augmented_pd_friction_model":
        return compute_augmented_pd_friction_model_tau(
            q,
            dq,
            q_des,
            dq_des,
            ddq_des,
            kp=kp,
            kd=kd,
            model_damping=model_damping,
            model_friction=model_friction,
            friction_deadband=friction_deadband,
            dynamics_mode=dynamics_mode,
            finite_diff_step=finite_diff_step,
            finite_diff_method=finite_diff_method,
        )

    if mode == "augmented_pid_friction_model":
        _, _, _, tau = compute_augmented_pid_friction_model_components(
            q,
            dq,
            q_des,
            dq_des,
            ddq_des,
            e_int=e_int,
            kp=kp,
            kd=kd,
            ki=ki,
            model_damping=model_damping,
            model_friction=model_friction,
            friction_deadband=friction_deadband,
            dynamics_mode=dynamics_mode,
            finite_diff_step=finite_diff_step,
            finite_diff_method=finite_diff_method,
        )
        return tau

    if mode == "computed_pid_friction_model":
        _, tau = compute_computed_pid_friction_model_components(
            q,
            dq,
            q_des,
            dq_des,
            ddq_des,
            e_int=e_int,
            kp=kp,
            kd=kd,
            ki=ki,
            model_damping=model_damping,
            model_friction=model_friction,
            friction_deadband=friction_deadband,
            dynamics_mode=dynamics_mode,
            finite_diff_step=finite_diff_step,
            finite_diff_method=finite_diff_method,
        )
        return tau

    if mode == "gazebo_friction_model":
        return compute_gazebo_friction_model_tau(
            q,
            dq,
            q_des,
            dq_des,
            ddq_des,
            kp=kp,
            kd=kd,
            model_damping=model_damping,
            model_friction=model_friction,
            friction_deadband=friction_deadband,
            dynamics_mode=dynamics_mode,
            finite_diff_step=finite_diff_step,
            finite_diff_method=finite_diff_method,
        )

    if mode == "feedforward_friction_model":
        return compute_feedforward_friction_model_tau(
            q,
            dq,
            q_des,
            dq_des,
            ddq_des,
            model_damping=model_damping,
            model_friction=model_friction,
            friction_deadband=friction_deadband,
            dynamics_mode=dynamics_mode,
            finite_diff_step=finite_diff_step,
            finite_diff_method=finite_diff_method,
        )

    gravity_tau = gravity_vector(q)
    if mode == "gravity_only":
        return gravity_tau

    Kp, Kd = _resolve_test_gains(kp=kp, kd=kd)
    tau_pd = Kp @ (q_des - q) + Kd @ (dq_des - dq)
    if mode == "pd_only":
        return tau_pd
    return tau_pd + gravity_tau


def compute_augmented_pd_components(
    q,
    dq,
    q_des,
    dq_des,
    ddq_des,
    kp=None,
    kd=None,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    ddq_des = _vec6(ddq_des, "ddq_des")
    Kp, Kd = _resolve_test_gains(kp=kp, kd=kd)

    # Computed torque puts feedback inside M(q). Augmented PD keeps model
    # feedforward separate and applies PD feedback directly as joint torque.
    M, C, N, _ = _dynamics_for_mode(
        q,
        dq,
        mode=dynamics_mode,
        finite_diff_step=finite_diff_step,
        finite_diff_method=finite_diff_method,
    )
    tau_ff = M @ ddq_des + C @ dq_des + N
    tau_fb = Kp @ (q_des - q) + Kd @ (dq_des - dq)
    tau = tau_ff + tau_fb
    return tau_ff, tau_fb, tau


def _gazebo_friction_terms(
    e: np.ndarray,
    dq_des: np.ndarray,
    model_damping=None,
    model_friction=None,
    friction_deadband: float = DEFAULT_FRICTION_DEADBAND,
) -> tuple[np.ndarray, np.ndarray]:
    e = _vec6(e, "e")
    dq_des = _vec6(dq_des, "dq_des")
    damping_values = GAZEBO_MODEL_DAMPING if model_damping is None else _vec6(model_damping, "model_damping")
    friction_values = GAZEBO_MODEL_FRICTION if model_friction is None else _vec6(model_friction, "model_friction")

    tau_damping = damping_values * dq_des
    velocity_direction = np.sign(dq_des)
    error_direction = np.sign(e)
    direction = np.where(np.abs(dq_des) > FRICTION_VELOCITY_EPS, velocity_direction, error_direction)
    wants_motion = (np.abs(e) > float(friction_deadband)) | (np.abs(dq_des) > FRICTION_VELOCITY_EPS)
    tau_friction = friction_values * direction * wants_motion
    return tau_damping, tau_friction


def compute_augmented_pd_friction_model_tau(
    q,
    dq,
    q_des,
    dq_des,
    ddq_des,
    kp=None,
    kd=None,
    model_damping=None,
    model_friction=None,
    friction_deadband: float = DEFAULT_FRICTION_DEADBAND,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> np.ndarray:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    if ddq_des is None:
        raise ValueError("ddq_des is required for augmented_pd_friction_model test controller")
    ddq_des = _vec6(ddq_des, "ddq_des")
    Kp, Kd = _resolve_test_gains(kp=kp, kd=kd)

    # Augmented PD keeps the model feedforward separate from direct joint-torque
    # feedback. This variant adds the Gazebo damping/friction compensation terms.
    M, C, N, _ = _dynamics_for_mode(
        q,
        dq,
        mode=dynamics_mode,
        finite_diff_step=finite_diff_step,
        finite_diff_method=finite_diff_method,
    )
    e = q_des - q
    tau_ff = M @ ddq_des + C @ dq_des + N
    tau_fb = Kp @ e + Kd @ (dq_des - dq)
    tau_damping, tau_friction = _gazebo_friction_terms(
        e,
        dq_des,
        model_damping=model_damping,
        model_friction=model_friction,
        friction_deadband=friction_deadband,
    )
    return tau_ff + tau_fb + tau_damping + tau_friction


def compute_augmented_pid_friction_model_components(
    q,
    dq,
    q_des,
    dq_des,
    ddq_des,
    e_int=None,
    kp=None,
    kd=None,
    ki=None,
    model_damping=None,
    model_friction=None,
    friction_deadband: float = DEFAULT_FRICTION_DEADBAND,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    if ddq_des is None:
        raise ValueError("ddq_des is required for augmented_pid_friction_model test controller")
    ddq_des = _vec6(ddq_des, "ddq_des")
    e_int = np.zeros(NDOF, dtype=float) if e_int is None else _vec6(e_int, "e_int")
    Kp, Kd, Ki = _resolve_pid_gains(kp=kp, kd=kd, ki=ki)

    M, C, N, _ = _dynamics_for_mode(
        q,
        dq,
        mode=dynamics_mode,
        finite_diff_step=finite_diff_step,
        finite_diff_method=finite_diff_method,
    )
    e = q_des - q
    de = dq_des - dq
    tau_ff = M @ ddq_des + C @ dq_des + N
    tau_fb = Kp @ e + Kd @ de
    tau_i = Ki @ e_int
    tau_damping, tau_friction = _gazebo_friction_terms(
        e,
        dq_des,
        model_damping=model_damping,
        model_friction=model_friction,
        friction_deadband=friction_deadband,
    )
    tau = tau_ff + tau_fb + tau_i + tau_damping + tau_friction
    return tau_ff, tau_fb, tau_i, tau


def compute_computed_pid_friction_model_components(
    q,
    dq,
    q_des,
    dq_des,
    ddq_des,
    e_int=None,
    kp=None,
    kd=None,
    ki=None,
    model_damping=None,
    model_friction=None,
    friction_deadband: float = DEFAULT_FRICTION_DEADBAND,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> tuple[np.ndarray, np.ndarray]:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    if ddq_des is None:
        raise ValueError("ddq_des is required for computed_pid_friction_model test controller")
    ddq_des = _vec6(ddq_des, "ddq_des")
    e_int = np.zeros(NDOF, dtype=float) if e_int is None else _vec6(e_int, "e_int")
    Kp, Kd, Ki = _resolve_pid_gains(kp=kp, kd=kd, ki=ki)

    e = q_des - q
    de = dq_des - dq
    i_accel = Ki @ e_int
    ddq_cmd = ddq_des + Kd @ de + Kp @ e + i_accel
    M, C, N, _ = _dynamics_for_mode(
        q,
        dq,
        mode=dynamics_mode,
        finite_diff_step=finite_diff_step,
        finite_diff_method=finite_diff_method,
    )
    tau_model = M @ ddq_cmd + C @ dq + N
    tau_i = M @ i_accel
    tau_damping, tau_friction = _gazebo_friction_terms(
        e,
        dq_des,
        model_damping=model_damping,
        model_friction=model_friction,
        friction_deadband=friction_deadband,
    )
    tau = tau_model + tau_damping + tau_friction
    return tau_i, tau


def compute_gazebo_friction_model_tau(
    q,
    dq,
    q_des,
    dq_des,
    ddq_des,
    kp=None,
    kd=None,
    model_damping=None,
    model_friction=None,
    friction_deadband: float = DEFAULT_FRICTION_DEADBAND,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> np.ndarray:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    if ddq_des is None:
        raise ValueError("ddq_des is required for gazebo_friction_model test controller")
    ddq_des = _vec6(ddq_des, "ddq_des")
    Kp, Kd = _resolve_test_gains(kp=kp, kd=kd)

    # computed_torque:
    #   tau = M(q)(qdd_des + Kd de + Kp e) + C dq + N
    # gazebo_friction_model:
    #   same model torque, plus Gazebo URDF/Xacro damping/friction compensation:
    #   tau += D dq_des + F direction
    # These D/F constants come from the Gazebo Z1 URDF/Xacro, not real robot
    # friction identification.
    e = q_des - q
    de = dq_des - dq
    ddq_cmd = ddq_des + Kd @ de + Kp @ e
    M, C, N, _ = _dynamics_for_mode(
        q,
        dq,
        mode=dynamics_mode,
        finite_diff_step=finite_diff_step,
        finite_diff_method=finite_diff_method,
    )
    tau_model = M @ ddq_cmd + C @ dq + N
    tau_damping, tau_friction = _gazebo_friction_terms(
        e,
        dq_des,
        model_damping=model_damping,
        model_friction=model_friction,
        friction_deadband=friction_deadband,
    )
    return tau_model + tau_damping + tau_friction


def compute_feedforward_friction_model_tau(
    q,
    dq,
    q_des,
    dq_des,
    ddq_des,
    model_damping=None,
    model_friction=None,
    friction_deadband: float = DEFAULT_FRICTION_DEADBAND,
    dynamics_mode: str = "analytic",
    finite_diff_step: float = 1e-5,
    finite_diff_method: str = "central",
) -> np.ndarray:
    q = _vec6(q, "q")
    dq = _vec6(dq, "dq")
    q_des = _vec6(q_des, "q_des")
    dq_des = _vec6(dq_des, "dq_des")
    if ddq_des is None:
        raise ValueError("ddq_des is required for feedforward_friction_model test controller")
    ddq_des = _vec6(ddq_des, "ddq_des")
    M, C, N, _ = _dynamics_for_mode(
        q,
        dq,
        mode=dynamics_mode,
        finite_diff_step=finite_diff_step,
        finite_diff_method=finite_diff_method,
    )
    tau_model = M @ ddq_des + C @ dq + N

    e = q_des - q
    tau_damping, tau_friction = _gazebo_friction_terms(
        e,
        dq_des,
        model_damping=model_damping,
        model_friction=model_friction,
        friction_deadband=friction_deadband,
    )
    return tau_model + tau_damping + tau_friction
