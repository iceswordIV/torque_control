#!/usr/bin/env python3
"""Real robot/Gazebo computed-torque runtime loop.

This script never integrates q or dq. The actual q and dq come from robot_io.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from pathlib import Path

import numpy as np

from controller import DEFAULT_WN, DEFAULT_ZETA, DYNAMICS_MODES, compute_tau
from robot_io import FileRobotIO, default_runtime_dir
from trajectory import NDOF, build_goal, parse_vec6, quintic_trajectory, scurve_trajectory

DEFAULT_TARGET = "0 1.5 -1.0 -0.54 0 0"
# Unitree z1_controller config/savedArmStates.csv uses this as "startFlat".
DEFAULT_HOME_TARGET = "0 0 -0.005 -0.074 0 0"
DEFAULT_MODEL_DAMPING = "1 2 1 1 1 1"
DEFAULT_MODEL_FRICTION = "1 2 1 1 1 1"
DEFAULT_RETURN_CONTROLLER = "augmented_pd_friction_model"
DEFAULT_RETURN_KP = "20 20 40 8 5 5"
DEFAULT_RETURN_KD = "3 3 6 1 0.6 0.4"
DEFAULT_KI = "0 0 0 0 0 0"
DEFAULT_INTEGRAL_LIMIT = "0.8 0.8 0.8 0.8 0.8 0.8"
PID_CONTROLLER_MODES = {"augmented_pid_friction_model", "computed_pid_model", "computed_pid_friction_model"}
TEST_CONTROLLER_CHOICES = [
    "none",
    "gravity_only",
    "pd_only",
    "pd_gravity",
    "augmented_pd",
    "augmented_pd_friction_model",
    "augmented_pid_friction_model",
    "computed_pid_model",
    "computed_pid_friction_model",
    "gazebo_friction_model",
    "feedforward_friction_model",
]


def parse_gain_text(text):
    if text is None:
        return None
    values = parse_vec_any(text)
    if values.size == NDOF:
        return values
    if values.size == NDOF * NDOF:
        return values.reshape(NDOF, NDOF)
    raise ValueError(f"gain must contain 6 or 36 values, got {values.size}")


def parse_vec_any(text: str) -> np.ndarray:
    import re

    parts = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    return np.array([float(p) for p in parts], dtype=float)


def parse_limit_text(text, name: str):
    if text is None:
        return None
    values = parse_vec_any(text)
    if values.size == 1:
        values = np.repeat(values.item(), NDOF)
    elif values.size != NDOF:
        raise ValueError(f"{name} must contain 1 or {NDOF} values, got {values.size}")
    if np.any(values < 0.0):
        raise ValueError(f"{name} values must be non-negative")
    return values


def clip_symmetric(values: np.ndarray, limits):
    if limits is None:
        return values
    return np.clip(values, -limits, limits)


def split_or_blanks(values):
    if values is None:
        return [""] * NDOF
    return list(values)


def controller_type_name(controller_mode: str) -> str:
    return "computed_torque" if controller_mode == "none" else controller_mode


def default_csv_path(prefix: str) -> str:
    return str(Path("logs") / f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")


def ensure_parent(path: str) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)


def desired_trajectory(
    args,
    trajectory_fn,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    q_return: np.ndarray,
    elapsed: float,
):
    if not (args.return_to_start or args.return_home):
        return trajectory_fn(q_start, q_goal, elapsed, args.move_time)

    if elapsed <= args.move_time:
        return trajectory_fn(q_start, q_goal, elapsed, args.move_time)
    if elapsed <= args.move_time + args.hold_time:
        return q_goal.copy(), np.zeros(NDOF), np.zeros(NDOF)

    return_t = elapsed - args.move_time - args.hold_time
    return trajectory_fn(q_goal, q_return, return_t, args.return_time)


def return_phase_active(args, elapsed: float) -> bool:
    if not (args.return_to_start or args.return_home):
        return False
    return elapsed > args.move_time + args.hold_time


def trajectory_phase_name(args, elapsed: float) -> str:
    if args.return_to_start or args.return_home:
        if elapsed <= args.move_time:
            return "outbound"
        if elapsed <= args.move_time + args.hold_time:
            return "outbound_hold"
        return_t = elapsed - args.move_time - args.hold_time
        if return_t <= args.return_time:
            return "return"
        return "return_hold"
    if elapsed <= args.move_time:
        return "outbound"
    return "goal_hold"


def write_header(writer: csv.writer) -> None:
    writer.writerow(
        ["t"]
        + ["phase"]
        + [f"q_actual_{i + 1}" for i in range(NDOF)]
        + [f"dq_actual_{i + 1}" for i in range(NDOF)]
        + [f"q_des_{i + 1}" for i in range(NDOF)]
        + [f"dq_des_{i + 1}" for i in range(NDOF)]
        + [f"ddq_des_{i + 1}" for i in range(NDOF)]
        + [f"tau_{i + 1}" for i in range(NDOF)]
        + [f"tau_ff_{i + 1}" for i in range(NDOF)]
        + [f"tau_fb_{i + 1}" for i in range(NDOF)]
        + [f"tau_total_{i + 1}" for i in range(NDOF)]
        + ["controller_type"]
        + ["dynamics_mode", "finite_diff_step", "finite_diff_method"]
        + [f"e_int_{i + 1}" for i in range(NDOF)]
        + [f"tau_i_{i + 1}" for i in range(NDOF)]
    )


def print_summary(args, steps: int, q_start: np.ndarray, q_goal: np.ndarray, maxes: dict) -> None:
    print("dt =", args.dt)
    print("duration =", args.duration)
    print("move_time =", args.move_time)
    print("hold_time =", args.hold_time)
    print("return_to_start =", args.return_to_start)
    print("return_home =", args.return_home)
    if args.return_to_start or args.return_home:
        print("return_time =", args.return_time)
    print("number of steps =", steps)
    print("controller_type =", maxes["controller_type"])
    if args.return_to_start or args.return_home:
        print("return_controller_type =", maxes["return_controller_type"])
        print("outbound_steps =", maxes["outbound_steps"])
        print("return_steps =", maxes["return_steps"])
    print("dynamics_mode =", args.dynamics_mode)
    if args.dynamics_mode == "finite_difference":
        print("finite_diff_step =", args.finite_diff_step)
        print("finite_diff_method =", args.finite_diff_method)
    print("trajectory mode =", args.mode)
    print("trajectory_profile =", args.trajectory_profile)
    print("q_start =", np.array2string(q_start, precision=6, suppress_small=False))
    print("q_goal =", np.array2string(q_goal, precision=6, suppress_small=False))
    if maxes.get("return_target") is not None:
        print("q_return =", np.array2string(maxes["return_target"], precision=6, suppress_small=False))
    print("max |q_des - q_start| =", np.array2string(maxes["q_delta"], precision=6, suppress_small=False))
    print("max |dq_des| =", np.array2string(maxes["dq_des"], precision=6, suppress_small=False))
    print("max |ddq_des| =", np.array2string(maxes["ddq_des"], precision=6, suppress_small=False))
    print("max |tau| =", np.array2string(maxes["tau"], precision=6, suppress_small=False))
    if maxes.get("split_seen", False):
        print("max |tau_ff| =", np.array2string(maxes["tau_ff"], precision=6, suppress_small=False))
        print("max |tau_fb| =", np.array2string(maxes["tau_fb"], precision=6, suppress_small=False))
        print("max |tau_total| =", np.array2string(maxes["tau_total"], precision=6, suppress_small=False))
    print("max tracking error =", np.array2string(maxes["tracking_error"], precision=6, suppress_small=False))
    print("final tracking error =", np.array2string(maxes["final_tracking_error"], precision=6, suppress_small=False))
    if maxes.get("wall_elapsed", 0.0) > 0.0:
        print("effective loop rate =", f"{steps / maxes['wall_elapsed']:.1f} Hz")
    print("CSV path =", args.csv_log)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unitree Z1 computed-torque runtime loop", allow_abbrev=False)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--mode",
        choices=["hold_current", "one_joint_relative", "one_joint_absolute", "full_pose_absolute", "scaled_pose"],
        default="hold_current",
    )
    parser.add_argument("--joint", type=int, default=1)
    parser.add_argument("--angle-deg", "--angle", dest="angle_deg", type=float, default=0.0)
    parser.add_argument("--target", type=str, default=DEFAULT_TARGET)
    parser.add_argument("--scale", type=float, default=0.25)
    parser.add_argument("--move-time", type=float, default=5.0)
    parser.add_argument("--hold-time", type=float, default=0.0, help="hold at q_goal after outbound motion before return [s]")
    parser.add_argument("--return-to-start", action="store_true", help="after reaching q_goal, command a return trajectory to measured q_start instead of home")
    parser.add_argument("--return-home", dest="return_home", action="store_true", default=True, help="after reaching q_goal, command a return trajectory to --home-target (default)")
    parser.add_argument("--no-return-home", dest="return_home", action="store_false", help="after reaching q_goal, hold q_goal instead of returning home")
    parser.add_argument("--return-time", type=float, default=None, help="return trajectory duration [s]; defaults to --move-time")
    parser.add_argument("--home-target", type=str, default=DEFAULT_HOME_TARGET, help="home joint pose for --return-home")
    parser.add_argument(
        "--return-controller",
        choices=TEST_CONTROLLER_CHOICES,
        default=DEFAULT_RETURN_CONTROLLER,
        help="controller used during --return-home/--return-to-start phase",
    )
    parser.add_argument(
        "--return-kp",
        type=str,
        default=None,
        help=f"return-phase Kp gains; defaults to {DEFAULT_RETURN_KP} for augmented_pd_friction_model",
    )
    parser.add_argument(
        "--return-kd",
        type=str,
        default=None,
        help=f"return-phase Kd gains; defaults to {DEFAULT_RETURN_KD} for augmented_pd_friction_model",
    )
    parser.add_argument("--trajectory-profile", choices=["quintic", "scurve"], default="quintic")
    parser.add_argument("--kp", type=str, default=None)
    parser.add_argument("--kd", type=str, default=None)
    parser.add_argument("--ki", type=str, default=DEFAULT_KI)
    parser.add_argument("--integral-limit", type=str, default=DEFAULT_INTEGRAL_LIMIT, help="PID integral state limit, scalar or 6 values [rad*s]")
    parser.add_argument("--tau-limit", type=str, default=None, help="optional symmetric torque limit, scalar or 6 values [Nm]")
    parser.add_argument("--tau-fb-limit", type=str, default=None, help="optional symmetric augmented-PD feedback torque limit, scalar or 6 values [Nm]")
    parser.add_argument("--model-damping", type=str, default=DEFAULT_MODEL_DAMPING, help="Gazebo friction-model damping values, scalar or 6 values")
    parser.add_argument("--model-friction", type=str, default=DEFAULT_MODEL_FRICTION, help="Gazebo friction-model friction values, scalar or 6 values")
    parser.add_argument("--friction-deadband", type=float, default=0.002, help="position-error deadband before applying friction compensation [rad]")
    parser.add_argument("--wn", type=str, default=None)
    parser.add_argument("--zeta", type=str, default=None)
    parser.add_argument("--test-controller", choices=TEST_CONTROLLER_CHOICES, default="none")
    parser.add_argument("--dynamics-mode", choices=DYNAMICS_MODES, default="analytic")
    parser.add_argument("--finite-diff-step", type=float, default=1e-5, help="finite-difference dM step [rad]")
    parser.add_argument("--finite-diff-method", choices=["central", "forward"], default="central")
    parser.add_argument("--csv-log", type=str, default=default_csv_path("robot"))
    parser.add_argument("--runtime-dir", type=str, default=str(default_runtime_dir()))
    parser.add_argument("--state-timeout", type=float, default=1.0)
    parser.add_argument("--feedback-watchdog-timeout", type=float, default=0.75)
    parser.add_argument("--feedback-watchdog-command-threshold", type=float, default=0.01)
    parser.add_argument("--feedback-watchdog-state-threshold", type=float, default=1e-4)
    parser.add_argument("--disable-feedback-watchdog", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.duration < 0.0:
        raise ValueError("--duration must be non-negative")
    if args.hold_time < 0.0:
        raise ValueError("--hold-time must be non-negative")
    if args.move_time <= 0.0:
        raise ValueError("--move-time must be positive")
    if args.return_time is None:
        args.return_time = args.move_time
    if args.return_time <= 0.0:
        raise ValueError("--return-time must be positive")
    return_home_requested = "--return-home" in sys.argv
    if args.return_to_start and return_home_requested:
        raise ValueError("--return-to-start and --return-home are mutually exclusive")
    if args.return_to_start:
        args.return_home = False
    if args.return_to_start or args.return_home:
        minimum_duration = args.hold_time + args.move_time + args.return_time
        if args.duration < minimum_duration:
            raise ValueError(
                "--duration is too short for requested return phase: "
                f"need at least hold_time + move_time + return_time = {minimum_duration:.6g} s, "
                f"got {args.duration:.6g} s"
            )
    if args.finite_diff_step <= 0.0:
        raise ValueError("--finite-diff-step must be positive")

    target = parse_vec_any(args.target) if args.mode == "one_joint_absolute" else parse_vec6(args.target)
    home_target = parse_vec6(args.home_target)
    kp = parse_gain_text(args.kp)
    kd = parse_gain_text(args.kd)
    ki = parse_gain_text(args.ki)
    integral_limit = parse_limit_text(args.integral_limit, "--integral-limit")
    return_kp_text = args.return_kp
    return_kd_text = args.return_kd
    if args.return_controller == "augmented_pd_friction_model":
        if return_kp_text is None:
            return_kp_text = DEFAULT_RETURN_KP
        if return_kd_text is None:
            return_kd_text = DEFAULT_RETURN_KD
    return_kp = parse_gain_text(return_kp_text)
    return_kd = parse_gain_text(return_kd_text)
    tau_limit = parse_limit_text(args.tau_limit, "--tau-limit")
    tau_fb_limit = parse_limit_text(args.tau_fb_limit, "--tau-fb-limit")
    model_damping = parse_limit_text(args.model_damping, "--model-damping")
    model_friction = parse_limit_text(args.model_friction, "--model-friction")
    if args.friction_deadband < 0.0:
        raise ValueError("--friction-deadband must be non-negative")
    wn = DEFAULT_WN if args.wn is None else parse_vec6(args.wn)
    zeta = DEFAULT_ZETA if args.zeta is None else parse_vec6(args.zeta)
    controller_type = controller_type_name(args.test_controller)
    return_controller_type = controller_type_name(args.return_controller)
    trajectory_fn = quintic_trajectory if args.trajectory_profile == "quintic" else scurve_trajectory
    compute_test_tau = None
    compute_augmented_pd_components = None
    compute_augmented_pid_friction_model_components = None
    compute_computed_pid_model_components = None
    compute_computed_pid_friction_model_components = None
    active_controller_modes = {args.test_controller}
    if args.return_to_start or args.return_home:
        active_controller_modes.add(args.return_controller)
    if any(mode != "none" for mode in active_controller_modes):
        from test_controller import compute_test_tau
    if "augmented_pd" in active_controller_modes:
        from test_controller import compute_augmented_pd_components
    if "augmented_pid_friction_model" in active_controller_modes:
        from test_controller import compute_augmented_pid_friction_model_components
    if "computed_pid_model" in active_controller_modes:
        from test_controller import compute_computed_pid_model_components
    if "computed_pid_friction_model" in active_controller_modes:
        from test_controller import compute_computed_pid_friction_model_components
    if tau_fb_limit is not None and "augmented_pd" not in active_controller_modes:
        raise ValueError("--tau-fb-limit only applies when augmented_pd is an active controller")

    def compute_phase_tau(controller_mode, phase_kp, phase_kd, q_actual, dq_actual, q_des, dq_des, ddq_des, e_int):
        tau_ff = None
        tau_fb = None
        tau_i = np.zeros(NDOF, dtype=float)
        if controller_mode == "none":
            tau = compute_tau(
                q_actual,
                dq_actual,
                q_des,
                dq_des,
                ddq_des,
                kp=phase_kp,
                kd=phase_kd,
                wn=wn,
                zeta=zeta,
                dynamics_mode=args.dynamics_mode,
                finite_diff_step=args.finite_diff_step,
                finite_diff_method=args.finite_diff_method,
            )
        elif controller_mode == "augmented_pd":
            tau_ff, tau_fb, tau = compute_augmented_pd_components(
                q_actual,
                dq_actual,
                q_des,
                dq_des,
                ddq_des,
                kp=phase_kp,
                kd=phase_kd,
                dynamics_mode=args.dynamics_mode,
                finite_diff_step=args.finite_diff_step,
                finite_diff_method=args.finite_diff_method,
            )
            tau_fb = clip_symmetric(tau_fb, tau_fb_limit)
            tau = tau_ff + tau_fb
        elif controller_mode == "augmented_pid_friction_model":
            tau_ff, tau_fb, tau_i, tau = compute_augmented_pid_friction_model_components(
                q_actual,
                dq_actual,
                q_des,
                dq_des,
                ddq_des,
                e_int=e_int,
                kp=phase_kp,
                kd=phase_kd,
                ki=ki,
                model_damping=model_damping,
                model_friction=model_friction,
                friction_deadband=args.friction_deadband,
                dynamics_mode=args.dynamics_mode,
                finite_diff_step=args.finite_diff_step,
                finite_diff_method=args.finite_diff_method,
            )
        elif controller_mode == "computed_pid_friction_model":
            tau_i, tau = compute_computed_pid_friction_model_components(
                q_actual,
                dq_actual,
                q_des,
                dq_des,
                ddq_des,
                e_int=e_int,
                kp=phase_kp,
                kd=phase_kd,
                ki=ki,
                model_damping=model_damping,
                model_friction=model_friction,
                friction_deadband=args.friction_deadband,
                dynamics_mode=args.dynamics_mode,
                finite_diff_step=args.finite_diff_step,
                finite_diff_method=args.finite_diff_method,
            )
        elif controller_mode == "computed_pid_model":
            tau_i, tau = compute_computed_pid_model_components(
                q_actual,
                dq_actual,
                q_des,
                dq_des,
                ddq_des,
                e_int=e_int,
                kp=phase_kp,
                kd=phase_kd,
                ki=ki,
                dynamics_mode=args.dynamics_mode,
                finite_diff_step=args.finite_diff_step,
                finite_diff_method=args.finite_diff_method,
            )
        else:
            tau = compute_test_tau(
                q_actual,
                dq_actual,
                q_des,
                dq_des,
                ddq_des=ddq_des,
                mode=controller_mode,
                kp=phase_kp,
                kd=phase_kd,
                model_damping=model_damping,
                model_friction=model_friction,
                friction_deadband=args.friction_deadband,
                dynamics_mode=args.dynamics_mode,
                finite_diff_step=args.finite_diff_step,
                finite_diff_method=args.finite_diff_method,
            )
        return tau_ff, tau_fb, tau_i, tau

    stop_requested = False

    def request_stop(signum, frame):  # noqa: ARG001
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    io = FileRobotIO(args.runtime_dir)
    print(f"Waiting for first robot state from {io.sensor_path}")
    initial_sensor_mtime = io.sensor_path.stat().st_mtime_ns if io.sensor_path.exists() else None
    sensor_timestamp, q_start, dq_start, _ = io.read_state(
        timeout=args.state_timeout,
        newer_than_mtime_ns=initial_sensor_mtime,
    )
    print("first q_actual =", np.array2string(q_start, precision=6, suppress_small=False))
    print("first dq_actual =", np.array2string(dq_start, precision=6, suppress_small=False))
    q_goal = build_goal(q_start, args.mode, joint=args.joint, angle_deg=args.angle_deg, target=target, scale=args.scale)
    q_return = q_start.copy()
    if args.return_home:
        q_return = home_target.copy()

    ensure_parent(args.csv_log)
    steps = 0
    maxes = {
        "q_delta": np.zeros(NDOF),
        "dq_des": np.zeros(NDOF),
        "ddq_des": np.zeros(NDOF),
        "tau": np.zeros(NDOF),
        "tau_ff": np.zeros(NDOF),
        "tau_fb": np.zeros(NDOF),
        "tau_total": np.zeros(NDOF),
        "tracking_error": np.zeros(NDOF),
        "final_tracking_error": np.zeros(NDOF),
        "wall_elapsed": 0.0,
        "controller_type": controller_type,
        "return_controller_type": return_controller_type,
        "outbound_steps": 0,
        "return_steps": 0,
        "return_target": q_return.copy() if (args.return_to_start or args.return_home) else None,
        "split_seen": False,
    }
    frozen_feedback_since = None
    run_start = None
    e_int = np.zeros(NDOF, dtype=float)
    last_trajectory_phase = None

    try:
        with open(args.csv_log, "w", newline="") as f:
            writer = csv.writer(f)
            write_header(writer)
            t0 = time.perf_counter()
            run_start = t0
            next_tick = t0
            while not stop_requested:
                now = time.perf_counter()
                elapsed = now - t0
                if elapsed > args.duration:
                    break

                sensor_timestamp, q_actual, dq_actual, _ = io.read_state(
                    timeout=args.state_timeout,
                    newer_than_timestamp=sensor_timestamp,
                )
                q_des, dq_des, ddq_des = desired_trajectory(args, trajectory_fn, q_start, q_goal, q_return, elapsed)
                trajectory_phase = trajectory_phase_name(args, elapsed)
                if trajectory_phase != last_trajectory_phase:
                    e_int[:] = 0.0
                    last_trajectory_phase = trajectory_phase
                in_return_phase = trajectory_phase in ("return", "return_hold")
                phase = "return" if in_return_phase else "outbound"
                if in_return_phase:
                    active_controller_mode = args.return_controller
                    active_controller_type = return_controller_type
                    active_kp = return_kp
                    active_kd = return_kd
                    maxes["return_steps"] += 1
                else:
                    active_controller_mode = args.test_controller
                    active_controller_type = controller_type
                    active_kp = kp
                    active_kd = kd
                    maxes["outbound_steps"] += 1
                if active_controller_mode in PID_CONTROLLER_MODES:
                    next_e_int = e_int + (q_des - q_actual) * args.dt
                    if integral_limit is not None:
                        next_e_int = np.clip(next_e_int, -integral_limit, integral_limit)
                    e_int[:] = next_e_int
                    e_int_log = e_int.copy()
                else:
                    e_int_log = np.zeros(NDOF, dtype=float)
                tau_ff, tau_fb, tau_i, tau = compute_phase_tau(
                    active_controller_mode,
                    active_kp,
                    active_kd,
                    q_actual,
                    dq_actual,
                    q_des,
                    dq_des,
                    ddq_des,
                    e_int_log,
                )
                tau = clip_symmetric(tau, tau_limit)
                if not np.all(np.isfinite(tau)):
                    raise RuntimeError(f"computed non-finite torque; refusing to send tau={tau}")
                io.send_torque(tau)

                if not args.disable_feedback_watchdog:
                    planned_motion = np.max(np.abs(q_des - q_start))
                    actual_motion = max(np.max(np.abs(q_actual - q_start)), np.max(np.abs(dq_actual)))
                    if planned_motion >= args.feedback_watchdog_command_threshold and actual_motion <= args.feedback_watchdog_state_threshold:
                        if frozen_feedback_since is None:
                            frozen_feedback_since = elapsed
                        elif elapsed - frozen_feedback_since >= args.feedback_watchdog_timeout:
                            raise RuntimeError(
                                "feedback appears frozen while commanded motion is nonzero: "
                                f"max |q_des - q_start|={planned_motion:.6g}, "
                                f"max(|q_actual - q_start|, |dq_actual|)={actual_motion:.6g}. "
                                "Check the bridge feedback source or use --disable-feedback-watchdog if this is intentional."
                            )
                    else:
                        frozen_feedback_since = None

                tracking_error = q_des - q_actual
                writer.writerow(
                    [
                        elapsed,
                        phase,
                        *q_actual,
                        *dq_actual,
                        *q_des,
                        *dq_des,
                        *ddq_des,
                        *tau,
                        *split_or_blanks(tau_ff),
                        *split_or_blanks(tau_fb),
                        *tau,
                        active_controller_type,
                        args.dynamics_mode,
                        args.finite_diff_step if args.dynamics_mode == "finite_difference" else "",
                        args.finite_diff_method if args.dynamics_mode == "finite_difference" else "",
                        *e_int_log,
                        *tau_i,
                    ]
                )
                maxes["q_delta"] = np.maximum(maxes["q_delta"], np.abs(q_des - q_start))
                maxes["dq_des"] = np.maximum(maxes["dq_des"], np.abs(dq_des))
                maxes["ddq_des"] = np.maximum(maxes["ddq_des"], np.abs(ddq_des))
                maxes["tau"] = np.maximum(maxes["tau"], np.abs(tau))
                maxes["tau_total"] = np.maximum(maxes["tau_total"], np.abs(tau))
                if tau_ff is not None and tau_fb is not None:
                    maxes["split_seen"] = True
                    maxes["tau_ff"] = np.maximum(maxes["tau_ff"], np.abs(tau_ff))
                    maxes["tau_fb"] = np.maximum(maxes["tau_fb"], np.abs(tau_fb))
                maxes["tracking_error"] = np.maximum(maxes["tracking_error"], np.abs(tracking_error))
                maxes["final_tracking_error"] = tracking_error
                steps += 1

                next_tick += args.dt
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.perf_counter()
            maxes["wall_elapsed"] = time.perf_counter() - t0
    finally:
        if run_start is not None and maxes["wall_elapsed"] <= 0.0:
            maxes["wall_elapsed"] = time.perf_counter() - run_start
        for _ in range(20):
            try:
                io.send_zero_torque()
            except Exception as exc:
                print(f"warning: failed to send zero torque: {exc}", file=sys.stderr)
            time.sleep(args.dt)
        io.close()
        print_summary(args, steps, q_start, q_goal, maxes)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
