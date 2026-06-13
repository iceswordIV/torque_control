#!/usr/bin/env python3
"""Compare local analytic dynamics against Unitree SDK inverseDynamics.

The Unitree SDK exposes inverse dynamics, not M/C/N/dM directly. This script
recovers comparable quantities from repeated inverseDynamics calls:

    N(q)        = ID(q, 0, 0, 0)
    M[:, i]    = ID(q, 0, e_i, 0) - N(q)
    h(q, dq)   = C(q, dq) dq = ID(q, dq, 0, 0) - N(q)
    dM/dq      = finite difference of recovered M(q)

Run from z1_project:
    python3 compare_sdk_dynamics.py
"""

from __future__ import annotations

import argparse
import ctypes
import sys
from pathlib import Path

import numpy as np

from z1_analytic_dynamics import NDOF, coriolis_from_dM, dynamics_analytic

ROOT = Path(__file__).resolve().parents[1]
SDK_LIB = ROOT / "z1_sdk" / "lib"
if str(SDK_LIB) not in sys.path:
    sys.path.insert(0, str(SDK_LIB))

def load_unitree_interface():
    sdk_core = SDK_LIB / "libZ1_SDK_x86_64.so"
    if sdk_core.exists():
        ctypes.CDLL(str(sdk_core), mode=ctypes.RTLD_GLOBAL)
    try:
        import unitree_arm_interface
    except ImportError as exc:  # pragma: no cover - depends on local SDK binary
        raise SystemExit(f"failed to import Unitree SDK Python module from {SDK_LIB}: {exc}") from exc
    return unitree_arm_interface


def parse_vec6(text: str) -> np.ndarray:
    values = np.fromstring(text.replace(",", " "), sep=" ", dtype=float)
    if values.size != NDOF:
        raise argparse.ArgumentTypeError(f"expected {NDOF} values, got {values.size}: {text!r}")
    return values


def sdk_inverse_dynamics(model, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
    return np.asarray(model.inverseDynamics(q, dq, ddq, np.zeros(NDOF)), dtype=float).reshape(NDOF)


def sdk_gravity(model, q: np.ndarray) -> np.ndarray:
    zero = np.zeros(NDOF)
    return sdk_inverse_dynamics(model, q, zero, zero)


def sdk_mass_matrix(model, q: np.ndarray) -> np.ndarray:
    zero = np.zeros(NDOF)
    n = sdk_gravity(model, q)
    m = np.zeros((NDOF, NDOF), dtype=float)
    for i in range(NDOF):
        ddq = np.zeros(NDOF)
        ddq[i] = 1.0
        m[:, i] = sdk_inverse_dynamics(model, q, zero, ddq) - n
    return 0.5 * (m + m.T)


def sdk_coriolis_vector(model, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
    zero = np.zeros(NDOF)
    return sdk_inverse_dynamics(model, q, dq, zero) - sdk_gravity(model, q)


def finite_difference_dM(mass_fn, q: np.ndarray, eps: float) -> np.ndarray:
    dM = np.zeros((NDOF, NDOF, NDOF), dtype=float)
    for k in range(NDOF):
        step = np.zeros(NDOF)
        step[k] = eps
        dM[:, :, k] = (mass_fn(q + step) - mass_fn(q - step)) / (2.0 * eps)
    return dM


def print_vec(name: str, value: np.ndarray) -> None:
    print(f"{name} = {np.array2string(value, precision=9, suppress_small=False)}")


def print_matrix_summary(name: str, value: np.ndarray) -> None:
    print_vec(f"diag({name})", np.diag(value))
    print(f"max asym {name} = {np.max(np.abs(value - value.T)):.12g}")
    print(f"cond {name} = {np.linalg.cond(value):.12g}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare analytic dynamics with Unitree SDK inverseDynamics")
    parser.add_argument("--q", type=parse_vec6, default=parse_vec6("0.2 0.25 -0.25 0.15 0.1 0.1"))
    parser.add_argument("--dq", type=parse_vec6, default=parse_vec6("0.5 -0.3 0.2 -0.1 0.4 -0.2"))
    parser.add_argument("--ddq", type=parse_vec6, default=parse_vec6("0.1 -0.2 0.3 -0.1 0.05 -0.02"))
    parser.add_argument("--eps", type=float, default=1e-6)
    parser.add_argument(
        "--sdk-no-gripper",
        "--no-gripper",
        dest="sdk_no_gripper",
        action="store_true",
        help="construct SDK ArmInterface(hasGripper=False); the local Python model is unchanged",
    )
    args = parser.parse_args()

    if args.eps <= 0.0:
        raise ValueError("--eps must be positive")

    q = args.q.astype(float)
    dq = args.dq.astype(float)
    ddq = args.ddq.astype(float)
    has_gripper = not args.sdk_no_gripper

    unitree_arm_interface = load_unitree_interface()
    arm = unitree_arm_interface.ArmInterface(hasGripper=has_gripper)
    model = arm._ctrlComp.armModel

    M_py, C_py, N_py, dM_py = dynamics_analytic(q, dq)
    h_py = C_py @ dq
    tau_py = M_py @ ddq + h_py + N_py

    M_sdk = sdk_mass_matrix(model, q)
    N_sdk = sdk_gravity(model, q)
    h_sdk = sdk_coriolis_vector(model, q, dq)
    tau_sdk = sdk_inverse_dynamics(model, q, dq, ddq)
    dM_sdk = finite_difference_dM(lambda x: sdk_mass_matrix(model, x), q, args.eps)
    h_sdk_from_dM = coriolis_from_dM(dM_sdk, dq) @ dq

    print("Unitree SDK inverseDynamics comparison")
    print(f"SDK hasGripper = {has_gripper}")
    print("Python model = current z1_analytic_dynamics.py merged link6/gripper model")
    print_vec("q", q)
    print_vec("dq", dq)
    print_vec("ddq", ddq)
    print()

    print_matrix_summary("M_sdk", M_sdk)
    print_matrix_summary("M_python", M_py)
    print_vec("diag(M_python - M_sdk)", np.diag(M_py - M_sdk))
    print(f"max abs M diff = {np.max(np.abs(M_py - M_sdk)):.12g}")
    print()

    print_vec("N_sdk", N_sdk)
    print_vec("N_python", N_py)
    print_vec("N_python - N_sdk", N_py - N_sdk)
    print(f"max abs N diff = {np.max(np.abs(N_py - N_sdk)):.12g}")
    print()

    print_vec("h_sdk = ID(q,dq,0)-N", h_sdk)
    print_vec("h_sdk_from_fd_dM", h_sdk_from_dM)
    print_vec("h_python = C@dq", h_py)
    print_vec("h_python - h_sdk", h_py - h_sdk)
    print(f"norm h_sdk - h_sdk_from_fd_dM = {np.linalg.norm(h_sdk - h_sdk_from_dM):.12g}")
    print(f"norm h_python - h_sdk = {np.linalg.norm(h_py - h_sdk):.12g}")
    print()

    print_vec("tau_sdk = ID(q,dq,ddq)", tau_sdk)
    print_vec("tau_python", tau_py)
    print_vec("tau_python - tau_sdk", tau_py - tau_sdk)
    print(f"norm tau diff = {np.linalg.norm(tau_py - tau_sdk):.12g}")
    print()

    print(f"max abs dM_python - fd_dM_sdk = {np.max(np.abs(dM_py - dM_sdk)):.12g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
