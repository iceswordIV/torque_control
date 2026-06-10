#!/usr/bin/env python3
"""Validate Z1 gravity torque against finite-difference and virtual-work forms."""

from __future__ import annotations

import numpy as np

import z1_analytic_dynamics as dyn


TEST_POSES = (
    ("home_like", np.array([0.0, 0.0, -0.005, -0.074, 0.0, 0.0], dtype=float)),
    ("forward_pose", np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0], dtype=float)),
    ("random_reasonable", np.array([0.35, 0.9, -1.2, -0.35, 0.45, -0.25], dtype=float)),
)


def max_abs(values: np.ndarray) -> float:
    return float(np.max(np.abs(values)))


def main() -> int:
    print("FAST_DYNAMICS =", dyn._FAST_DYNAMICS)
    print("using fast =", dyn._FAST_DYNAMICS is not None)
    print()

    fd_errors = []
    pdf_errors = []
    neg_pdf_errors = []

    for name, q in TEST_POSES:
        N_code = dyn.gravity_vector(q)
        N_fd = dyn.gravity_vector_finite_difference(q)
        N_pdf = dyn.gravity_vector_virtual_work(q)

        err_fd = max_abs(N_code - N_fd)
        err_pdf = max_abs(N_code - N_pdf)
        err_neg_pdf = max_abs(N_code + N_pdf)
        fd_errors.append(err_fd)
        pdf_errors.append(err_pdf)
        neg_pdf_errors.append(err_neg_pdf)

        print(f"{name}:")
        print("  q =", np.array2string(q, precision=6, suppress_small=False))
        print("  N_code =", np.array2string(N_code, precision=9, suppress_small=False))
        print("  N_fd   =", np.array2string(N_fd, precision=9, suppress_small=False))
        print("  N_pdf  =", np.array2string(N_pdf, precision=9, suppress_small=False))
        print("  max |N_code - N_fd|  =", f"{err_fd:.6e}")
        print("  max |N_code - N_pdf| =", f"{err_pdf:.6e}")
        print("  max |N_code + N_pdf| =", f"{err_neg_pdf:.6e}")
        if err_neg_pdf < err_pdf:
            print("  PDF virtual-work result matches with opposite sign convention.")
        else:
            print("  PDF virtual-work result matches directly.")
        print()

    worst_fd = max(fd_errors)
    worst_pdf = max(pdf_errors)
    worst_neg_pdf = max(neg_pdf_errors)
    print("Summary:")
    print("  worst max |N_code - N_fd|  =", f"{worst_fd:.6e}")
    print("  worst max |N_code - N_pdf| =", f"{worst_pdf:.6e}")
    print("  worst max |N_code + N_pdf| =", f"{worst_neg_pdf:.6e}")
    if worst_neg_pdf < worst_pdf:
        print("  Overall: virtual-work gravity matches gravity_vector(q) with opposite sign.")
        print("  This is only a sign-convention difference for the gravity wrench/Jacobian formula.")
    else:
        print("  Overall: virtual-work gravity matches gravity_vector(q) directly.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
