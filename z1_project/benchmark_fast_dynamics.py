#!/usr/bin/env python3
"""Benchmark the Z1 analytic dynamics backend."""

from __future__ import annotations

import argparse
import time

import numpy as np

import z1_analytic_dynamics as dyn


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark z1_analytic_dynamics.dynamics_analytic")
    parser.add_argument("--n", type=int, default=2000, help="number of timed dynamics calls")
    parser.add_argument("--warmup", type=int, default=20, help="number of untimed warmup calls")
    args = parser.parse_args()
    if args.n <= 0:
        raise ValueError("--n must be positive")
    if args.warmup < 0:
        raise ValueError("--warmup must be non-negative")

    q = np.array([0.0, 1.5, -1.0, -0.54, 0.2, -0.1], dtype=float)
    dq = np.array([0.05, -0.03, 0.04, -0.02, 0.01, -0.015], dtype=float)

    print("FAST_DYNAMICS =", dyn._FAST_DYNAMICS)
    print("using fast =", dyn._FAST_DYNAMICS is not None)
    print("calls =", args.n)
    print("warmup =", args.warmup)

    result = None
    for _ in range(args.warmup):
        result = dyn.dynamics_analytic(q, dq)

    t0 = time.perf_counter()
    for _ in range(args.n):
        result = dyn.dynamics_analytic(q, dq)
    total = time.perf_counter() - t0

    if result is None:
        result = dyn.dynamics_analytic(q, dq)
    M, _C, N, _dM = result
    avg = total / args.n

    print("total time [s] =", f"{total:.6f}")
    print("average time per call [us] =", f"{avg * 1e6:.3f}")
    print("equivalent max loop rate [Hz] =", f"{1.0 / avg:.1f}")
    print("sample M[0,0] =", f"{M[0, 0]:.12g}")
    print("sample N =", np.array2string(N, precision=9, suppress_small=False))
    print()
    print("Compare pure Python with:")
    print("  Z1_DISABLE_FAST_DYNAMICS=1 python3 benchmark_fast_dynamics.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
