#!/usr/bin/env python3
"""Build the compiled analytic Z1 dynamics module."""

from __future__ import annotations

import numpy as np
from Cython.Build import cythonize
from setuptools import Extension, setup


setup(
    name="z1_analytic_dynamics_fast",
    ext_modules=cythonize(
        [
            Extension(
                "z1_analytic_dynamics_fast",
                ["z1_analytic_dynamics_fast.pyx"],
                include_dirs=[np.get_include()],
                extra_compile_args=["-O3"],
            )
        ],
        compiler_directives={"language_level": 3},
    ),
)
