# 2026-05-28 Torque Control Work Report

## Main Finding From `robot_20260528_003416.csv`

Command tested:

```bash
python3 torque_main.py --mode one_joint_relative --joint 1 --angle-deg 60 \
  --move-time 10 --duration 11 \
  --dynamics-mode finite_difference \
  --finite-diff-step 1e-5 \
  --finite-diff-method central \
  --test-controller feedforward_friction_model
```

The large J1 error comes mainly from using `feedforward_friction_model`, which is
open-loop. This controller sends:

```text
tau = M(q) ddq_des + C(q,dq) dq + N(q) + D dq_des + F direction
```

It does not contain `Kp * (q_des - q_actual)` or
`Kd * (dq_des - dq_actual)`, so it cannot correct model error, friction error,
torque mapping error, delay, or any initial tracking lag.

Important numbers from the log:

```text
q_start_1        = -0.042695 rad
q_goal_1         =  1.004503 rad
desired motion   =  1.047198 rad
actual motion    =  0.823385 rad
final q_actual_1 =  0.780690 rad
final error J1   =  0.223813 rad = 12.82 deg
max error J1     =  0.224593 rad
tau_1 at end     =  1.000000 Nm
```

The error accumulated during the 10 s move. After the trajectory finished,
`q_des` stopped at the goal, `dq_des = 0`, and the controller kept sending about
`+1.0 Nm`, which is just the configured friction compensation. In the final
1 s hold, J1 moved only about `0.00062 rad`, so that open-loop friction torque
was not enough to remove the remaining position error.

Comparison with the previous `gazebo_friction_model` run:

```text
log: logs/robot_20260528_000144.csv
controller: gazebo_friction_model
final J1 error: 0.001903 rad
```

That controller uses the same friction compensation but also includes feedback:

```text
ddq_cmd = ddq_des + Kd * de + Kp * e
tau = M(q) ddq_cmd + C(q,dq) dq + N(q) + friction_terms
```

This explains why the PD+friction run can reach the target while pure
feedforward+friction stays about `0.224 rad` behind.

## Secondary Log Issue

The same CSV shows that `dq_actual_1` is not fully consistent with the logged
`q_actual_1` and the Python-side `t` column. Using the CSV time column,
integrating `dq_actual_1` gives about `1.049 rad`, but `q_actual_1` changes only
`0.823 rad`.

This does not explain the final position error by itself, because the position
error is directly visible in `q_actual_1`. It does mean velocity-based diagnosis
should be treated carefully until we log the bridge sensor timestamp and/or add
a derived velocity from finite differences of `q_actual`.

## Work Completed Today

- Added faster dynamics options so the runtime loop no longer depends only on
  the slow full analytic `M, C, N, dM` path.
- Added finite-difference dynamics mode:
  `--dynamics-mode finite_difference`,
  `--finite-diff-step`, and `--finite-diff-method central|forward`.
- Added compiled finite-difference support in `z1_analytic_dynamics.py`.
- Added `compare_finite_difference_dynamics.py` to compare finite-difference
  speed and torque error against the analytic model.
- Added diagnostic controller modes in `test_controller.py`, including:
  `augmented_pd`, `gazebo_friction_model`, and
  `feedforward_friction_model`.
- Added return phases to `torque_main.py`:
  `--return-to-start`, `--return-home`, and `--return-time`.
- Updated `--return-home` to use Unitree `startFlat`, not all zeros:
  `0 0 -0.005 -0.074 0 0`.
- Fixed the C++ bridge `--back-to-start-end` path so it starts the Unitree
  send/receive thread before calling `arm.backToStart()`.
- Updated README usage examples for finite-difference dynamics, friction-model
  tests, and return-home/return-start runs.

## Current Practical Recommendation

For real tracking tests, use `gazebo_friction_model` or another controller with
feedback. Use `feedforward_friction_model` only as an identification test to
measure how much open-loop model/friction error remains.

Recommended next command for a full out-and-home test:

```bash
python3 torque_main.py --mode one_joint_relative --joint 1 --angle-deg 60 \
  --move-time 10 --return-home --return-time 10 --duration 21 \
  --dynamics-mode finite_difference \
  --finite-diff-step 1e-5 \
  --finite-diff-method central \
  --test-controller gazebo_friction_model
```

Recommended next code improvement: log the bridge sensor timestamp in
`torque_main.py` and add a plot/analysis column for velocity estimated from
`q_actual`, so velocity feedback and SDK velocity can be compared directly.
