# Unitree Z1 Computed-Torque Project

## Project Idea

The runtime loop is deliberately simple:

```text
robot / Gazebo / Unitree bridge gives measured q, dq
    -> Python trajectory generates q_des, dq_des, ddq_des
    -> Python analytic dynamics computes M, C, N, dM
    -> Python computed torque computes tau
    -> Python sends tau to C++ bridge
    -> C++ bridge calls arm.setArmCmd(q_actual, dq_actual, tau_cmd)
    -> C++ bridge calls arm.setGripperCmd(0.0, 0.0, 0.0)
    -> C++ bridge calls arm.sendRecv()
```

`torque_main.py` does not integrate `q` or `dq`. The real robot or Gazebo bridge supplies measured joint position and velocity. Integration exists only in `simulate_offline.py`.

The default analytic dynamics file uses the Lie-bracket method for `dM/dq`.
There is also an opt-in finite-difference diagnostic path that computes `dM/dq`
from repeated `M(q)` calls so the error and runtime cost can be measured before
using it in runtime control.
If `z1_analytic_dynamics_fast` is built, the same analytic formulas run through a compiled
Cython accelerator. Set `Z1_DISABLE_FAST_DYNAMICS=1` to force the pure Python reference path.

## Important Timing

`dt = 0.002 s` targets 500 Hz, but the dynamics and file IPC must also finish within 2 ms.
Check the printed effective loop rate after each run. On this machine, the compiled analytic
`M/C/N/dM` path is much faster than the target loop period; the pure Python reference path is
kept for validation.

Build the compiled analytic dynamics module after editing `z1_analytic_dynamics_fast.pyx`:

```bash
python3 setup_fast_dynamics.py build_ext --inplace
```

`move_time = 6 s` means 3000 loop steps at 500 Hz.

`hold_time` holds `q_goal` after the outbound move and before any return phase.
By default, after this hold ends the trajectory commands a second trajectory to
the configured home pose. Add `--no-return-home` to hold `q_goal` after the move,
or add `--return-to-start` to return to the measured `q_start` instead. For a
return phase, `--duration` must be at least `move_time + hold_time + return_time`.

## Offline Preview Commands

Run these from the `z1_project/` directory.

Compare analytic `dM` against finite-difference `dM` for speed and torque error:

```bash
python3 compare_finite_difference_dynamics.py \
  --mode full_pose_absolute \
  --target "0 1.5 -1 -0.54 0 0" \
  --trajectory-profile scurve \
  --move-time 6 \
  --duration 6 \
  --finite-diff-methods "central forward" \
  --finite-diff-steps "1e-3 1e-4 1e-5"
```

Run the runtime controller with finite-difference `dM`:

```bash
python3 torque_main.py \
  --mode one_joint_relative \
  --joint 1 \
  --angle-deg 5 \
  --move-time 5 \
  --duration 11 \
  --dynamics-mode finite_difference \
  --finite-diff-step 1e-5 \
  --finite-diff-method central \
  --csv-log logs/robot_j1_5deg_fd.csv
```

Run feedforward dynamics plus Gazebo damping/friction compensation, without PD:

```bash
python3 torque_main.py \
  --mode one_joint_relative \
  --joint 1 \
  --angle-deg 60 \
  --move-time 10 \
  --duration 21 \
  --dynamics-mode finite_difference \
  --finite-diff-step 1e-5 \
  --finite-diff-method central \
  --test-controller feedforward_friction_model \
  --csv-log logs/robot_j1_60deg_ff_friction.csv
```

Run model feedforward plus direct augmented PD and Gazebo damping/friction
compensation:

```bash
python3 torque_main.py \
  --mode one_joint_relative \
  --joint 1 \
  --angle-deg 30 \
  --trajectory-profile scurve \
  --move-time 10 \
  --return-to-start \
  --duration 22 \
  --test-controller augmented_pd_friction_model \
  --model-damping "1 2 1 1 1 1" \
  --model-friction "1 2 1 1 1 1" \
  --csv-log logs/j1_30deg_scurve_augpd_friction_return_start.csv
```

For isolating one joint at an absolute target while the other joints hold their
measured start pose, use `one_joint_absolute`:

```bash
python3 torque_main.py \
  --mode one_joint_absolute \
  --joint 4 \
  --target "-0.54" \
  --trajectory-profile scurve \
  --move-time 20 \
  --duration 22 \
  --no-return-home \
  --test-controller augmented_pd_friction_model \
  --dynamics-mode analytic \
  --kp "20 20 40 8 5 5" \
  --kd "3 3 6 1 0.6 0.4" \
  --model-damping "1 2 1 1 1 1" \
  --model-friction "1 2 1 1 1 1" \
  --tau-limit "20 40 25 12 10 10" \
  --csv-log logs/j4_abs_neg054_augpd_friction_retuned.csv
```

For tests that should come back to the measured start pose, the return phase
uses `augmented_pd_friction_model` by default with conservative return gains:
`--return-kp "20 20 40 8 5 5"` and
`--return-kd "3 3 6 1 0.6 0.4"`. Override `--return-controller`,
`--return-kp`, or `--return-kd` if needed:

```bash
python3 torque_main.py \
  --mode one_joint_relative \
  --joint 1 \
  --angle-deg 60 \
  --move-time 10 \
  --return-to-start \
  --return-time 10 \
  --duration 21 \
  --dynamics-mode finite_difference \
  --finite-diff-step 1e-5 \
  --finite-diff-method central \
  --test-controller feedforward_friction_model \
  --return-controller augmented_pd_friction_model
```

By default, tests finish at the configured home pose. The default home target
matches Unitree `startFlat` from
`z1_controller/config/savedArmStates.csv`:
`0 0 -0.005 -0.074 0 0`. The home return also uses
`augmented_pd_friction_model` unless `--return-controller` is changed. Override
the target with `--home-target` if needed:

```bash
python3 torque_main.py \
  --mode one_joint_relative \
  --joint 1 \
  --angle-deg 60 \
  --move-time 10 \
  --return-time 10 \
  --duration 21 \
  --dynamics-mode finite_difference \
  --finite-diff-step 1e-5 \
  --finite-diff-method central \
  --test-controller gazebo_friction_model
```

One-joint 5 deg:

```bash
python3 compare_torque.py --mode one_joint_relative --joint 1 --angle-deg 5 --move-time 5 --duration 6 --csv-log logs/preview_j1_5deg.csv
```

One-joint 30 deg:

```bash
python3 compare_torque.py --mode one_joint_relative --joint 1 --angle-deg 30 --move-time 8 --duration 9 --csv-log logs/preview_j1_30deg.csv
```

Full forward pose:

```bash
python3 compare_torque.py --mode full_pose_absolute --target "0 1.5 -1 -0.54 0 0" --move-time 6 --duration 7 --csv-log logs/preview_forward_6s.csv
```

## Offline Closed-Loop Simulation Commands

One-joint 5 deg:

```bash
python3 simulate_offline.py --mode one_joint_relative --joint 1 --angle-deg 5 --move-time 5 --duration 6 --csv-log logs/sim_j1_5deg.csv
```

Full forward pose:

```bash
python3 simulate_offline.py --mode full_pose_absolute --target "0 1.5 -1 -0.54 0 0" --move-time 6 --duration 7 --csv-log logs/sim_forward_6s.csv
```

## Plot Commands

```bash
python3 plot_log.py logs/sim_j1_5deg.csv
python3 plot_log.py logs/preview_forward_6s.csv --show
```

PNG files are written to `logs/plots/`.

## C++ Bridge Build

```bash
cd cpp
mkdir -p build
cd build
cmake .. -DUNITREE_ARM_SDK_PATH=/path/to/unitree_arm_sdk
make -j
```

The bridge uses file IPC in `/tmp/z1_torque_<uid>` by default:

- `z1_torque_cmd.txt`: Python writes commanded torque.
- `z1_sensor.txt`: C++ writes timestamp, `q`, `dq`, and the active torque command.
- `z1_stop.txt`: if present, the C++ bridge exits its loop.

## Real Robot Test Sequence

The `unitree_gazebo z1.launch` simulation in `unitree_ros` uses ROS control topics, not the
Unitree arm SDK UDP feedback path. For that Gazebo launch, use the ROS bridge:

```bash
python3 ros_torque_bridge.py
```

It reads `/z1_gazebo/joint_states`, publishes torque-only `MotorCmd` messages to
`/z1_gazebo/Joint01_controller/command` through `/z1_gazebo/Joint06_controller/command`,
and writes the same `/tmp/z1_torque_<uid>/z1_sensor.txt` file used by `torque_main.py`.

Start the C++ bridge:

```bash
./pure_torque_bridge
```

For the Unitree ROS/Gazebo simulation, the SDK examples commonly use the alternate SDK ports:

```bash
./pure_torque_bridge --gazebo-ports
```

This is shorthand for:

```bash
./pure_torque_bridge --udp-to-ip 127.0.0.1 --udp-to-port 8073 --udp-own-port 8074
```

The simulator side must use the reversed pair, for example `ARMSDK(..., 8074, 8073, ...)`.
If the bridge is connected to the wrong feedback source, the Python controller can still send
torque while logging `q_actual = 0` and `dq_actual = 0`.

Start Python controller, small test first:

```bash
python3 torque_main.py --mode one_joint_relative --joint 1 --angle-deg 5 --move-time 5 --duration 11 --csv-log logs/robot_j1_5deg.csv
```

Then test:

- joint 1, 5 deg
- joint 1, 10 deg
- joint 1, 30 deg
- scaled forward pose 25%
- scaled forward pose 50%
- full forward pose 100%

Full forward pose target:

```text
[0, 1.5, -1.0, -0.54, 0, 0]
```

Example scaled-pose command:

```bash
python3 torque_main.py --mode scaled_pose --target "0 1.5 -1 -0.54 0 0" --scale 0.25 --move-time 6 --duration 13 --csv-log logs/robot_forward_25pct.csv
```

## SDK Forward Torque Replay Check

This test records the torque produced by the Unitree SDK example-style forward-pose lowcmd motion,
then replays only that same torque through the ROS torque bridge.

Record the SDK trace:

```bash
python3 record_sdk_forward_torque.py --csv-log logs/sdk_forward_tau.csv
```

If SDK UDP is not connected and you only need a replayable inverse-dynamics torque trace, generate
the same command-shape offline from a known start pose:

```bash
python3 record_sdk_forward_torque.py --offline --start-q "0 0 0 0 0 0" --csv-log logs/sdk_forward_tau.csv
```

Start the ROS/Gazebo torque bridge in another terminal:

```bash
python3 ros_torque_bridge.py
```

Replay the recorded torque through the bridge:

```bash
python3 replay_recorded_torque.py logs/sdk_forward_tau.csv --csv-log logs/replay_sdk_forward_tau.csv
```

Important interpretation: the SDK lowcmd example sends `q`, `qd`, and `tau`. If SDK lowcmd gains
are active, the recorded SDK movement may include position/velocity servo effort, while the replay
script sends torque only. A failed torque-only replay therefore proves that the recorded inverse-
dynamics torque alone is insufficient, but it does not by itself prove that the ROS bridge is wrong.

To reproduce the SDK lowcmd effect in Gazebo, stop `ros_torque_bridge.py` and replay the full
`q/dq/tau` command with Gazebo gains. Use a clean source CSV from
`record_sdk_forward_torque.py`; do not use a replay output CSV as the source.

```bash
python3 replay_sdk_lowcmd_gazebo.py logs/sdk_forward_tau_source_offline.csv \
  --csv-log logs/sdk_lowcmd_replay_forward_gazebo.csv \
  --kp "80 120 120 80 40 30" \
  --kd "150 150 150 150 80 80" \
  --hold-final-sec 8.0
```

This is not pure torque control. It uses the Gazebo controller's
`Kp * (q_cmd - q) + Kd * (dq_cmd - dq) + tau_cmd` path.
Do not add `--zero-on-exit` if you want the arm to remain at the final pose.

## Warning

- Do not start with full forward pose on real hardware.
- Preview torque first.
- Start with small one-joint motions.
- Use conservative gains.
- Keep Ctrl+C cleanup enabled.
- The runtime controller has a feedback watchdog. It aborts if the desired motion becomes nonzero
  but measured `q` and `dq` remain frozen, because that usually means the bridge is not reading the
  real robot/Gazebo joint state.
