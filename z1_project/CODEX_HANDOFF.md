# Codex handoff: Z1 forward-pose torque / J4 diagnosis

Date: 2026-05-27

## 2026-05-29 update

- `torque_main.py` now supports a separate return-phase controller.
- `--return-controller` defaults to `augmented_pd_friction_model` for
  `--return-home` and `--return-to-start`.
- Return-specific defaults are tuned lower for direct torque PD:
  `--return-kp "20 20 40 8 5 5"` and
  `--return-kd "3 3 6 1 0.6 0.4"`.
- CSV logs now include a `phase` column and per-row active `controller_type`.
- Work record:
  `reports/2026-05-29_return_home_controller_work_report.md`.

## Current user question

The user is debugging why the Unitree SDK can move the Z1 arm to the forward
pose with near-zero error, but the ROS/Gazebo pure torque bridge and
`torque_main.py` augmented PD do not. The sharp symptom is J4:

```text
target q4 = -0.54
augmented-PD final q4 ~= +0.537
```

That looks like a sign/coupling/plant problem, but the current evidence says it
is mainly a slow external pure-torque feedback-loop instability, not a simple
bridge sign flip.

## Important distinction

The SDK command is not "pure torque only".

For SDK/Gazebo lowcmd, the applied controller law is:

```text
tau_applied = Kp * (q_cmd - q_actual) + Kd * (dq_cmd - dq_actual) + tau_cmd
```

The SDK-recorded `tau` in the forward trace is feedforward:

```text
tau_cmd = inverseDynamics(q_cmd, dq_cmd, ddq_cmd, 0)
```

The real effort in Gazebo/lowcmd also includes the internal PD terms. Replaying
only the recorded `tau_cmd` through a pure torque bridge is therefore not
expected to reproduce the SDK motion.

## Files/scripts involved

- `record_sdk_forward_torque.py`
  - Records/generates SDK-style forward trace with `q_cmd`, `dq_cmd`,
    `ddq_cmd`, and feedforward `tau`.
- `replay_recorded_torque.py`
  - Replays recorded feedforward torque only through the torque bridge.
- `replay_sdk_lowcmd_gazebo.py`
  - Replays full SDK-style lowcmd directly to Gazebo controllers:
    `q_cmd`, `dq_cmd`, `tau_cmd`, `Kp`, `Kd`.
- `ros_torque_bridge.py`
  - File bridge from `torque_main.py` to ROS/Gazebo. It sends `MotorCmd` with
    measured `q/dq`, commanded `tau`, and `Kp = Kd = 0`.
- `torque_main.py`
  - Runtime pure-torque controller. With `--test-controller augmented_pd`, it
    computes:
    `tau = tau_ff + Kp*(q_des-q_actual) + Kd*(dq_des-dq_actual)`.
- `test_controller.py`
  - Contains `compute_augmented_pd_components()`.

## Source-code facts checked

Gazebo Unitree joint controller torque law:

```cpp
calcTorque = posStiffness*(targetPos-currentPos)
           + velStiffness*(targetVel-currentVel)
           + targetTorque;
```

Source:

```text
/home/icesword/Desktop/unitree_ws/src/unitree_ros/unitree_legged_control/src/unitree_joint_control_tool.cpp
```

J4 mapping is normal:

```text
/home/icesword/Desktop/unitree_ws/src/unitree_ros/robots/z1_description/config/robot_control.yaml
Joint04_controller -> joint4

/home/icesword/Desktop/unitree_ws/src/unitree_ros/robots/z1_description/xacro/robot.xacro
joint4 axis xyz="0 1 0"
```

The pure torque bridge publishes `Kp = 0`, `Kd = 0`:

```text
ros_torque_bridge.py::_publish_torque()
```

So when using `torque_main.py`, all PD feedback is external and depends on the
bridge/file/topic feedback rate.

## Key logs and results

### Bad full-forward augmented-PD run

Log:

```text
logs/augpd_forward_controllerstate_safe.csv
```

Command used:

```bash
python3 torque_main.py --mode full_pose_absolute --target "0 1.5 -1 -0.54 0 0" \
  --trajectory-profile scurve --move-time 6 --duration 9 \
  --test-controller augmented_pd \
  --kp "40 60 60 20 20 20" \
  --kd "15 20 20 10 8 8" \
  --tau-limit "20 40 30 20 10 10" \
  --tau-fb-limit "20 40 30 15 10 10" \
  --csv-log logs/augpd_forward_controllerstate_safe.csv
```

Result:

```text
target    = [0, 1.5, -1, -0.54, 0, 0]
final q   = [-0.001979, 1.531163, -0.919488, +0.536799, 0.004104, -0.008831]
J4 error  = -1.0768 rad
```

Observed log rate:

```text
samples = 475
duration ~= 8.99 s
effective rate ~= 53 Hz
median dt ~= 18 ms
```

J4 does not simply receive negative torque and move positive. Around the start
of instability, J4 torque/velocity flip signs rapidly:

```text
t=1.4306  q4=-0.0087  qdes4=-0.0483  dq4=+3.0150  tau_fb4=-15.0000
t=1.4901  q4=+0.0150  qdes4=-0.0528  dq4=-3.4512  tau_fb4=+15.0000
t=1.5611  q4=-0.0871  qdes4=-0.0587  dq4=+0.4056  tau4=-6.5240
t=1.6325  q4=+0.1373  qdes4=-0.0652  dq4=-2.0698  tau4=+12.8593
```

This is a saturated sampled-data oscillation. The derivative term tries to
brake the large velocity, so the sign of torque flips.

### J4-only augmented-PD asymmetry

Logs:

```text
logs/week_scurve/augmented_pd/j4_neg5_scurve_augpd.csv
logs/week_scurve/augmented_pd/j4_pos5_scurve_augpd.csv
```

Negative J4 test:

```text
final q4 ~= +0.2637
target q4 ~= +0.0749
dq4 final ~= +5.48
tau range ~= -170 to +143 Nm
effective rate ~= 37 Hz
unstable
```

Positive J4 test:

```text
final q4 ~= +0.1658
target q4 ~= +0.2247
tau range ~= -2.64 to -0.66 Nm
effective rate ~= 43 Hz
stable but undertracks
```

The asymmetry is real, but it does not prove a sign-flipped topic. It is
consistent with J4 gravity/friction/low inertia plus a delayed external PD loop.

### SDK-style lowcmd replay succeeds much better

Logs:

```text
logs/sdk_lowcmd_replay_forward_clean_kp_high_hold8.csv
logs/sdk_lowcmd_replay_forward_clean_kp_strong_hold8.csv
logs/sdk_lowcmd_replay_forward_clean_reset.csv
```

Example, high-gain lowcmd replay:

```text
samples ~= 3009
rate ~= 300 Hz
q4 start ~= -0.0589
q4 final ~= -0.5037
q4 target = -0.54
q4 error ~= -0.0363
```

This uses Gazebo's internal `Kp/Kd` controller path, not pure external torque.
That is why it behaves much more like the SDK.

## Feedforward comparison

Earlier confusion: comparing a no-gripper standalone `Z1Model` to the SDK
`ArmInterface(hasGripper=True)` made feedforward torques appear mismatched.

The correct has-gripper comparison showed SDK and Python inverse dynamics are
very close at the forward pose:

```text
SDK    ~= [0, -6.6688, -7.4410, -2.1574, 0.00007, -0.00185]
Python ~= [0, -6.6752, -7.4488, -2.1537, 0.00016, -0.00390]
```

So the primary problem is not that the feedforward model is wildly wrong.

## Current conclusion

The ROS torque bridge is not the main bug and J4 is not obviously sign-flipped.

What is wrong is the control architecture:

1. SDK/Gazebo lowcmd applies PD inside the Unitree joint controller at the
   simulator/controller update rate.
2. `torque_main.py --test-controller augmented_pd` applies PD externally through
   a file bridge and ROS topics, with measured feedback arriving only around
   40-55 Hz in the bad logs.
3. J4 then hits high velocity and feedback saturation, so the derivative term
   alternates signs and the joint enters a limit cycle.

Pure feedforward torque replay cannot match SDK motion because it omits the
internal SDK/lowcmd PD contribution. External augmented PD can in principle
move the joint, but only if it runs fast enough and with gains/limits stable for
the sampled feedback loop.

## Recommended next steps

1. For SDK-equivalence in Gazebo, use `replay_sdk_lowcmd_gazebo.py` or a similar
   path that publishes `q_cmd`, `dq_cmd`, `tau_cmd`, `Kp`, and `Kd` directly to
   `/z1_gazebo/JointXX_controller/command`.
2. If the project requires pure torque control, move the control loop into a
   C++ Gazebo/ROS controller running at the Gazebo control update rate. Do not
   rely on the Python file bridge for high-gain PD.
3. If continuing with Python/file pure torque for diagnostics, lower J4 gains
   and torque limits a lot, run slower trajectories, and log actual loop rate.
4. To fully rule out sign/topic mapping, run a safe J4 torque-pulse test from a
   reset pose:
   - publish only small positive J4 torque for a short time;
   - publish zero;
   - publish only small negative J4 torque;
   - check whether `q4` acceleration sign follows command sign.
   Existing logs already suggest positive torque gives positive acceleration
   and negative torque gives negative acceleration.

## Rate follow-up on 2026-05-27

The 40-55 Hz rate was consistent with the Python dynamics path:

```text
old dynamics_analytic(q,dq) ~= 19 ms
old computed torque call    ~= 15 ms
```

The project already had the analytic Lie-bracket `dM` method from
`reference/z1_analytic_dM_project.zip`; it was not doing runtime finite
difference or symbolic differentiation. Most of the cost was Python/Numpy
overhead while building `dM` and `C`.

Implemented changes:

1. Added `z1_analytic_dynamics_fast.pyx`, a Cython implementation of the same
   product-of-exponentials / Lie-bracket analytic `M(q)` and `dM/dq` equations.
   It still computes `M`, `C`, `N`, and `dM` explicitly; it does not call SDK
   inverse dynamics and does not finite-difference `M`.
2. Added `setup_fast_dynamics.py` for building the compiled module:

   ```bash
   python3 setup_fast_dynamics.py build_ext --inplace
   ```

3. Updated `z1_analytic_dynamics.py` to use the compiled module when available
   and fall back to the pure Python reference implementation otherwise. Use
   `Z1_DISABLE_FAST_DYNAMICS=1` to force the pure Python reference path.
4. Replaced expensive `np.cross` calls in the pure-Python analytic Lie-bracket
   hot path with direct scalar arithmetic.
5. Removed Python-side `fsync()` from the file IPC atomic writes. On `/tmp`,
   `FileRobotIO.send_torque()` dropped from about `0.94 ms` to `0.25 ms`.

Measured local timings after the change:

```text
mass_and_dM_analytic(q) with Cython fast path ~= 21 us
dynamics_analytic(q,dq) with Cython fast path ~= 30 us
compute_tau(...) with Cython fast path        ~= 72 us
pure-Python dynamics_analytic(q,dq) fallback  ~= 9.5 ms
FileRobotIO.send_torque()                     ~= 253 us
```

Caveat: `torque_main.py` still waits for a newer sensor timestamp each
iteration. If the ROS/Gazebo state source only updates at 40-55 Hz, the loop
will still run at that feedback rate even though torque computation is now
fast. The next bottleneck to check is therefore the bridge/state publication
rate.

## Useful commands

Analyze a log:

```bash
python3 analyze_gain_logs.py logs/augpd_forward_controllerstate_safe.csv
```

Start bridge with per-controller feedback:

```bash
python3 ros_torque_bridge.py --state-source controller_states
```

Replay SDK-style lowcmd to Gazebo:

```bash
python3 replay_sdk_lowcmd_gazebo.py logs/sdk_forward_tau_source_offline.csv \
  --csv-log logs/sdk_lowcmd_replay_next.csv \
  --state-source controller_states
```

Compare SDK/Python inverse dynamics at forward pose:

```bash
python3 compare_sdk_dynamics.py \
  --q "0 1.5 -1 -0.54 0 0" \
  --dq "0 0 0 0 0 0" \
  --ddq "0 0 0 0 0 0"
```

## Caution

Some CSV files were overwritten during earlier experiments. In particular,
do not assume `logs/sdk_forward_tau.csv` is always a clean source trace unless
its columns are checked first. Prefer clean source logs with explicit
`q_cmd_*`, `dq_cmd_*`, `ddq_cmd_*`, and `tau_*` columns.
