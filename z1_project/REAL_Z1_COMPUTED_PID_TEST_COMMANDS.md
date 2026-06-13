# Real Z1 Computed-PID Friction Test Commands

This document is for the real Unitree Z1 arm test, not Gazebo.

Today's main goal:

1. Use `computed_pid_friction_model` as the studied controller.
2. Use the tuned real-arm J2 friction value `2.5`.
3. Test the feasible 90 deg motion: J2 +90 deg together with J3 -90 deg.
4. Keep augmented PD only for prehome/reset.

Important result from yesterday:

- J2 +90 deg alone can hit the ground, so it is not a good test.
- Test J2 as a coordinated motion with J3: `J2 = +90 deg`, `J3 = -90 deg`.
- For CPID, J2 friction `2.0` was too small, `3.0` was too large/aggressive, and `2.5` worked well.

Tuned CPID parameters for today's main test:

```text
controller = computed_pid_friction_model
KP         = "64 100 100 60 64 100"
KD         = "13 16 16 14 13 16"
KI         = "0 0 0 20 0 0"
DAMPING    = "1 2 1 1 1 1"
FRICTION   = "1 2.5 1 1.5 1 1.5"
TAU        = "5 12 12 10 3 3"
```

Prehome/reset still uses augmented PD:

```text
controller = augmented_pd_friction_model
KP         = "20 20 40 15 5 5"
KD         = "3 3 6 2.5 0.6 0.4"
DAMPING    = "1 2 1 1 1 1"
FRICTION   = "1 2.5 1 1.5 1 1.5"
TAU        = "5 12 12 10 3 3"
PREHOME    = "0 0 -0.005 -0.074 0 0"
```

---

## 0. Emergency stop command

Keep this ready in a separate terminal:

```bash
touch /tmp/z1_torque_$(id -u)/z1_stop.txt
```

Also be ready to press `Ctrl+C` in the Python terminal and the bridge terminal.

---

## 1. Terminal 0: check real robot network

```bash
ping -c 3 192.168.123.110
```

Only continue if the ping works.

---

## 2. Terminal 1: start real Unitree controller

Use `z1_ctrl`, not `sim_ctrl`.

```bash
cd /home/icesword/Desktop/z1_controller/build
./z1_ctrl
```

Leave this terminal running.

---

## 3. Terminal 2: start pure torque bridge

Real hardware command, gripper enabled:

```bash
cd /home/icesword/Desktop/torque_control/z1_project/cpp/build

./pure_torque_bridge \
  --dt 0.002 \
  --tau-limit "5 12 12 10 3 3" \
  --max-command-age-ms 200
```

Do **not** add `--gazebo-ports`.

Do **not** add `--no-gripper`.

If the bridge prints `unknown argument: --tau-limit` or `unknown argument: --max-command-age-ms`, then this safety update is not compiled yet. Temporary fallback:

```bash
cd /home/icesword/Desktop/torque_control/z1_project/cpp/build
./pure_torque_bridge --dt 0.002
```

In that fallback case, Python `--tau-limit` is the only clamp, so watch the motion carefully.

Watch the bridge output. Around 300-500 Hz is acceptable. If the rate is very low or unstable, stop.

---

## 4. Close visualizers before final test

Visualization can reduce loop rate. For final logged tests, close RViz/Gazebo/Gazebo client first:

```bash
killall -9 rviz gzclient gzserver gazebo 2>/dev/null
```

Then run only the controller terminals.

---

# Part A: paste helpers in Terminal 3

```bash
cd /home/icesword/Desktop/torque_control/z1_project

prehome() {
  local label="${1:-manual}"
  local log="logs/real_prehome_${label}_$(date +%H%M%S).csv"

  echo
  echo "============================================================"
  echo "REAL Z1 PREHOME"
  echo "controller = augmented_pd_friction_model"
  echo "target = 0 0 -0.005 -0.074 0 0"
  echo "log=${log}"
  echo "STOP if wrong direction, vibration, sagging, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to move to prehome, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode full_pose_absolute \
    --target "0 0 -0.005 -0.074 0 0" \
    --trajectory-profile scurve \
    --move-time 12 \
    --hold-time 3 \
    --no-return-home \
    --duration 16 \
    --test-controller augmented_pd_friction_model \
    --kp "20 20 40 15 5 5" \
    --kd "3 3 6 2.5 0.6 0.4" \
    --model-damping "1 2 1 1 1 1" \
    --model-friction "1 2.5 1 1.5 1 1.5" \
    --tau-limit "5 12 12 10 3 3" \
    --dynamics-mode analytic \
    --csv-log "$log"
}

run_cpid_j2j3_90() {
  local label="${1:-real_j2j3_90_cpid_fric2p5}"

  echo
  echo "============================================================"
  echo "REAL Z1 CPID FRICTION TEST: J2 +90 deg, J3 -90 deg"
  echo "controller = computed_pid_friction_model"
  echo "friction = 1 2.5 1 1.5 1 1.5"
  echo "tau-limit = 5 12 12 10 3 3"
  echo "log=logs/${label}.csv"
  echo "STOP if wrong direction, vibration, contact, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to start J2/J3 90 CPID test, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode full_pose_absolute \
    --target "0 1.5708 -1.5708 -0.074 0 0" \
    --trajectory-profile scurve \
    --move-time 5 \
    --hold-time 3 \
    --return-to-start \
    --return-time 5 \
    --duration 15 \
    --test-controller computed_pid_friction_model \
    --return-controller computed_pid_friction_model \
    --kp "64 100 100 60 64 100" \
    --kd "13 16 16 14 13 16" \
    --ki "0 0 0 20 0 0" \
    --integral-limit "0.8 0.8 0.8 0.8 0.8 0.8" \
    --model-damping "1 2 1 1 1 1" \
    --model-friction "1 2.5 1 1.5 1 1.5" \
    --tau-limit "5 12 12 10 3 3" \
    --dynamics-mode analytic \
    --csv-log "logs/${label}.csv"
}

run_cpid_j2_30_check() {
  local label="${1:-real_j2_pos30_cpid_fric2p5_check}"

  echo
  echo "============================================================"
  echo "REAL Z1 CPID SANITY CHECK: J2 +30 deg"
  echo "This is only a quick check before J2/J3 90."
  echo "log=logs/${label}.csv"
  echo "============================================================"
  read -p "Press Enter to start J2 +30 check, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode one_joint_relative \
    --joint 2 \
    --angle-deg 30 \
    --trajectory-profile scurve \
    --move-time 5 \
    --hold-time 3 \
    --return-to-start \
    --return-time 5 \
    --duration 15 \
    --test-controller computed_pid_friction_model \
    --return-controller computed_pid_friction_model \
    --kp "64 100 100 60 64 100" \
    --kd "13 16 16 14 13 16" \
    --ki "0 0 0 20 0 0" \
    --integral-limit "0.8 0.8 0.8 0.8 0.8 0.8" \
    --model-damping "1 2 1 1 1 1" \
    --model-friction "1 2.5 1 1.5 1 1.5" \
    --tau-limit "5 12 12 10 3 3" \
    --dynamics-mode analytic \
    --csv-log "logs/${label}.csv"
}

plot_last() {
  local csv="$1"
  python3 plot_log.py "$csv"
  echo "plots written to logs/plots"
}
```

---

# Part B: recommended run order today

Run prehome first:

```bash
prehome start
```

Optional quick sanity check for J2 +30:

```bash
run_cpid_j2_30_check real_j2_pos30_cpid_fric2p5_check
python3 plot_log.py logs/real_j2_pos30_cpid_fric2p5_check.csv
```

Main professor-goal test, CPID with J2 friction 2.5:

```bash
run_cpid_j2j3_90 real_j2j3_90_cpid_fric2p5_5move_3hold_5return
python3 plot_log.py logs/real_j2j3_90_cpid_fric2p5_5move_3hold_5return.csv
```

If the arm does not return cleanly or you want to reset before another test:

```bash
prehome after_j2j3
```

---

# Part C: success criteria

The latest good result with friction2 = 2.5 had approximately:

```text
J2 max tracking error ≈ 0.087 rad = 5.0 deg
J2 final error        ≈ 0.053 rad = 3.0 deg
J3 max tracking error ≈ 0.055 rad = 3.2 deg
J3 final error        ≈ 0.0008 rad
max tau2              ≈ 11.05 Nm under 12 Nm limit
max tau3              ≈ 10.07 Nm under 12 Nm limit
loop rate             ≈ 349 Hz
```

This is the current best real-arm CPID-friction result for:

```text
J2 +90 deg, J3 -90 deg, 5 s move, 3 s hold, 5 s return
```

---

# Stop rules

Stop immediately if any of these happen:

- wrong joint moves
- correct joint moves in wrong direction
- physical contact or ground contact
- vibration
- arm sagging or falling
- gripper behaves unexpectedly
- bridge rate very low
- Python `effective loop rate` very low
- torque saturates for a long time
- tracking error grows instead of shrinking

Emergency stop command:

```bash
touch /tmp/z1_torque_$(id -u)/z1_stop.txt
```
