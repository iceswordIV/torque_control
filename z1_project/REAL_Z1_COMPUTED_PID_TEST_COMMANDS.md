# Real Z1 Computed-PID Friction Test Commands

This document is for the real Unitree Z1 arm test, not Gazebo.

Goal for lab time:

1. Start the real Z1 control chain correctly.
2. Use `computed_pid_friction_model` as the studied controller.
3. Use `augmented_pd_friction_model` only for prehome/reset.
4. Keep flexible helpers so any single joint or any full pose can be tested by hand.
5. Use the real-arm tuned J2 friction value `2.5` for CPID tests.

Important current tuning:

```text
CPID KP       = "64 100 100 60 64 100"
CPID KD       = "13 16 16 14 13 16"
CPID KI       = "0 0 0 20 0 0"
DAMPING       = "1 2 1 1 1 1"
FRICTION      = "1 2.5 1 1.5 1 1.5"
TAU           = "5 12 12 10 3 3"
PREHOME       = "0 0 -0.005 -0.074 0 0"
```

Why J2 friction is `2.5`:

- J2 friction `2.0` was too small and J2 got stuck during return.
- J2 friction `3.0` was too aggressive / too large for torque output.
- J2 friction `2.5` worked well for the real-arm CPID J2/J3 motion.

Do **not** test J2 +90 deg alone. It can hit the ground. For 90 deg J2 motion, use coordinated J2/J3:

```text
J2 = +90 deg, J3 = -90 deg
Target = "0 1.5708 -1.5708 -0.074 0 0"
```

---

## 0. Emergency stop command

Keep this ready in a separate terminal:

```bash
touch /tmp/z1_torque_$(id -u)/z1_stop.txt
```

Also be ready to press `Ctrl+C` in the Python terminal and bridge terminal.

---

## 1. Terminal 0: check real robot network

The host must be on the robot subnet `192.168.123.x`.

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

## 4. Close visualizers before final logged test

Visualization can reduce loop rate. For final logged tests, close RViz/Gazebo/Gazebo client first:

```bash
killall -9 rviz gzclient gzserver gazebo 2>/dev/null
```

Then run only the controller terminals.

---

# Part A: paste flexible helpers once in Terminal 3

These helpers are the main part. They give freedom to test any single joint, any full pose with return, and any full pose without return.

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

run_joint() {
  local joint="$1"
  local angle="$2"
  local label="${3:-j${joint}_${angle}deg_cpid_fric2p5}"
  local move_time="${4:-5}"
  local hold_time="${5:-3}"
  local return_time="${6:-5}"
  local duration="${7:-15}"
  local log="logs/real_${label}.csv"

  echo
  echo "============================================================"
  echo "REAL Z1 CPID SINGLE-JOINT TEST"
  echo "joint=${joint}, angle=${angle} deg"
  echo "move_time=${move_time}, hold_time=${hold_time}, return_time=${return_time}, duration=${duration}"
  echo "controller=computed_pid_friction_model"
  echo "friction=1 2.5 1 1.5 1 1.5"
  echo "log=${log}"
  echo "STOP if wrong direction, vibration, contact, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to start this single-joint test, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode one_joint_relative \
    --joint "$joint" \
    --angle-deg "$angle" \
    --trajectory-profile scurve \
    --move-time "$move_time" \
    --hold-time "$hold_time" \
    --return-to-start \
    --return-time "$return_time" \
    --duration "$duration" \
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
    --csv-log "$log"
}

run_pose() {
  local label="$1"
  local target="$2"
  local move_time="${3:-5}"
  local hold_time="${4:-3}"
  local return_time="${5:-5}"
  local duration="${6:-15}"
  local log="logs/real_${label}.csv"

  echo
  echo "============================================================"
  echo "REAL Z1 CPID FULL-POSE TEST WITH RETURN"
  echo "target=${target}"
  echo "move_time=${move_time}, hold_time=${hold_time}, return_time=${return_time}, duration=${duration}"
  echo "controller=computed_pid_friction_model"
  echo "friction=1 2.5 1 1.5 1 1.5"
  echo "log=${log}"
  echo "STOP if wrong direction, vibration, contact, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to start this pose test, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode full_pose_absolute \
    --target "$target" \
    --trajectory-profile scurve \
    --move-time "$move_time" \
    --hold-time "$hold_time" \
    --return-to-start \
    --return-time "$return_time" \
    --duration "$duration" \
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
    --csv-log "$log"
}

move_pose() {
  local label="$1"
  local target="$2"
  local move_time="${3:-8}"
  local hold_time="${4:-5}"
  local duration="${5:-15}"
  local log="logs/real_${label}.csv"

  echo
  echo "============================================================"
  echo "REAL Z1 CPID MOVE TO POSE, NO RETURN"
  echo "target=${target}"
  echo "move_time=${move_time}, hold_time=${hold_time}, duration=${duration}"
  echo "controller=computed_pid_friction_model"
  echo "friction=1 2.5 1 1.5 1 1.5"
  echo "log=${log}"
  echo "STOP if wrong direction, vibration, contact, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to move to this pose, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode full_pose_absolute \
    --target "$target" \
    --trajectory-profile scurve \
    --move-time "$move_time" \
    --hold-time "$hold_time" \
    --no-return-home \
    --duration "$duration" \
    --test-controller computed_pid_friction_model \
    --kp "64 100 100 60 64 100" \
    --kd "13 16 16 14 13 16" \
    --ki "0 0 0 20 0 0" \
    --integral-limit "0.8 0.8 0.8 0.8 0.8 0.8" \
    --model-damping "1 2 1 1 1 1" \
    --model-friction "1 2.5 1 1.5 1 1.5" \
    --tau-limit "5 12 12 10 3 3" \
    --dynamics-mode analytic \
    --csv-log "$log"
}

plot_log_file() {
  local csv="$1"
  python3 plot_log.py "$csv"
  echo "plots written to logs/plots"
}
```

---

# Part B: how to use the helpers

## B1. Reset/prehome

```bash
prehome start
```

Run prehome again only after bad motion or before an important new group:

```bash
prehome after_bad_motion
prehome before_forward_pose
```

## B2. Single-joint tests

Format:

```bash
run_joint JOINT ANGLE_DEG LABEL MOVE_TIME HOLD_TIME RETURN_TIME DURATION
```

Examples:

```bash
run_joint 1 30  j1_pos30_cpid_fric2p5 5 3 5 15
run_joint 2 30  j2_pos30_cpid_fric2p5 5 3 5 15
run_joint 3 -30 j3_neg30_cpid_fric2p5 5 3 5 15
run_joint 4 -30 j4_neg30_cpid_fric2p5 5 3 5 15
run_joint 5 30  j5_pos30_cpid_fric2p5 5 3 5 15
run_joint 6 30  j6_pos30_cpid_fric2p5 5 3 5 15
```

Plot example:

```bash
python3 plot_log.py logs/real_j2_pos30_cpid_fric2p5.csv
```

Do **not** run J2 +90 as a single-joint test. Use the J2/J3 pose test below.

## B3. Arbitrary full-pose test with return

Format:

```bash
run_pose LABEL "q1 q2 q3 q4 q5 q6" MOVE_TIME HOLD_TIME RETURN_TIME DURATION
```

Main J2/J3 90 deg test:

```bash
run_pose j2j3_90_cpid_fric2p5_5move_3hold_5return "0 1.5708 -1.5708 -0.074 0 0" 5 3 5 15
python3 plot_log.py logs/real_j2j3_90_cpid_fric2p5_5move_3hold_5return.csv
```

Forward-pose examples:

```bash
run_pose forward_25pct_cpid_fric2p5  "0 0.375 -0.25 -0.135 0 0" 20 3 20 48
run_pose forward_50pct_cpid_fric2p5  "0 0.750 -0.50 -0.270 0 0" 20 3 20 48
run_pose forward_75pct_cpid_fric2p5  "0 1.125 -0.75 -0.405 0 0" 20 3 20 48
run_pose forward_100pct_cpid_fric2p5 "0 1.500 -1.00 -0.540 0 0" 25 3 25 58
```

## B4. Move to a chosen pose without return

Use this when you want to move to a certain pose and hold there, not immediately return.

Format:

```bash
move_pose LABEL "q1 q2 q3 q4 q5 q6" MOVE_TIME HOLD_TIME DURATION
```

Examples:

```bash
move_pose check_pose_1 "0 0.4 0 -0.074 0 0" 8 5 15
move_pose check_j2j3_shape "0 1.5708 -1.5708 -0.074 0 0" 8 5 15
```

---

# Part C: latest good result to compare against

The good real-arm CPID result with J2 friction 2.5 was approximately:

```text
Target                = "0 1.5708 -1.5708 -0.074 0 0"
J2 max tracking error = 0.087 rad = 5.0 deg
J2 final error        = 0.053 rad = 3.0 deg
J3 max tracking error = 0.055 rad = 3.2 deg
J3 final error        = 0.0008 rad
max tau2              = 11.05 Nm under 12 Nm limit
max tau3              = 10.07 Nm under 12 Nm limit
loop rate             = 349 Hz
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
