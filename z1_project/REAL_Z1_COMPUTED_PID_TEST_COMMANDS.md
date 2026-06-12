# Real Z1 Computed-PID Friction Test Commands

This document is for the real Unitree Z1 arm test, not Gazebo.

Goal for limited lab time:

1. Start the real Z1 control chain correctly.
2. Use `computed_pid_friction_model`.
3. Use the Gazebo-tuned parameters that worked well yesterday.
4. Manually test J1-J6 joint motions at 5, 10, and 30 deg.
5. If joint tests are stable, manually test forward pose from 25% to 100%.

Important assumptions:

- Real Z1 uses `z1_ctrl`, not `sim_ctrl`.
- Real Z1 bridge uses default SDK UDP ports `8071/8072`.
- Do **not** use `--gazebo-ports` for real hardware.
- Keep gripper enabled. Do **not** use `--no-gripper` unless SDK reports gripper connection errors.
- Use bridge/Python torque limit `"5 8 10 10 3 3"`.
- These commands now use the stronger Gazebo-tuned computed-PID/friction parameters.

Gazebo-tuned parameters used here:

```text
KP       = "64 100 100 60 64 100"
KD       = "13 16 16 14 13 16"
KI       = "0 0 0 20 0 0"
DAMPING  = "1 2 1 1 1 1"
FRICTION = "1 2 1 1.5 1 1.5"
TAU      = "5 8 10 10 3 3"
```

Notes:

- `KI4 = 20` is kept because yesterday J4 negative and forward-pose behavior used the tuned controller style.
- `FRICTION` and `DAMPING` are the Gazebo-tuned compensation values. They may not be physically exact for the real arm, but these are the parameters that worked in the simulation tests.
- Positive J4 was the known difficult direction. Test J4 negative first. Test J4 positive only if time and safety allow.

---

## 0. Emergency stop command

Keep this ready in a separate terminal:

```bash
touch /tmp/z1_torque_$(id -u)/z1_stop.txt
```

Also be ready to press `Ctrl+C` in the Python terminal and the bridge terminal.

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
  --tau-limit "5 8 10 10 3 3" \
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

Watch the bridge output. It should print a rate. Around 300-500 Hz is acceptable. If the rate is very low or unstable, stop.

---

# Part A: quick sign/safety verification

Run these first, one by one. They use the same tuned parameters, but very small motion.

## A1. J1 +2 deg

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode one_joint_relative \
  --joint 1 \
  --angle-deg 2 \
  --trajectory-profile scurve \
  --move-time 8 \
  --hold-time 3 \
  --return-to-start \
  --return-time 8 \
  --duration 25 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "64 100 100 60 64 100" \
  --kd "13 16 16 14 13 16" \
  --ki "0 0 0 20 0 0" \
  --integral-limit "0.8 0.8 0.8 0.8 0.8 0.8" \
  --model-damping "1 2 1 1 1 1" \
  --model-friction "1 2 1 1.5 1 1.5" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j1_pos2_tuned_first.csv
```

## A2. J2 +2 deg

Only run this after J1 is correct.

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode one_joint_relative \
  --joint 2 \
  --angle-deg 2 \
  --trajectory-profile scurve \
  --move-time 10 \
  --hold-time 3 \
  --return-to-start \
  --return-time 10 \
  --duration 28 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "64 100 100 60 64 100" \
  --kd "13 16 16 14 13 16" \
  --ki "0 0 0 20 0 0" \
  --integral-limit "0.8 0.8 0.8 0.8 0.8 0.8" \
  --model-damping "1 2 1 1 1 1" \
  --model-friction "1 2 1 1.5 1 1.5" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j2_pos2_tuned_first.csv
```

## A3. J3 -2 deg

Only run this after J1 and J2 are correct.

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode one_joint_relative \
  --joint 3 \
  --angle-deg -2 \
  --trajectory-profile scurve \
  --move-time 10 \
  --hold-time 3 \
  --return-to-start \
  --return-time 10 \
  --duration 28 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "64 100 100 60 64 100" \
  --kd "13 16 16 14 13 16" \
  --ki "0 0 0 20 0 0" \
  --integral-limit "0.8 0.8 0.8 0.8 0.8 0.8" \
  --model-damping "1 2 1 1 1 1" \
  --model-friction "1 2 1 1.5 1 1.5" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j3_neg2_tuned_first.csv
```

---

# Part B: manual J1-J6 joint tests at 5, 10, 30 deg

This part is **by hand**. Paste the helper function once, then run one `run_joint ...` line at a time.

Do not run the entire list blindly. After each line, watch the arm and check the terminal output.

## B1. Paste this helper once in Terminal 3

```bash
cd /home/icesword/Desktop/torque_control/z1_project

run_joint() {
  local joint="$1"
  local angle="$2"

  local abs_angle="${angle#-}"
  local sign_label="pos${abs_angle}"
  if [[ "$angle" == -* ]]; then
    sign_label="neg${abs_angle}"
  fi

  local move_time=10
  local return_time=10
  local duration=26

  if [[ "$abs_angle" == "5" ]]; then
    move_time=8
    return_time=8
    duration=22
  elif [[ "$abs_angle" == "10" ]]; then
    move_time=10
    return_time=10
    duration=26
  elif [[ "$abs_angle" == "30" ]]; then
    move_time=15
    return_time=15
    duration=38
  else
    echo "Only use 5, 10, or 30 deg for this helper."
    return 1
  fi

  local log="logs/real_j${joint}_${sign_label}deg_tuned_computed_pid.csv"

  echo
  echo "============================================================"
  echo "REAL Z1 manual joint test with Gazebo-tuned parameters"
  echo "joint=${joint}, angle=${angle} deg"
  echo "move_time=${move_time}, return_time=${return_time}, duration=${duration}"
  echo "log=${log}"
  echo "STOP if wrong direction, vibration, sagging, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to start this single test, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode one_joint_relative \
    --joint "$joint" \
    --angle-deg "$angle" \
    --trajectory-profile scurve \
    --move-time "$move_time" \
    --hold-time 3 \
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
    --model-friction "1 2 1 1.5 1 1.5" \
    --tau-limit "5 8 10 10 3 3" \
    --dynamics-mode analytic \
    --csv-log "$log"
}
```

## B2. Manual test order

Run one line at a time.

### J1

```bash
run_joint 1 5
run_joint 1 10
run_joint 1 30
run_joint 1 -5
run_joint 1 -10
run_joint 1 -30
```

### J2

Use positive direction first.

```bash
run_joint 2 5
run_joint 2 10
run_joint 2 30
```

### J3

Use negative direction first.

```bash
run_joint 3 -5
run_joint 3 -10
run_joint 3 -30
```

### J4

Use negative direction first. Positive J4 was the known difficult direction in simulation, so do positive J4 only if you still have time and the negative side is stable.

```bash
run_joint 4 -5
run_joint 4 -10
run_joint 4 -30
```

Optional J4 positive last:

```bash
run_joint 4 5
run_joint 4 10
run_joint 4 30
```

### J5

```bash
run_joint 5 5
run_joint 5 10
run_joint 5 30
run_joint 5 -5
run_joint 5 -10
run_joint 5 -30
```

### J6

```bash
run_joint 6 5
run_joint 6 10
run_joint 6 30
run_joint 6 -5
run_joint 6 -10
run_joint 6 -30
```

---

# Part C: manual forward-pose tests from 25% to 100%

Only run this if the manual J1-J6 joint tests are stable.

Full target:

```text
[0, 1.5, -1.0, -0.54, 0, 0]
```

Use these scaled targets:

```text
25%  = [0, 0.375, -0.25, -0.135, 0, 0]
50%  = [0, 0.750, -0.50, -0.270, 0, 0]
75%  = [0, 1.125, -0.75, -0.405, 0, 0]
100% = [0, 1.500, -1.00, -0.540, 0, 0]
```

## C1. Paste this helper once

```bash
cd /home/icesword/Desktop/torque_control/z1_project

run_forward() {
  local scale_label="$1"
  local target="$2"
  local move_time="$3"
  local return_time="$4"
  local duration="$5"
  local log="logs/real_forward_pose_${scale_label}_tuned_computed_pid.csv"

  echo
  echo "============================================================"
  echo "REAL Z1 manual forward-pose test with Gazebo-tuned parameters"
  echo "scale=${scale_label}"
  echo "target=${target}"
  echo "log=${log}"
  echo "STOP if wrong direction, vibration, sagging, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to start this forward-pose test, or Ctrl+C to abort..."

  python3 torque_main.py \
    --mode full_pose_absolute \
    --target "$target" \
    --trajectory-profile scurve \
    --move-time "$move_time" \
    --hold-time 3 \
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
    --model-friction "1 2 1 1.5 1 1.5" \
    --tau-limit "5 8 10 10 3 3" \
    --dynamics-mode analytic \
    --csv-log "$log"
}
```

## C2. Run forward pose by hand

Run one line at a time.

```bash
run_forward 25pct  "0 0.375 -0.25 -0.135 0 0" 20 20 48
run_forward 50pct  "0 0.75 -0.50 -0.270 0 0" 20 20 48
run_forward 75pct  "0 1.125 -0.75 -0.405 0 0" 20 20 48
run_forward 100pct "0 1.5 -1.0 -0.54 0 0" 25 25 58
```

---

# Stop rules

Stop immediately if any of these happen:

- wrong joint moves
- correct joint moves in wrong direction
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
