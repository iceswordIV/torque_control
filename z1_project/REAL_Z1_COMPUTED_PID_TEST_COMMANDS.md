# Real Z1 Computed-PID Friction Test Commands

This document is for the real Unitree Z1 arm test, not Gazebo.

Goal for limited lab time:

1. Start the real Z1 control chain correctly.
2. Use `computed_pid_friction_model`.
3. Test J1-J6 with small single-joint motions.
4. If single-joint tests are stable, test forward pose manually from 25% to 100%.

Important assumptions:

- Real Z1 uses `z1_ctrl`, not `sim_ctrl`.
- Real Z1 bridge uses default SDK UDP ports `8071/8072`.
- Do **not** use `--gazebo-ports` for real hardware.
- Keep gripper enabled. Do **not** use `--no-gripper` unless SDK reports gripper connection errors.
- Use torque limit `"5 8 10 10 3 3"`.
- Start with conservative gains and zero Gazebo friction compensation.
- These tests are real-hardware bring-up tests, not final performance tuning.

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

In that fallback case, Python `--tau-limit` is the only clamp, so avoid large motions.

Watch the bridge output. It should print a rate. Around 300-500 Hz is acceptable. If the rate is very low or unstable, stop.

---

## 4. Shared Python settings

Use these for the first real tests:

```text
controller = computed_pid_friction_model
kp         = "16 16 16 16 5 5"
kd         = "6 6 6 6 2 2"
ki         = "0 0 0 0 0 0"
damping    = "0 0 0 0 0 0"
friction   = "0 0 0 0 0 0"
tau-limit  = "5 8 10 10 3 3"
```

For real first tests, damping/friction are zero because previous friction values were Gazebo compensation values.

---

# Part A: first-hand J1-J3 tests

Run these first, one by one.

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
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j1_pos2_first.csv
```

## A2. J1 +5 deg

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode one_joint_relative \
  --joint 1 \
  --angle-deg 5 \
  --trajectory-profile scurve \
  --move-time 8 \
  --hold-time 3 \
  --return-to-start \
  --return-time 8 \
  --duration 25 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j1_pos5_second.csv
```

## A3. J2 +2 deg

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
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j2_pos2_first.csv
```

## A4. J2 +5 deg

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode one_joint_relative \
  --joint 2 \
  --angle-deg 5 \
  --trajectory-profile scurve \
  --move-time 10 \
  --hold-time 3 \
  --return-to-start \
  --return-time 10 \
  --duration 28 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j2_pos5_second.csv
```

## A5. J3 -2 deg

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
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j3_neg2_first.csv
```

## A6. J3 -5 deg

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode one_joint_relative \
  --joint 3 \
  --angle-deg -5 \
  --trajectory-profile scurve \
  --move-time 10 \
  --hold-time 3 \
  --return-to-start \
  --return-time 10 \
  --duration 28 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_j3_neg5_second.csv
```

---

# Part B: fast J1-J6 single-joint check

Run this only if J1-J3 are stable. This checks one small command per joint.

It excludes positive J4 because positive J4 was already a known bad direction in simulation and SDK comparison. Use J4 negative only.

```bash
cd /home/icesword/Desktop/torque_control/z1_project

cat > run_real_j1_to_j6_computed_pid_quick.sh <<'SH'
#!/usr/bin/env bash
set -euo pipefail

TAU_LIMIT="5 8 10 10 3 3"
KP="16 16 16 16 5 5"
KD="6 6 6 6 2 2"
KI="0 0 0 0 0 0"
ZERO6="0 0 0 0 0 0"

mkdir -p logs

run_one() {
  local joint="$1"
  local angle="$2"
  local label="$3"
  local move_time="$4"
  local return_time="$5"
  local duration="$6"

  echo
  echo "============================================================"
  echo "REAL Z1 quick single-joint check"
  echo "joint=${joint}, angle=${angle} deg"
  echo "log=logs/${label}.csv"
  echo "tau-limit=${TAU_LIMIT}"
  echo "STOP if wrong direction, vibration, sagging, or unexpected motion."
  echo "============================================================"
  read -p "Press Enter to start this test, or Ctrl+C to abort..."

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
    --kp "$KP" \
    --kd "$KD" \
    --ki "$KI" \
    --model-damping "$ZERO6" \
    --model-friction "$ZERO6" \
    --tau-limit "$TAU_LIMIT" \
    --dynamics-mode analytic \
    --csv-log "logs/${label}.csv"

  echo "Finished ${label}"
  sleep 1
}

echo "Real Z1 quick J1-J6 computed-PID/friction check."
echo "Use z1_ctrl, not sim_ctrl."
echo "Use pure_torque_bridge without --gazebo-ports."
echo "Gripper should remain enabled."
echo "Emergency stop: touch /tmp/z1_torque_\$(id -u)/z1_stop.txt"
echo

run_one 1  5  real_j1_pos5_quick 8  8  25
run_one 2  5  real_j2_pos5_quick 10 10 28
run_one 3 -5  real_j3_neg5_quick 10 10 28
run_one 4 -2  real_j4_neg2_quick 10 10 28
run_one 5  5  real_j5_pos5_quick 8  8  25
run_one 6  5  real_j6_pos5_quick 8  8  25

echo
echo "Quick J1-J6 real check completed."
SH

chmod +x run_real_j1_to_j6_computed_pid_quick.sh
bash run_real_j1_to_j6_computed_pid_quick.sh
```

---

# Part C: manual forward-pose tests from 25% to 100%

Only run this if the J1-J6 quick check is stable.

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

## C1. Forward pose 25%

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode full_pose_absolute \
  --target "0 0.375 -0.25 -0.135 0 0" \
  --trajectory-profile scurve \
  --move-time 20 \
  --hold-time 3 \
  --return-to-start \
  --return-time 20 \
  --duration 48 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_forward_pose_25pct_computed_pid.csv
```

## C2. Forward pose 50%

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode full_pose_absolute \
  --target "0 0.75 -0.50 -0.270 0 0" \
  --trajectory-profile scurve \
  --move-time 20 \
  --hold-time 3 \
  --return-to-start \
  --return-time 20 \
  --duration 48 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_forward_pose_50pct_computed_pid.csv
```

## C3. Forward pose 75%

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode full_pose_absolute \
  --target "0 1.125 -0.75 -0.405 0 0" \
  --trajectory-profile scurve \
  --move-time 20 \
  --hold-time 3 \
  --return-to-start \
  --return-time 20 \
  --duration 48 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_forward_pose_75pct_computed_pid.csv
```

## C4. Forward pose 100%

Only run this if 25%, 50%, and 75% are stable.

```bash
cd /home/icesword/Desktop/torque_control/z1_project

python3 torque_main.py \
  --mode full_pose_absolute \
  --target "0 1.5 -1.0 -0.54 0 0" \
  --trajectory-profile scurve \
  --move-time 25 \
  --hold-time 3 \
  --return-to-start \
  --return-time 25 \
  --duration 58 \
  --test-controller computed_pid_friction_model \
  --return-controller computed_pid_friction_model \
  --kp "16 16 16 16 5 5" \
  --kd "6 6 6 6 2 2" \
  --ki "0 0 0 0 0 0" \
  --model-damping "0 0 0 0 0 0" \
  --model-friction "0 0 0 0 0 0" \
  --tau-limit "5 8 10 10 3 3" \
  --dynamics-mode analytic \
  --csv-log logs/real_forward_pose_100pct_computed_pid.csv
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
