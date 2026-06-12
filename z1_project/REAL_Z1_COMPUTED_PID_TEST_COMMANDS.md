# Real Z1 Computed-PID Friction Test Commands

This document is for the real Unitree Z1 arm test, not Gazebo.

Goal for limited lab time:

1. Start the real Z1 control chain correctly.
2. Use `computed_pid_friction_model`.
3. Test only J1-J3 with small motions.
4. Stop after J1-J3 and inspect logs.

Do **not** run J1-J6 workspace tests today.

Do **not** run forward-pose tests today.

Important assumptions:

- Real Z1 uses `z1_ctrl`, not `sim_ctrl`.
- Real Z1 bridge uses default SDK UDP ports `8071/8072`.
- Do **not** use `--gazebo-ports` for real hardware.
- Keep gripper enabled. Do **not** use `--no-gripper` unless SDK reports gripper connection errors.
- Use torque limit `"5 8 10 10 3 3"`.
- These tests are safety/communication/sign tests first, not final performance tests.

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

In that fallback case, Python `--tau-limit` is the only clamp, so run only the tiny tests below.

Watch the bridge output. It should print a rate. If the rate is very low or unstable, stop.

---

# 4. Terminal 3: separate first-hand J1-J3 tests

Run these one by one. Do not paste the full sequence until the previous joint is stable.

Shared settings:

```text
controller = computed_pid_friction_model
kp         = "16 16 16 16 5 5"
kd         = "6 6 6 6 2 2"
ki         = "0 0 0 0 0 0"
damping    = "0 0 0 0 0 0"
friction   = "0 0 0 0 0 0"
tau-limit  = "5 8 10 10 3 3"
```

For real first tests, damping/friction are zero because the previous friction values were Gazebo compensation values.

---

## 4.1 J1 +2 deg first

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

Check:

- J1 direction is correct.
- No vibration.
- No sagging or unexpected motion.
- `effective loop rate` is reasonable.
- Ctrl+C or stop file stops safely.

---

## 4.2 J1 +5 deg second

Only run this if J1 +2 deg is good.

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

---

## 4.3 J2 +2 deg first

Only run this after J1 is good.

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

---

## 4.4 J2 +5 deg second

Only run this if J2 +2 deg is good.

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

---

## 4.5 J3 -2 deg first

Only run this after J1 and J2 are good.

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

---

## 4.6 J3 -5 deg second

Only run this if J3 -2 deg is good.

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

# 5. Optional J1-J3 sequence script

Only create and use this after the separate first-hand tests above are stable.

```bash
cd /home/icesword/Desktop/torque_control/z1_project

cat > run_real_j1_j2_j3_computed_pid_small.sh <<'SH'
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
  echo "REAL Z1 computed_pid_friction_model test"
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
  sleep 2
}

echo "Real Z1 J1-J3 small computed-PID/friction sequence."
echo "Use z1_ctrl, not sim_ctrl."
echo "Use pure_torque_bridge without --gazebo-ports."
echo "Gripper should remain enabled."
echo "Emergency stop from another terminal:"
echo "  touch /tmp/z1_torque_\$(id -u)/z1_stop.txt"
echo

run_one 1  2  real_j1_pos2_sequence 8  8  25
run_one 1  5  real_j1_pos5_sequence 8  8  25
run_one 2  2  real_j2_pos2_sequence 10 10 28
run_one 2  5  real_j2_pos5_sequence 10 10 28
run_one 3 -2  real_j3_neg2_sequence 10 10 28
run_one 3 -5  real_j3_neg5_sequence 10 10 28

echo
echo "All J1-J3 small real tests completed. Stop here and inspect logs."
SH

chmod +x run_real_j1_j2_j3_computed_pid_small.sh
```

Run it:

```bash
cd /home/icesword/Desktop/torque_control/z1_project
bash run_real_j1_j2_j3_computed_pid_small.sh
```

---

# 6. Stop rules

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

After J1-J3, stop and inspect logs. Do not run J1-J6 or forward-pose tests in limited lab time.
