# Real Z1 Computed-PID Friction Test Commands

This document is for the real Unitree Z1 arm test, not Gazebo.

Today's controller is the studied controller:

```text
TEST CONTROLLER   = computed_pid_friction_model
RETURN CONTROLLER = computed_pid_friction_model
MOVE TIME         = 5 s
HOLD TIME         = 3 s
RETURN TIME       = 5 s
TOTAL DURATION    = 15 s
FRICTION          = "1 2.5 1 1.5 1 1.5"
TAU LIMIT         = "5 12 12 10 3 3"
```

Important: the return controller is **not hidden**. It is this command line:

```bash
--return-controller computed_pid_friction_model
```

The return time is this command line:

```bash
--return-time 5
```

For flexible helpers, `return_time` is the **5th numeric argument** after target:

```bash
run_pose LABEL "q1 q2 q3 q4 q5 q6" MOVE_TIME HOLD_TIME RETURN_TIME DURATION
```

Example:

```bash
run_pose j2j3_90_cpid_fric2p5_5move_3hold_5return "0 1.5708 -1.5708 -0.074 0 0" 5 3 5 15
```

means:

```text
move_time   = 5
hold_time   = 3
return_time = 5
duration    = 15
```

---

# 0. Emergency stop

Keep this ready in a separate terminal:

```bash
touch /tmp/z1_torque_$(id -u)/z1_stop.txt
```

Also be ready to press `Ctrl+C` in the Python terminal and bridge terminal.

---

# 1. Start real robot chain

## Terminal 0: check robot network

```bash
ping -c 3 192.168.123.110
```

## Terminal 1: start real Unitree controller

Use `z1_ctrl`, not `sim_ctrl`.

```bash
cd /home/administrator/z1_controller/build
./z1_ctrl
```

If your local user is `icesword`, use:

```bash
cd /home/icesword/Desktop/z1_controller/build
./z1_ctrl
```

## Terminal 2: start pure torque bridge

First set the SDK library path. This fixes:

```text
libZ1_SDK_x86_64.so: cannot open shared object file
```

For administrator machine:

```bash
export LD_LIBRARY_PATH=/home/administrator/torque_control/z1_sdk/lib:$LD_LIBRARY_PATH
cd /home/administrator/torque_control/z1_project/cpp/build
```

For icesword machine:

```bash
export LD_LIBRARY_PATH=/home/icesword/Desktop/torque_control/z1_sdk/lib:$LD_LIBRARY_PATH
cd /home/icesword/Desktop/torque_control/z1_project/cpp/build
```

Then start bridge:

```bash
./pure_torque_bridge \
  --dt 0.002 \
  --tau-limit "5 12 12 10 3 3" \
  --max-command-age-ms 200
```

Do **not** add `--gazebo-ports` for real hardware.

Do **not** add `--no-gripper` unless SDK reports gripper connection errors.

If the bridge says `unknown argument: --tau-limit` or `unknown argument: --max-command-age-ms`, use fallback:

```bash
./pure_torque_bridge --dt 0.002
```

In fallback mode, Python `--tau-limit` is the only torque clamp.

---

# 2. Close visualizers before final logged test

Visualization can reduce loop rate. For final logged tests:

```bash
killall -9 rviz gzclient gzserver gazebo 2>/dev/null
```

---

# 3. Paste helpers once in Terminal 3

Use the path that exists on the current computer.

Administrator machine:

```bash
cd /home/administrator/torque_control/z1_project
```

Icesword machine:

```bash
cd /home/icesword/Desktop/torque_control/z1_project
```

Then paste these helpers:

```bash
prehome() {
  local label="${1:-manual}"
  local log="logs/real_prehome_${label}_$(date +%H%M%S).csv"

  echo
  echo "============================================================"
  echo "REAL Z1 PREHOME / RESET"
  echo "controller = augmented_pd_friction_model"
  echo "target = 0 0 -0.005 -0.074 0 0"
  echo "move_time=12, hold_time=3, no return, duration=16"
  echo "friction=1 2.5 1 1.5 1 1.5"
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
  echo "move_time=${move_time}"
  echo "hold_time=${hold_time}"
  echo "return_time=${return_time}"
  echo "duration=${duration}"
  echo "test_controller=computed_pid_friction_model"
  echo "return_controller=computed_pid_friction_model"
  echo "friction=1 2.5 1 1.5 1 1.5"
  echo "tau_limit=5 12 12 10 3 3"
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
  echo "REAL Z1 CPID SINGLE-JOINT TEST WITH RETURN"
  echo "joint=${joint}, angle=${angle} deg"
  echo "move_time=${move_time}"
  echo "hold_time=${hold_time}"
  echo "return_time=${return_time}"
  echo "duration=${duration}"
  echo "test_controller=computed_pid_friction_model"
  echo "return_controller=computed_pid_friction_model"
  echo "friction=1 2.5 1 1.5 1 1.5"
  echo "tau_limit=5 12 12 10 3 3"
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
```

---

# 4. Run order today

## 4.1 Reset/prehome

```bash
prehome start
```

## 4.2 Optional J2 +30 sanity check

This confirms J2 friction `2.5` still solves the return issue before testing 90 deg.

```bash
run_joint 2 30 j2_pos30_cpid_fric2p5_5move_3hold_5return 5 3 5 15
python3 plot_log.py logs/real_j2_pos30_cpid_fric2p5_5move_3hold_5return.csv
```

## 4.3 Main J2/J3 90 deg CPID test

This is the professor-goal style test:

```bash
run_pose j2j3_90_cpid_fric2p5_5move_3hold_5return "0 1.5708 -1.5708 -0.074 0 0" 5 3 5 15
python3 plot_log.py logs/real_j2j3_90_cpid_fric2p5_5move_3hold_5return.csv
```

This command expands to the explicit direct command below.

---

# 5. Direct command for main J2/J3 90 deg CPID test

Use this if you do not want helpers.

```bash
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
  --csv-log logs/real_j2j3_90_cpid_fric2p5_5move_3hold_5return.csv
```

---

# 6. Forward-pose examples with return

These are optional after J2/J3 90 deg works.

Format:

```bash
run_pose LABEL "q1 q2 q3 q4 q5 q6" MOVE_TIME HOLD_TIME RETURN_TIME DURATION
```

Examples:

```bash
run_pose forward_25pct_cpid_fric2p5  "0 0.375 -0.25 -0.135 0 0" 20 3 20 48
run_pose forward_50pct_cpid_fric2p5  "0 0.750 -0.50 -0.270 0 0" 20 3 20 48
run_pose forward_75pct_cpid_fric2p5  "0 1.125 -0.75 -0.405 0 0" 20 3 20 48
run_pose forward_100pct_cpid_fric2p5 "0 1.500 -1.00 -0.540 0 0" 25 3 25 58
```

---

# 7. Latest good result to compare against

The good real-arm CPID result with J2 friction `2.5` was approximately:

```text
Target                = "0 1.5708 -1.5708 -0.074 0 0"
move_time             = 5 s
hold_time             = 3 s
return_time           = 5 s
return_controller     = computed_pid_friction_model
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
