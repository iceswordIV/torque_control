#!/usr/bin/env bash
set -euo pipefail

mkdir -p logs/eval_joint_workspace_computed_pid_friction

# Computed-torque PID + friction gains
KP="20 20 40 60 5 5"
KD="3 3 6 14 0.6 0.4"
KI="0 0 0 20 0 0"

INTEGRAL_LIMIT="0.8 0.8 0.8 0.8 0.8 0.8"

DAMPING="1 2 1 1 1 1"
FRICTION="1 2 1 1.5 1 1"

TAU_LIMIT="20 40 25 12 10 10"

COMMON=(
  --trajectory-profile scurve
  --move-time 15
  --hold-time 5
  --return-to-start
  --return-time 15
  --duration 35
  --test-controller computed_pid_friction_model
  --return-controller computed_pid_friction_model
  --kp "$KP"
  --kd "$KD"
  --return-kp "$KP"
  --return-kd "$KD"
  --ki "$KI"
  --integral-limit "$INTEGRAL_LIMIT"
  --model-damping "$DAMPING"
  --model-friction "$FRICTION"
  --tau-limit "$TAU_LIMIT"
  --dynamics-mode analytic
)

prehome() {
  local label="$1"

  echo
  echo "------------------------------------------------------------"
  echo "Pre-home after $label"
  echo "------------------------------------------------------------"

  python3 torque_main.py \
    --mode full_pose_absolute \
    --target "0 0 -0.005 -0.074 0 0" \
    --trajectory-profile scurve \
    --move-time 8 \
    --hold-time 3 \
    --no-return-home \
    --duration 11 \
    --test-controller augmented_pd_friction_model \
    --kp "20 20 40 15 5 5" \
    --kd "3 3 6 2.5 0.6 0.4" \
    --model-damping "$DAMPING" \
    --model-friction "$FRICTION" \
    --tau-limit "$TAU_LIMIT" \
    --dynamics-mode analytic \
    --csv-log "logs/eval_joint_workspace_computed_pid_friction/prehome_${label}.csv"

  sleep 1
}

run_one() {
  local joint="$1"
  local angle="$2"

  if [[ "$angle" == -* ]]; then
    angle_label="neg${angle#-}deg"
  else
    angle_label="pos${angle}deg"
  fi

  local test_name="workspace_j${joint}_${angle_label}"
  local csv_path="logs/eval_joint_workspace_computed_pid_friction/${test_name}_computed_pid_friction.csv"

  echo
  echo "============================================================"
  echo "Running computed_pid_friction_model | joint=$joint | angle=$angle deg"
  echo "CSV: $csv_path"
  echo "============================================================"

  python3 torque_main.py \
    --mode one_joint_relative \
    --joint "$joint" \
    --angle-deg "$angle" \
    "${COMMON[@]}" \
    --csv-log "$csv_path"

  prehome "$test_name"

  sleep 1
}

run_joint_angle_set() {
  local joint="$1"
  shift
  local angles=("$@")

  for angle in "${angles[@]}"
  do
    run_one "$joint" "$angle"
  done
}

# Same workspace set as before
run_joint_angle_set 1 -30 -10 -5 5 10 30
run_joint_angle_set 2 5 10 30
run_joint_angle_set 3 -5 -10 -30
run_joint_angle_set 4 -5 -10 -30 5 10 30
run_joint_angle_set 5 -5 -10 -30 5 10 30
run_joint_angle_set 6 -5 -10 -30 5 10 30

echo
echo "============================================================"
echo "Finished computed PID friction joint workspace evaluation."
echo "Logs written to logs/eval_joint_workspace_computed_pid_friction/"
echo "============================================================"
