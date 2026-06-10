#!/usr/bin/env bash
set -euo pipefail

mkdir -p logs/eval_forward_pose_computed_pid_friction

KP="20 20 40 60 5 5"
KD="3 3 6 14 0.6 0.4"
KI="0 0 0 20 0 0"

INTEGRAL_LIMIT="0.8 0.8 0.8 0.8 0.8 0.8"

DAMPING="1 2 1 1 1 1"
FRICTION="1 2 1 1.5 1 1"

TAU_LIMIT="20 40 25 12 10 10"
TARGET="0 1.5 -1 -0.54 0 0"

COMMON=(
  --mode scaled_pose
  --target "$TARGET"
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

run_forward() {
  local scale="$1"
  local tag="$2"

  local csv_path="logs/eval_forward_pose_computed_pid_friction/forward_${tag}_computed_pid_friction.csv"

  echo
  echo "============================================================"
  echo "Forward pose computed_pid_friction_model | scale=$scale | tag=$tag"
  echo "CSV: $csv_path"
  echo "============================================================"

  python3 torque_main.py \
    "${COMMON[@]}" \
    --scale "$scale" \
    --csv-log "$csv_path"

  sleep 1
}

# Start conservative. Stop manually if 25% or 50% is unstable.
for item in "0.25 25pct" "0.50 50pct" "0.75 75pct" "1.00 100pct"
do
  scale=$(echo "$item" | awk '{print $1}')
  tag=$(echo "$item" | awk '{print $2}')

  run_forward "$scale" "$tag"
done

echo
echo "============================================================"
echo "Finished computed PID friction forward-pose evaluation."
echo "Logs written to logs/eval_forward_pose_computed_pid_friction/"
echo "============================================================"
