#!/usr/bin/env bash
set -euo pipefail

mkdir -p logs/tune_j2_computed_pid_friction

KD_BASE_J1=3
KD_BASE_J3=6
KD_BASE_J4=14
KD_BASE_J5=0.6
KD_BASE_J6=0.4

KI="0 0 0 20 0 0"
INTEGRAL_LIMIT="0.8 0.8 0.8 0.8 0.8 0.8"

DAMPING="1 2 1 1 1 1"
FRICTION="1 2 1 1.5 1 1"

TAU_LIMIT="20 40 25 12 10 10"

run_j2() {
  local kp2="$1"
  local kd2="$2"
  local label="$3"

  local KP="20 ${kp2} 40 60 5 5"
  local KD="${KD_BASE_J1} ${kd2} ${KD_BASE_J3} ${KD_BASE_J4} ${KD_BASE_J5} ${KD_BASE_J6}"

  local csv="logs/tune_j2_computed_pid_friction/j2_abs_1p5_${label}.csv"

  echo
  echo "============================================================"
  echo "J2 tuning test: $label"
  echo "KP=$KP"
  echo "KD=$KD"
  echo "CSV=$csv"
  echo "============================================================"

  python3 torque_main.py \
    --mode one_joint_absolute \
    --joint 2 \
    --target "1.5" \
    --trajectory-profile scurve \
    --move-time 15 \
    --hold-time 5 \
    --return-to-start \
    --return-time 25 \
    --duration 45 \
    --test-controller computed_pid_friction_model \
    --return-controller computed_pid_friction_model \
    --kp "$KP" \
    --kd "$KD" \
    --return-kp "$KP" \
    --return-kd "$KD" \
    --ki "$KI" \
    --integral-limit "$INTEGRAL_LIMIT" \
    --model-damping "$DAMPING" \
    --model-friction "$FRICTION" \
    --tau-limit "$TAU_LIMIT" \
    --dynamics-mode analytic \
    --csv-log "$csv"

  sleep 1
}

run_j2 40 9  kp2_40_kd2_9
run_j2 60 14 kp2_60_kd2_14
run_j2 80 18 kp2_80_kd2_18

echo
echo "Finished J2 computed PID friction tuning."
echo "Logs written to logs/tune_j2_computed_pid_friction/"
