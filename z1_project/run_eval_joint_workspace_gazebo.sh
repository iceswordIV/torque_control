#!/usr/bin/env bash
set -euo pipefail

mkdir -p logs/eval_joint_workspace

# Base gains
# J4 is tuned moderately based on your tests:
# original J4 Kp/Kd: 8 / 1
# tuned J4 Kp/Kd:    15 / 2.5
KP="20 20 40 15 5 5"
KD="3 3 6 2.5 0.6 0.4"

# Official URDF-like damping/friction, but J4 friction is empirically compensated.
# Official:  [1 2 1 1   1 1]
# Tuned J4: [1 2 1 1.5 1 1]
DAMPING="1 2 1 1 1 1"
FRICTION_TUNED="1 2 1 1.5 1 1"
FRICTION_OFFICIAL="1 2 1 1 1 1"

TAU_LIMIT="20 40 25 12 10 10"

COMMON=(
  --trajectory-profile scurve
  --move-time 10
  --return-to-start
  --return-time 10
  --duration 22
  --kp "$KP"
  --kd "$KD"
  --tau-limit "$TAU_LIMIT"
  --dynamics-mode analytic
)

run_one() {
  local controller="$1"
  local label="$2"
  local joint="$3"
  local angle="$4"
  local test_name="$5"

  local extra=()

  case "$controller" in
    augpd_nofric)
      extra=(--test-controller augmented_pd)
      ;;
    augpd_fric)
      extra=(
        --test-controller augmented_pd_friction_model
        --model-damping "$DAMPING"
        --model-friction "$FRICTION_TUNED"
      )
      ;;
    computed_nofric)
      extra=(--test-controller none)
      ;;
    computed_fric)
      extra=(
        --test-controller gazebo_friction_model
        --model-damping "$DAMPING"
        --model-friction "$FRICTION_TUNED"
      )
      ;;
    *)
      echo "unknown controller: $controller"
      exit 1
      ;;
  esac

  echo
  echo "============================================================"
  echo "Running $test_name | controller=$label | joint=$joint | angle=$angle deg"
  echo "============================================================"

  python3 torque_main.py \
    --mode one_joint_relative \
    --joint "$joint" \
    --angle-deg "$angle" \
    "${COMMON[@]}" \
    "${extra[@]}" \
    --csv-log "logs/eval_joint_workspace/${test_name}_${label}.csv"

  sleep 1
}

run_joint_angle_set() {
  local joint="$1"
  shift
  local angles=("$@")

  for angle in "${angles[@]}"
  do
    if [[ "$angle" == -* ]]; then
      angle_label="neg${angle#-}deg"
    else
      angle_label="pos${angle}deg"
    fi

    test_name="workspace_j${joint}_${angle_label}"

    run_one augpd_nofric     augpd_nofric     "$joint" "$angle" "$test_name"
    run_one augpd_fric       augpd_fric       "$joint" "$angle" "$test_name"
    run_one computed_nofric  computed_nofric  "$joint" "$angle" "$test_name"
    run_one computed_fric    computed_fric    "$joint" "$angle" "$test_name"
  done
}

# Your requested workspace test list
run_joint_angle_set 1 -30 -10 -5 5 10 30
run_joint_angle_set 2 5 10 30
run_joint_angle_set 3 -5 -10 -30
run_joint_angle_set 4 -5 -10 -30 5 10 30
run_joint_angle_set 5 -5 -10 -30 5 10 30
run_joint_angle_set 6 -5 -10 -30 5 10 30

echo
echo "============================================================"
echo "Finished joint workspace evaluation."
echo "Logs written to logs/eval_joint_workspace/"
echo "============================================================"
