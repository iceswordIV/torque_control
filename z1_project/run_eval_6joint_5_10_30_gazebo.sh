#!/usr/bin/env bash
set -euo pipefail

mkdir -p logs/eval_6joint_5_10_30

KP="20 20 40 8 5 5"
KD="3 3 6 1 0.6 0.4"
DAMPING="1 2 1 1 1 1"
FRICTION="1 2 1 1 1 1"
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
        --model-friction "$FRICTION"
      )
      ;;
    computed_nofric)
      extra=(--test-controller none)
      ;;
    computed_fric)
      extra=(
        --test-controller gazebo_friction_model
        --model-damping "$DAMPING"
        --model-friction "$FRICTION"
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
    --csv-log "logs/eval_6joint_5_10_30/${test_name}_${label}.csv"

  sleep 1
}

for joint in 1 2 3 4 5 6
do
  for deg in 5 10 30
  do
    if [[ "$joint" == "3" || "$joint" == "4" ]]; then
      angle="-$deg"
      angle_label="neg${deg}deg"
    else
      angle="$deg"
      angle_label="${deg}deg"
    fi

    test_name="report_j${joint}_${angle_label}"

    run_one augpd_nofric     augpd_nofric     "$joint" "$angle" "$test_name"
    run_one augpd_fric       augpd_fric       "$joint" "$angle" "$test_name"
    run_one computed_nofric  computed_nofric  "$joint" "$angle" "$test_name"
    run_one computed_fric    computed_fric    "$joint" "$angle" "$test_name"
  done
done
