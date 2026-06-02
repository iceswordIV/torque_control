#!/usr/bin/env bash
set -euo pipefail

mkdir -p logs/eval_forward_pose_4controllers

# Tuned gains after single-joint workspace experiments.
# J4 uses stronger gain than original:
# original J4 Kp/Kd: 8 / 1
# tuned J4 Kp/Kd:    15 / 2.5
KP="20 20 40 15 5 5"
KD="3 3 6 2.5 0.6 0.4"

# Official-like damping, empirical J4 friction compensation.
# Official URDF-like friction would be: 1 2 1 1 1 1
# Tuned J4 friction from single-joint tests: 1.5
DAMPING="1 2 1 1 1 1"
FRICTION_TUNED="1 2 1 1.5 1 1"
FRICTION_OFFICIAL="1 2 1 1 1 1"

TAU_LIMIT="20 40 25 12 10 10"
TARGET="0 1.5 -1 -0.54 0 0"

COMMON=(
  --mode scaled_pose
  --target "$TARGET"
  --trajectory-profile scurve
  --move-time 15
  --return-to-start
  --return-time 15
  --duration 32
  --kp "$KP"
  --kd "$KD"
  --tau-limit "$TAU_LIMIT"
  --dynamics-mode analytic
)

run_forward() {
  local controller="$1"
  local label="$2"
  local scale="$3"
  local tag="$4"

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
  echo "Forward pose $tag | controller=$label | scale=$scale"
  echo "============================================================"

  python3 torque_main.py \
    "${COMMON[@]}" \
    --scale "$scale" \
    "${extra[@]}" \
    --csv-log "logs/eval_forward_pose_4controllers/forward_${tag}_${label}.csv"

  sleep 1
}

# Start conservative. Stop if 25% or 50% is unstable.
for item in "0.25 25pct" "0.50 50pct" "0.75 75pct" "1.00 100pct"
do
  scale=$(echo "$item" | awk '{print $1}')
  tag=$(echo "$item" | awk '{print $2}')

  run_forward augpd_nofric     augpd_nofric     "$scale" "$tag"
  run_forward augpd_fric       augpd_fric       "$scale" "$tag"
  run_forward computed_nofric  computed_nofric  "$scale" "$tag"
  run_forward computed_fric    computed_fric    "$scale" "$tag"
done

echo
echo "============================================================"
echo "Finished forward-pose 4-controller evaluation."
echo "Logs written to logs/eval_forward_pose_4controllers/"
echo "============================================================"
