#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Simulation batch comparison:
#   1) AugPD no friction
#   2) AugPD with friction
#   3) Computed-torque baseline from controller.py
#   4) Our CPID with friction
#
# Also runs M/C/N comparison:
#   local analytic dynamics vs Unitree SDK inverseDynamics
#
# Make sure Gazebo simulation + pure_torque_bridge are running.
# DO NOT run this on real arm.
# ============================================================

PROJECT_DIR="${PROJECT_DIR:-$HOME/Desktop/torque_control/z1_project}"

if [[ ! -f "$PROJECT_DIR/torque_main.py" ]]; then
  echo "Cannot find torque_main.py at: $PROJECT_DIR"
  echo "Edit PROJECT_DIR at the top of this script."
  exit 1
fi

cd "$PROJECT_DIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUTDIR="logs/sim_compare_joint_controllers_${STAMP}"
mkdir -p "$OUTDIR"

MOVE_TIME=5
HOLD_TIME=3
RETURN_TIME=5
DURATION=15

TAU_LIMIT="5 12 12 10 3 3"

# AugPD gains
AUGPD_KP="20 25 25 15 5 5"
AUGPD_KD="4 7 7 3 1 1"

# Computed-torque baseline natural-frequency gains
BASE_WN="2 2 1.5 1.5 1.2 1.2"
BASE_ZETA="1 1 1 1 1 1"

# Our CPID gains
CPID_KP="64 100 100 60 64 100"
CPID_KD="13 16 16 14 13 16"
CPID_KI="0 0 0 20 0 0"
INTEGRAL_LIMIT="0.8 0.8 0.8 0.8 0.8 0.8"

# Friction/damping model
MODEL_DAMPING="1 2 1 1 1 1"
MODEL_FRICTION="1 2.5 1 1.5 1 1.5"

run_one() {
  local ctrl="$1"
  local joint="$2"
  local angle="$3"
  local label="$4"

  local csv="${OUTDIR}/${ctrl}_${label}.csv"

  echo
  echo "============================================================"
  echo "SIM TEST"
  echo "controller = ${ctrl}"
  echo "joint      = ${joint}"
  echo "angle      = ${angle} deg"
  echo "move/hold/return = ${MOVE_TIME}/${HOLD_TIME}/${RETURN_TIME} s"
  echo "csv        = ${csv}"
  echo "============================================================"

  case "$ctrl" in
    augpd_nofric)
      python3 torque_main.py \
        --mode one_joint_relative \
        --joint "$joint" \
        --angle-deg "$angle" \
        --trajectory-profile scurve \
        --move-time "$MOVE_TIME" \
        --hold-time "$HOLD_TIME" \
        --return-to-start \
        --return-time "$RETURN_TIME" \
        --duration "$DURATION" \
        --test-controller augmented_pd \
        --return-controller augmented_pd \
        --kp "$AUGPD_KP" \
        --kd "$AUGPD_KD" \
        --tau-limit "$TAU_LIMIT" \
        --dynamics-mode analytic \
        --csv-log "$csv"
      ;;

    augpd_fric2p5)
      python3 torque_main.py \
        --mode one_joint_relative \
        --joint "$joint" \
        --angle-deg "$angle" \
        --trajectory-profile scurve \
        --move-time "$MOVE_TIME" \
        --hold-time "$HOLD_TIME" \
        --return-to-start \
        --return-time "$RETURN_TIME" \
        --duration "$DURATION" \
        --test-controller augmented_pd_friction_model \
        --return-controller augmented_pd_friction_model \
        --kp "$AUGPD_KP" \
        --kd "$AUGPD_KD" \
        --model-damping "$MODEL_DAMPING" \
        --model-friction "$MODEL_FRICTION" \
        --tau-limit "$TAU_LIMIT" \
        --dynamics-mode analytic \
        --csv-log "$csv"
      ;;

    computed_torque_baseline)
      python3 torque_main.py \
        --mode one_joint_relative \
        --joint "$joint" \
        --angle-deg "$angle" \
        --trajectory-profile scurve \
        --move-time "$MOVE_TIME" \
        --hold-time "$HOLD_TIME" \
        --return-to-start \
        --return-time "$RETURN_TIME" \
        --duration "$DURATION" \
        --test-controller none \
        --return-controller none \
        --wn "$BASE_WN" \
        --zeta "$BASE_ZETA" \
        --tau-limit "$TAU_LIMIT" \
        --dynamics-mode analytic \
        --csv-log "$csv"
      ;;

    cpid_fric2p5)
      python3 torque_main.py \
        --mode one_joint_relative \
        --joint "$joint" \
        --angle-deg "$angle" \
        --trajectory-profile scurve \
        --move-time "$MOVE_TIME" \
        --hold-time "$HOLD_TIME" \
        --return-to-start \
        --return-time "$RETURN_TIME" \
        --duration "$DURATION" \
        --test-controller computed_pid_friction_model \
        --return-controller computed_pid_friction_model \
        --kp "$CPID_KP" \
        --kd "$CPID_KD" \
        --ki "$CPID_KI" \
        --integral-limit "$INTEGRAL_LIMIT" \
        --model-damping "$MODEL_DAMPING" \
        --model-friction "$MODEL_FRICTION" \
        --tau-limit "$TAU_LIMIT" \
        --dynamics-mode analytic \
        --csv-log "$csv"
      ;;

    *)
      echo "Unknown controller: $ctrl"
      exit 1
      ;;
  esac

  sleep 0.5
}

# ------------------------------------------------------------
# Test list:
# J1: +5 +10 +30
# J2: +5 +10 +30
# J3: -5 -10 -30
# J4: -5 -10 -30
# J5: -5 -10 -30 +5 +10 +30
# J6: -5 -10 -30 +5 +10 +30
# ------------------------------------------------------------

TESTS=(
  "1 5 j1_pos5"
  "1 10 j1_pos10"
  "1 30 j1_pos30"

  "2 5 j2_pos5"
  "2 10 j2_pos10"
  "2 30 j2_pos30"

  "3 -5 j3_neg5"
  "3 -10 j3_neg10"
  "3 -30 j3_neg30"

  "4 -5 j4_neg5"
  "4 -10 j4_neg10"
  "4 -30 j4_neg30"

  "5 -5 j5_neg5"
  "5 -10 j5_neg10"
  "5 -30 j5_neg30"
  "5 5 j5_pos5"
  "5 10 j5_pos10"
  "5 30 j5_pos30"

  "6 -5 j6_neg5"
  "6 -10 j6_neg10"
  "6 -30 j6_neg30"
  "6 5 j6_pos5"
  "6 10 j6_pos10"
  "6 30 j6_pos30"
)

CONTROLLERS=(
  "augpd_nofric"
  "augpd_fric2p5"
  "computed_torque_baseline"
  "cpid_fric2p5"
)

echo "Output directory: $OUTDIR"
echo "Total tests: $((${#TESTS[@]} * ${#CONTROLLERS[@]}))"

for ctrl in "${CONTROLLERS[@]}"; do
  echo
  echo "############################################################"
  echo "START CONTROLLER GROUP: $ctrl"
  echo "############################################################"

  for item in "${TESTS[@]}"; do
    read -r joint angle label <<< "$item"
    run_one "$ctrl" "$joint" "$angle" "$label"
  done
done

# ------------------------------------------------------------
# Build summary CSV
# ------------------------------------------------------------
python3 - "$OUTDIR" <<'PY'
import csv
import glob
import math
import os
import re
import sys

outdir = sys.argv[1]
summary_path = os.path.join(outdir, "summary_controller_compare.csv")

rows_out = []

pattern = r"(augpd_nofric|augpd_fric2p5|computed_torque_baseline|cpid_fric2p5)_j(\d+)_(pos|neg)(\d+)\.csv"

for path in sorted(glob.glob(os.path.join(outdir, "*.csv"))):
    name = os.path.basename(path)

    m = re.match(pattern, name)
    if not m:
        continue

    controller, joint_s, sign, angle_abs_s = m.groups()
    joint = int(joint_s)
    angle_deg = float(angle_abs_s) * (1 if sign == "pos" else -1)

    q_actual_key = f"q_actual_{joint}"
    q_des_key = f"q_des_{joint}"
    tau_key = f"tau_{joint}"

    errors = []
    taus = []
    all_errors = []
    all_taus = []
    final_error = None

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                e = float(r[q_des_key]) - float(r[q_actual_key])
                tau = float(r[tau_key])
                errors.append(e)
                taus.append(tau)
                final_error = e

                for j in range(1, 7):
                    all_errors.append(float(r[f"q_des_{j}"]) - float(r[f"q_actual_{j}"]))
                    all_taus.append(float(r[f"tau_{j}"]))
            except Exception:
                pass

    if not errors:
        continue

    rms = math.sqrt(sum(e * e for e in errors) / len(errors))
    max_abs_error = max(abs(e) for e in errors)
    final_abs_error = abs(final_error)
    max_abs_tau = max(abs(t) for t in taus)
    all_joint_max_abs_error = max(abs(e) for e in all_errors)
    all_joint_max_abs_tau = max(abs(t) for t in all_taus)

    rows_out.append({
        "controller": controller,
        "joint": joint,
        "angle_deg": angle_deg,
        "max_abs_error_rad_cmd_joint": max_abs_error,
        "final_abs_error_rad_cmd_joint": final_abs_error,
        "rms_error_rad_cmd_joint": rms,
        "max_abs_tau_nm_cmd_joint": max_abs_tau,
        "all_joint_max_abs_error_rad": all_joint_max_abs_error,
        "all_joint_max_abs_tau_nm": all_joint_max_abs_tau,
        "csv": path,
    })

with open(summary_path, "w", newline="") as f:
    fieldnames = [
        "controller",
        "joint",
        "angle_deg",
        "max_abs_error_rad_cmd_joint",
        "final_abs_error_rad_cmd_joint",
        "rms_error_rad_cmd_joint",
        "max_abs_tau_nm_cmd_joint",
        "all_joint_max_abs_error_rad",
        "all_joint_max_abs_tau_nm",
        "csv",
    ]
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows_out)

print("Summary written to:", summary_path)

# Also make controller-average summary.
avg_path = os.path.join(outdir, "summary_controller_average.csv")
groups = {}
for r in rows_out:
    groups.setdefault(r["controller"], []).append(r)

with open(avg_path, "w", newline="") as f:
    fieldnames = [
        "controller",
        "mean_max_abs_error_rad",
        "mean_final_abs_error_rad",
        "mean_rms_error_rad",
        "mean_max_abs_tau_nm",
        "num_tests",
    ]
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

    for ctrl, rows in sorted(groups.items()):
        writer.writerow({
            "controller": ctrl,
            "mean_max_abs_error_rad": sum(x["max_abs_error_rad_cmd_joint"] for x in rows) / len(rows),
            "mean_final_abs_error_rad": sum(x["final_abs_error_rad_cmd_joint"] for x in rows) / len(rows),
            "mean_rms_error_rad": sum(x["rms_error_rad_cmd_joint"] for x in rows) / len(rows),
            "mean_max_abs_tau_nm": sum(x["max_abs_tau_nm_cmd_joint"] for x in rows) / len(rows),
            "num_tests": len(rows),
        })

print("Controller-average summary written to:", avg_path)
PY

# ------------------------------------------------------------
# Compare local analytic M/C/N with Unitree SDK inverseDynamics
# ------------------------------------------------------------
echo
echo "============================================================"
echo "M/C/N DYNAMICS COMPARISON: Python analytic model vs Unitree SDK"
echo "============================================================"

DYN_OUT="${OUTDIR}/sdk_dynamics_compare.txt"

{
  echo "===== Case 1: default mixed pose ====="
  python3 compare_sdk_dynamics.py \
    --q "0.2 0.25 -0.25 0.15 0.1 0.1" \
    --dq "0.5 -0.3 0.2 -0.1 0.4 -0.2" \
    --ddq "0.1 -0.2 0.3 -0.1 0.05 -0.02"

  echo
  echo "===== Case 2: J2/J3 90 deg pose ====="
  python3 compare_sdk_dynamics.py \
    --q "0 1.5708 -1.5708 -0.074 0 0" \
    --dq "0.1 -0.2 0.15 0 0 0" \
    --ddq "0.05 -0.1 0.1 0 0 0"

  echo
  echo "===== Case 3: forward 100pct pose ====="
  python3 compare_sdk_dynamics.py \
    --q "0 1.5 -1.0 -0.54 0 0" \
    --dq "0.1 -0.2 0.1 -0.05 0 0" \
    --ddq "0.05 -0.1 0.05 -0.02 0 0"
} > "$DYN_OUT" 2>&1 || {
  echo "SDK dynamics compare failed. This is usually because Unitree SDK Python module is not available."
  echo "See log:"
  echo "$DYN_OUT"
}

echo
echo "============================================================"
echo "ALL SIM COMPARISON TESTS FINISHED"
echo "Output directory:"
echo "$OUTDIR"
echo
echo "Joint tracking summary:"
echo "$OUTDIR/summary_controller_compare.csv"
echo
echo "Controller average summary:"
echo "$OUTDIR/summary_controller_average.csv"
echo
echo "M/C/N SDK comparison:"
echo "$OUTDIR/sdk_dynamics_compare.txt"
echo "============================================================"
