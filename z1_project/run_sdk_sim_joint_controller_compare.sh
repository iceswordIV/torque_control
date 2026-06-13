#!/usr/bin/env bash
set -euo pipefail

# Run the Unitree SDK LOWCMD baseline for the same joint/angle list used by
# run_sim_joint_controller_compare.sh, then compare SDK logs against that sim
# result directory. By default this uses the direct ROS/Gazebo LOWCMD path,
# which matches z1_controller/sim_ctrl's ROS MotorCmd output and avoids UDP
# connection hangs. Set SDK_RUNNER=udp to use run_eval_joint_workspace_sdk_gazebo.py.

PROJECT_DIR="${PROJECT_DIR:-$HOME/Desktop/torque_control/z1_project}"

SDK_RUNNER="${SDK_RUNNER:-ros_direct}"
if [[ "$SDK_RUNNER" == "udp" ]]; then
  SDK_PY="run_eval_joint_workspace_sdk_gazebo.py"
else
  SDK_PY="run_eval_joint_workspace_sdk_ros_gazebo.py"
fi

if [[ ! -f "$PROJECT_DIR/$SDK_PY" ]]; then
  echo "Cannot find $SDK_PY at: $PROJECT_DIR"
  exit 1
fi

cd "$PROJECT_DIR"

if [[ $# -ge 1 ]]; then
  SIM_DIR="$1"
else
  SIM_DIR="$(
    find logs -maxdepth 1 -type d -name 'sim_compare_joint_controllers_*' -printf '%T@ %p\n' \
      | sort -n \
      | tail -1 \
      | cut -d' ' -f2-
  )"
fi

if [[ -z "${SIM_DIR:-}" || ! -d "$SIM_DIR" ]]; then
  echo "Could not find a sim_compare_joint_controllers_* directory."
  echo "Usage: $0 logs/sim_compare_joint_controllers_YYYYMMDD_HHMMSS"
  exit 1
fi

SDK_DIR="${SDK_DIR:-$SIM_DIR/sdk_lowcmd}"
MOVE_TIME="${MOVE_TIME:-5}"
HOLD_TIME="${HOLD_TIME:-3}"
RETURN_TIME="${RETURN_TIME:-5}"
DURATION="${DURATION:-15}"
PAUSE_SEC="${PAUSE_SEC:-0.5}"
STOP_TORQUE_BRIDGE="${STOP_TORQUE_BRIDGE:-1}"
RESTART_TORQUE_BRIDGE="${RESTART_TORQUE_BRIDGE:-1}"
RUNTIME_DIR="${RUNTIME_DIR:-/tmp/z1_torque_$(id -u)}"

export LD_LIBRARY_PATH="$PROJECT_DIR/../z1_sdk/lib:${LD_LIBRARY_PATH:-}"

STOPPED_BRIDGE_PIDS=""
BRIDGE_LOG=""

restart_torque_bridge() {
  if [[ -n "$STOPPED_BRIDGE_PIDS" && "$RESTART_TORQUE_BRIDGE" == "1" ]]; then
    mkdir -p "$RUNTIME_DIR"
    printf "0 0 0 0 0 0\n" > "$RUNTIME_DIR/z1_torque_cmd.txt"
    rm -f "$RUNTIME_DIR/z1_stop.txt"
    BRIDGE_LOG="$SIM_DIR/ros_torque_bridge_restarted.log"
    echo
    echo "Restarting ros_torque_bridge.py with zero torque command."
    echo "Bridge log: $BRIDGE_LOG"
    nohup python3 ros_torque_bridge.py --runtime-dir "$RUNTIME_DIR" > "$BRIDGE_LOG" 2>&1 &
  fi
}

trap restart_torque_bridge EXIT

BRIDGE_PIDS="$(pgrep -f 'python3 ros_torque_bridge.py' || true)"
if [[ -n "$BRIDGE_PIDS" ]]; then
  if [[ "$STOP_TORQUE_BRIDGE" != "1" ]]; then
    echo "ros_torque_bridge.py is running and publishes to the same joint command topics."
    echo "Stop it first, or rerun with STOP_TORQUE_BRIDGE=1."
    exit 1
  fi
  echo "Stopping ros_torque_bridge.py while SDK LOWCMD baseline runs: $BRIDGE_PIDS"
  STOPPED_BRIDGE_PIDS="$BRIDGE_PIDS"
  kill $BRIDGE_PIDS
  sleep 1
fi

run_joint() {
  local joint="$1"
  local angles="$2"

  echo
  echo "============================================================"
  echo "SDK LOWCMD TESTS"
  echo "joint  = $joint"
  echo "angles = $angles deg"
  echo "logs   = $SDK_DIR"
  echo "============================================================"

  python3 "$SDK_PY" \
    --log-dir "$SDK_DIR" \
    --move-time "$MOVE_TIME" \
    --hold-time "$HOLD_TIME" \
    --return-time "$RETURN_TIME" \
    --duration "$DURATION" \
    --pause-sec "$PAUSE_SEC" \
    --joint "$joint" \
    --angles-deg="$angles"
}

run_joint 1 "5 10 30"
run_joint 2 "5 10 30"
run_joint 3 "-5 -10 -30"
run_joint 4 "-5 -10 -30"
run_joint 5 "-5 -10 -30 5 10 30"
run_joint 6 "-5 -10 -30 5 10 30"

python3 compare_sim_joint_controller_sdk.py \
  --sim-dir "$SIM_DIR" \
  --sdk-dir "$SDK_DIR" \
  --out "$SIM_DIR/sdk_vs_sim_controller_compare.csv"

echo
echo "SDK logs:"
echo "$SDK_DIR"
echo
echo "SDK vs sim comparison:"
echo "$SIM_DIR/sdk_vs_sim_controller_compare.csv"
