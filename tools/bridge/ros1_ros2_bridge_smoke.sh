#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "MISSING: $cmd"
    return 1
  fi
  return 0
}

echo "[check] verifying required commands"
MISSING=0
for cmd in roscore rostopic ros2; do
  if ! require_cmd "$cmd"; then
    MISSING=1
  fi
done

set +u
if ! . /opt/ros/humble/setup.bash >/dev/null 2>&1; then
  echo "MISSING: /opt/ros/humble/setup.bash"
  MISSING=1
fi
set -u

if [[ "$MISSING" -ne 0 ]]; then
  echo "[result] prerequisites missing; bridge smoke test not run"
  exit 2
fi

if ! ros2 pkg list | grep -q '^ros1_bridge$'; then
  echo "MISSING: ros1_bridge package in current ROS 2 environment"
  echo "[result] prerequisites missing; bridge smoke test not run"
  exit 2
fi

ROSCORE_LOG="/tmp/roscore_bridge_smoke.log"
BRIDGE_LOG="/tmp/ros1_bridge_smoke.log"
ROS2_ECHO_OUT="/tmp/ros2_bridge_echo.out"

cleanup() {
  set +e
  [[ -n "${ECHO_PID:-}" ]] && kill "$ECHO_PID" >/dev/null 2>&1
  [[ -n "${PUB_PID:-}" ]] && kill "$PUB_PID" >/dev/null 2>&1
  [[ -n "${BRIDGE_PID:-}" ]] && kill "$BRIDGE_PID" >/dev/null 2>&1
  [[ -n "${ROSCORE_PID:-}" ]] && kill "$ROSCORE_PID" >/dev/null 2>&1
}
trap cleanup EXIT

echo "[run] starting roscore"
roscore >"$ROSCORE_LOG" 2>&1 &
ROSCORE_PID=$!
sleep 2

echo "[run] starting dynamic bridge"
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
sleep 3

echo "[run] starting ROS 2 echo"
ros2 topic echo /bridge_smoke/joint_states --once >"$ROS2_ECHO_OUT" 2>&1 &
ECHO_PID=$!
sleep 1

echo "[run] publishing ROS 1 joint state telemetry"
rostopic pub -r 5 /bridge_smoke/joint_states sensor_msgs/JointState \
  "{name: ['right_j0'], position: [0.1], velocity: [0.0], effort: [0.0]}" \
  >/tmp/ros1_bridge_pub.log 2>&1 &
PUB_PID=$!

# wait up to 12s for ROS2 echo to finish
for _ in $(seq 1 24); do
  if ! kill -0 "$ECHO_PID" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

if kill -0 "$ECHO_PID" >/dev/null 2>&1; then
  echo "[fail] did not observe bridged telemetry on ROS 2 topic"
  exit 1
fi

if grep -q "name:" "$ROS2_ECHO_OUT"; then
  echo "[pass] observed bridged telemetry on /bridge_smoke/joint_states"
  exit 0
fi

echo "[fail] ROS 2 echo completed without expected JointState payload"
exit 1
