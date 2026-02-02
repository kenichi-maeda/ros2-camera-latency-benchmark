#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC=${DURATION_SEC:-10}
OUTPUT_DIR=${OUTPUT_DIR:-"benchmark_results_$(date +%Y%m%d_%H%M%S)"}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-30.0}
QOS_DEPTH=${QOS_DEPTH:-10}
QOS_RELIABILITY=${QOS_RELIABILITY:-best_effort}
WITH_DORA=${WITH_DORA:-false}
DORA_BRIDGE_CMD=${DORA_BRIDGE_CMD:-"dora run scripts/dora_ros_bridge_dataflow.yml"}
DORA_STARTUP_SEC=${DORA_STARTUP_SEC:-2}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      DURATION_SEC="$2"; shift 2 ;;
    --output)
      OUTPUT_DIR="$2"; shift 2 ;;
    --width)
      WIDTH="$2"; shift 2 ;;
    --height)
      HEIGHT="$2"; shift 2 ;;
    --fps)
      FPS="$2"; shift 2 ;;
    --qos-depth)
      QOS_DEPTH="$2"; shift 2 ;;
    --qos-reliability)
      QOS_RELIABILITY="$2"; shift 2 ;;
    --with-dora)
      WITH_DORA=true; shift ;;
    --dora-cmd)
      DORA_BRIDGE_CMD="$2"; shift 2 ;;
    --dora-startup-sec)
      DORA_STARTUP_SEC="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: $0 [--duration SEC] [--output DIR] [--width PX] [--height PX] [--fps FPS] [--qos-depth N] [--qos-reliability best_effort|reliable] [--with-dora] [--dora-cmd CMD] [--dora-startup-sec SEC]";
      exit 0 ;;
    *)
      echo "Unknown arg: $1"; exit 1 ;;
  esac
done

mkdir -p "$OUTPUT_DIR"

if [[ -f ros2_ws/install/setup.bash ]]; then
  # Ensure overlay is available even if user forgot to source it
  set +u
  source ros2_ws/install/setup.bash
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Activate your environment first (e.g., 'pixi s -e ros2-cpu')."
  exit 1
fi

# Stable single-machine benchmarks.
export ROS_LOCALHOST_ONLY=1

run_case() {
  local label="$1"
  local transport="$2"
  local shared="$3"
  local publish_topic="${4:-image}"
  local subscribe_topic="${5:-image}"
  local csv_path="$OUTPUT_DIR/${label}.csv"
  echo "==> $label (transport=$transport shared_memory=$shared)"
  set +e
  timeout "${DURATION_SEC}s" ros2 launch camera_latency_benchmark benchmark.launch.py \
    transport:="$transport" shared_memory:="$shared" \
    use_composition:=false intra_process:=false \
    publish_topic:="$publish_topic" subscribe_topic:="$subscribe_topic" \
    width:="$WIDTH" height:="$HEIGHT" fps:="$FPS" \
    qos_depth:="$QOS_DEPTH" qos_reliability:="$QOS_RELIABILITY" \
    csv_path:="$csv_path" >/dev/null
  local status=$?
  set -e
  if [[ $status -ne 0 && $status -ne 124 ]]; then
    echo "Run failed for $label (exit $status)"
    exit $status
  fi
}

bridge_pid=""
bridge_log=""

start_bridge() {
  local label="$1"
  local transport="$2"
  local in_topic="$3"
  local out_topic="$4"
  bridge_log="$OUTPUT_DIR/${label}_bridge.log"
  DORA_TRANSPORT="$transport" DORA_IN_TOPIC="$in_topic" DORA_OUT_TOPIC="$out_topic" \
    bash -c "$DORA_BRIDGE_CMD" >"$bridge_log" 2>&1 &
  bridge_pid=$!
  sleep "$DORA_STARTUP_SEC"
  if ! kill -0 "$bridge_pid" >/dev/null 2>&1; then
    echo "DORA bridge exited early. Log: $bridge_log"
    tail -n 60 "$bridge_log" || true
    exit 1
  fi
}

stop_bridge() {
  if [[ -n "$bridge_pid" ]]; then
    kill "$bridge_pid" >/dev/null 2>&1 || true
    wait "$bridge_pid" >/dev/null 2>&1 || true
    bridge_pid=""
  fi
  bridge_log=""
}

roudi_pid=""
start_roudi() {
  if [[ -n "$roudi_pid" ]]; then
    return
  fi
  if ! command -v iox-roudi >/dev/null 2>&1; then
    echo "iox-roudi not found. Install iceoryx or enable shared memory in your env.";
    exit 1
  fi
  iox-roudi >/dev/null 2>&1 &
  roudi_pid=$!
  sleep 1
}

stop_roudi() {
  if [[ -n "$roudi_pid" ]]; then
    kill "$roudi_pid" >/dev/null 2>&1 || true
    wait "$roudi_pid" >/dev/null 2>&1 || true
    roudi_pid=""
  fi
}

cleanup() {
  stop_bridge
  stop_roudi
}

trap cleanup EXIT

run_case "raw" "raw" "false"
run_case "compressed" "compressed" "false"

start_roudi
run_case "shared_compressed" "compressed" "true"
stop_roudi

start_roudi
run_case "shared_raw" "raw" "true"
stop_roudi

if [[ "$WITH_DORA" == "true" ]]; then
  if ! command -v dora >/dev/null 2>&1; then
    echo "dora not found in PATH. Install dora-rs-cli or disable --with-dora."
    exit 1
  fi
  start_bridge "dora_raw" "raw" "image" "image_dora"
  run_case "dora_raw" "raw" "false" "image" "image_dora"
  stop_bridge

  start_bridge "dora_compressed" "compressed" "image/compressed" "image_dora/compressed"
  run_case "dora_compressed" "compressed" "false" "image" "image_dora"
  stop_bridge
fi

echo "Results saved to: $OUTPUT_DIR"
