#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC=${DURATION_SEC:-10}
OUTPUT_DIR=${OUTPUT_DIR:-"benchmark_results_$(date +%Y%m%d_%H%M%S)"}
WIDTH=${WIDTH:-640}
HEIGHT=${HEIGHT:-480}
FPS=${FPS:-30.0}
QOS_DEPTH=${QOS_DEPTH:-10}
QOS_RELIABILITY=${QOS_RELIABILITY:-best_effort}

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
    -h|--help)
      echo "Usage: $0 [--duration SEC] [--output DIR] [--width PX] [--height PX] [--fps FPS] [--qos-depth N] [--qos-reliability best_effort|reliable]";
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

run_case() {
  local label="$1"
  local transport="$2"
  local shared="$3"
  local csv_path="$OUTPUT_DIR/${label}.csv"
  echo "==> $label (transport=$transport shared_memory=$shared)"
  set +e
  timeout "${DURATION_SEC}s" ros2 launch camera_latency_benchmark benchmark.launch.py \
    transport:="$transport" shared_memory:="$shared" \
    use_composition:=false intra_process:=false \
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

trap stop_roudi EXIT

run_case "raw" "raw" "false"
run_case "compressed" "compressed" "false"

start_roudi
run_case "shared_compressed" "compressed" "true"
stop_roudi

start_roudi
run_case "shared_raw" "raw" "true"
stop_roudi

echo "Results saved to: $OUTPUT_DIR"
