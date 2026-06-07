#!/usr/bin/env bash
# Batch runner for railway sequences on Splat-SLAM.
# It keeps the algorithm flow unchanged and only relies on the railway clean export
# stage at the end of SLAM.terminate().

set -uo pipefail

PROJECT_ROOT="/home/leizongru/lzr_ws/Splat-SLAM"
DATA_ROOT="/home/leizongru/lzr_ws/railway_data"
BASE_CONFIG="$PROJECT_ROOT/configs/Railway/railway.yaml"
OUTPUT_ROOT="$PROJECT_ROOT/output"
LOG_ROOT="$PROJECT_ROOT/logs"
PYTHON_BIN="${PYTHON_BIN:-/home/leizongru/miniconda3/envs/splat-slam/bin/python}"
THIRDPARTY_PYTHONPATH="$PROJECT_ROOT:$PROJECT_ROOT/thirdparty/lietorch:$PROJECT_ROOT/thirdparty/simple-knn:$PROJECT_ROOT/thirdparty/diff-gaussian-rasterization-w-pose:$PROJECT_ROOT/thirdparty/evaluate_3d_reconstruction_lib"
if [[ -n "${PYTHONPATH:-}" ]]; then
  RUN_PYTHONPATH="$THIRDPARTY_PYTHONPATH:$PYTHONPATH"
else
  RUN_PYTHONPATH="$THIRDPARTY_PYTHONPATH"
fi

MIN_FREE_MB="${MIN_FREE_MB:-20000}"
MAX_UTIL="${MAX_UTIL:-10}"
RETRIES="${RETRIES:-0}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-0}"
KEEP_FAILED_OUTPUT="${KEEP_FAILED_OUTPUT:-0}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"

DEFAULT_SCENES=(
  scene_05_train
  scene_11_train
  scene_13_train
  scene_14_train
  scene_16_train
  scene_17_train
  scene_19_train
)

RUN_DIR="$LOG_ROOT/railway_batch_$RUN_ID"
CONFIG_DIR="$RUN_DIR/configs"
STATUS_DIR="$RUN_DIR/status"
FAILED_OUTPUT_DIR="$RUN_DIR/failed_outputs"
QUEUE_FILE="$RUN_DIR/queue.txt"
SUMMARY_FILE="$RUN_DIR/summary.tsv"
SUMMARY_LOCK="$RUN_DIR/summary.lock"
QUEUE_LOCK="$RUN_DIR/queue.lock"

mkdir -p "$CONFIG_DIR" "$STATUS_DIR" "$LOG_ROOT"

log_msg() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

print_usage() {
  cat <<'EOF'
Usage:
  GPUS=2,3 bash scripts/run_railway_batch.sh

Environment variables:
  GPUS                 Physical GPU ids to use, comma or space separated. If unset, auto-detect free GPUs.
  SCENES               Optional scene list. If unset, runs all seven railway sequences.
  RETRIES              Retry count per scene after failure. Default: 0.
  TIMEOUT_SECONDS      Per-attempt timeout. 0 means no timeout. Default: 0.
  KEEP_FAILED_OUTPUT   1 moves failed partial output into the run log dir; 0 deletes it. Default: 0.
  MIN_FREE_MB          Auto-detect threshold for free GPU memory. Default: 20000.
  MAX_UTIL             Auto-detect threshold for GPU utilization. Default: 10.
  RUN_ID               Optional run id used in logs/railway_batch_<RUN_ID>.
  PYTHON_BIN           Python executable. Default: splat-slam conda env python.

Logs:
  /home/leizongru/lzr_ws/Splat-SLAM/logs/railway_batch_<RUN_ID>/
EOF
}

normalize_list() {
  printf '%s\n' "$1" | tr ',' ' ' | xargs
}

detect_free_gpus() {
  nvidia-smi --query-gpu=index,memory.free,utilization.gpu --format=csv,noheader,nounits \
    | awk -F',' -v min_free="$MIN_FREE_MB" -v max_util="$MAX_UTIL" '
      {
        gsub(/ /, "", $1); gsub(/ /, "", $2); gsub(/ /, "", $3);
        if ($2 >= min_free && $3 <= max_util) print $1;
      }'
}

prepare_run_dir() {
  mkdir -p "$RUN_DIR" "$CONFIG_DIR" "$STATUS_DIR"
  if [[ "$KEEP_FAILED_OUTPUT" == "1" ]]; then
    mkdir -p "$FAILED_OUTPUT_DIR"
  fi
  : > "$SUMMARY_FILE"
  printf 'scene\tgpu\tattempt\tstatus\texit_code\tstart_time\tend_time\tlog\n' >> "$SUMMARY_FILE"
  nvidia-smi > "$RUN_DIR/gpu_snapshot_before.txt" 2>&1 || true
}

resolve_scenes() {
  if [[ -n "${SCENES:-}" ]]; then
    read -r -a SCENE_LIST <<< "$(normalize_list "$SCENES")"
  else
    SCENE_LIST=("${DEFAULT_SCENES[@]}")
  fi
}

resolve_gpus() {
  if [[ -n "${GPUS:-}" ]]; then
    read -r -a GPU_LIST <<< "$(normalize_list "$GPUS")"
  else
    mapfile -t GPU_LIST < <(detect_free_gpus)
  fi

  if [[ ${#GPU_LIST[@]} -eq 0 ]]; then
    log_msg "ERROR: no free GPU found. You can override with GPUS=2,3."
    exit 2
  fi
}

validate_inputs() {
  if [[ ! -f "$BASE_CONFIG" ]]; then
    log_msg "ERROR: missing base config: $BASE_CONFIG"
    exit 2
  fi
  if [[ ! -x "$PYTHON_BIN" ]]; then
    log_msg "ERROR: python is not executable: $PYTHON_BIN"
    exit 2
  fi
  for scene in "${SCENE_LIST[@]}"; do
    if [[ ! -d "$DATA_ROOT/$scene" ]]; then
      log_msg "ERROR: missing scene directory: $DATA_ROOT/$scene"
      exit 2
    fi
    if [[ ! -f "$DATA_ROOT/gt_poses/$scene.parquet" ]]; then
      log_msg "ERROR: missing GT pose parquet: $DATA_ROOT/gt_poses/$scene.parquet"
      exit 2
    fi
  done
}

generate_config() {
  local scene="$1"
  local cfg="$2"
  awk -v scene="$scene" '
    /^scene:/ { print "scene: " scene; next }
    /^[[:space:]]*input_folder:/ { print "  input_folder: " scene; next }
    { print }
  ' "$BASE_CONFIG" > "$cfg"
}

next_scene() {
  local scene=""
  {
    flock -x 200
    if [[ ! -s "$QUEUE_FILE" ]]; then
      exit 1
    fi
    scene="$(head -n 1 "$QUEUE_FILE")"
    tail -n +2 "$QUEUE_FILE" > "$QUEUE_FILE.tmp"
    mv "$QUEUE_FILE.tmp" "$QUEUE_FILE"
    printf '%s\n' "$scene"
  } 200>"$QUEUE_LOCK"
}

append_summary() {
  local scene="$1"
  local gpu="$2"
  local attempt="$3"
  local status="$4"
  local code="$5"
  local start_time="$6"
  local end_time="$7"
  local log_file="$8"
  {
    flock -x 201
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$scene" "$gpu" "$attempt" "$status" "$code" "$start_time" "$end_time" "$log_file" >> "$SUMMARY_FILE"
  } 201>"$SUMMARY_LOCK"
}

validate_output() {
  local scene="$1"
  local out_dir="$OUTPUT_ROOT/$scene"
  [[ -f "$out_dir/config.yaml" ]] || return 1
  [[ -s "$out_dir/poses/estimated_c2w.csv" ]] || return 1
  [[ -s "$out_dir/poses/estimated_c2w_tum.txt" ]] || return 1
  [[ -s "$out_dir/metrics/render_metrics.csv" ]] || return 1
  [[ -s "$out_dir/metrics/render_metrics_summary.json" ]] || return 1
  [[ -s "$out_dir/metrics/ate_metrics.json" ]] || return 1
  shopt -s nullglob
  local renders=("$out_dir"/renders/*.png)
  shopt -u nullglob
  [[ ${#renders[@]} -gt 0 ]] || return 1
  local extra=""
  extra="$(find "$out_dir" -mindepth 1 -maxdepth 1 \
    ! -name config.yaml ! -name renders ! -name poses ! -name metrics -print -quit)"
  [[ -z "$extra" ]] || return 1
}

handle_failed_output() {
  local scene="$1"
  local attempt="$2"
  local out_dir="$OUTPUT_ROOT/$scene"
  [[ -d "$out_dir" ]] || return 0
  if [[ "$KEEP_FAILED_OUTPUT" == "1" ]]; then
    mkdir -p "$FAILED_OUTPUT_DIR"
    mv "$out_dir" "$FAILED_OUTPUT_DIR/${scene}_attempt${attempt}_$(date +%H%M%S)" 2>/dev/null || rm -rf "$out_dir"
  else
    rm -rf "$out_dir"
  fi
}

run_one_scene() {
  local scene="$1"
  local gpu="$2"
  local max_attempts=$((RETRIES + 1))
  local attempt=1
  local final_status="FAIL"
  local final_code=1

  rm -f "$STATUS_DIR/$scene.ok" "$STATUS_DIR/$scene.failed"

  while [[ $attempt -le $max_attempts ]]; do
    local cfg="$CONFIG_DIR/${scene}.yaml"
    local log_file="$RUN_DIR/${scene}.gpu${gpu}.attempt${attempt}.log"
    local out_dir="$OUTPUT_ROOT/$scene"
    local start_time="$(date '+%F %T')"
    local end_time=""
    local code=0

    generate_config "$scene" "$cfg"
    rm -rf "$out_dir"

    {
      echo "===== Splat-SLAM railway batch ====="
      echo "scene: $scene"
      echo "physical_gpu: $gpu"
      echo "attempt: $attempt/$max_attempts"
      echo "config: $cfg"
      echo "output: $out_dir"
      echo "start: $start_time"
      echo "CUDA_VISIBLE_DEVICES=$gpu"
      echo "PYTHONPATH=$RUN_PYTHONPATH"
      echo "===================================="
    } > "$log_file"

    log_msg "START scene=$scene gpu=$gpu attempt=$attempt/$max_attempts"

    if [[ "$TIMEOUT_SECONDS" -gt 0 ]]; then
      CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES="$gpu" PYTHONPATH="$RUN_PYTHONPATH" \
        timeout "$TIMEOUT_SECONDS" "$PYTHON_BIN" "$PROJECT_ROOT/run.py" "$cfg" >> "$log_file" 2>&1
      code=$?
    else
      CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES="$gpu" PYTHONPATH="$RUN_PYTHONPATH" \
        "$PYTHON_BIN" "$PROJECT_ROOT/run.py" "$cfg" >> "$log_file" 2>&1
      code=$?
    fi

    end_time="$(date '+%F %T')"

    if [[ $code -eq 0 ]] && validate_output "$scene"; then
      final_status="OK"
      final_code=0
      append_summary "$scene" "$gpu" "$attempt" "OK" "0" "$start_time" "$end_time" "$log_file"
      printf 'ok\n' > "$STATUS_DIR/$scene.ok"
      log_msg "DONE scene=$scene gpu=$gpu"
      return 0
    fi

    if [[ $code -eq 0 ]]; then
      code=97
      echo "ERROR: run.py exited 0 but clean railway outputs are missing or not clean." >> "$log_file"
    fi
    append_summary "$scene" "$gpu" "$attempt" "FAIL" "$code" "$start_time" "$end_time" "$log_file"
    log_msg "FAIL scene=$scene gpu=$gpu attempt=$attempt code=$code log=$log_file"
    handle_failed_output "$scene" "$attempt"

    final_code=$code
    attempt=$((attempt + 1))
    if [[ $attempt -le $max_attempts ]]; then
      sleep 10
    fi
  done

  printf '%s\n' "$final_code" > "$STATUS_DIR/$scene.failed"
  return "$final_code"
}

worker() {
  local gpu="$1"
  while true; do
    local scene=""
    scene="$(next_scene)" || break
    run_one_scene "$scene" "$gpu"
  done
}

cleanup_on_signal() {
  log_msg "Received interrupt; stopping child jobs."
  jobs -pr | xargs -r kill 2>/dev/null || true
  wait 2>/dev/null || true
  exit 130
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    print_usage
    exit 0
  fi

  cd "$PROJECT_ROOT" || exit 2
  resolve_scenes
  resolve_gpus
  validate_inputs
  prepare_run_dir

  printf '%s\n' "${SCENE_LIST[@]}" > "$QUEUE_FILE"

  log_msg "Run dir: $RUN_DIR"
  log_msg "Scenes: ${SCENE_LIST[*]}"
  log_msg "GPUs: ${GPU_LIST[*]}"
  log_msg "Retries per scene: $RETRIES"
  log_msg "Timeout seconds: $TIMEOUT_SECONDS"
  log_msg "Keep failed output: $KEEP_FAILED_OUTPUT"

  trap cleanup_on_signal INT TERM

  local gpu=""
  for gpu in "${GPU_LIST[@]}"; do
    worker "$gpu" &
  done

  local wait_code=0
  wait || wait_code=$?

  nvidia-smi > "$RUN_DIR/gpu_snapshot_after.txt" 2>&1 || true

  log_msg "Summary: $SUMMARY_FILE"
  column -t -s $'\t' "$SUMMARY_FILE" 2>/dev/null || cat "$SUMMARY_FILE"

  local failed_count=0
  failed_count="$(find "$STATUS_DIR" -name '*.failed' | wc -l | tr -d ' ')"
  if [[ "$failed_count" -gt 0 ]]; then
    log_msg "Batch finished with $failed_count failed sequence(s)."
    exit 1
  fi

  if [[ "$wait_code" -ne 0 ]]; then
    log_msg "Batch finished with worker wait code $wait_code."
    exit "$wait_code"
  fi

  log_msg "Batch finished successfully."
}

main "$@"
