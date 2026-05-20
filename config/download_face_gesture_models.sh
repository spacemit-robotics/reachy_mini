#!/bin/sh
# 下载模型 (视觉 + LLM) 并可选启动 llama-server
# 模型保存路径: ~/.cache/models/vision/ 和 ~/.cache/models/llm/
#
# 用法:
#   ./download_face_gesture_models.sh                          # 仅下载模型
#   ./download_face_gesture_models.sh --start-server           # 下载模型并用默认模型启动
#   ./download_face_gesture_models.sh --start-server --model qwen2.5-0.5b-instruct-q4_0.gguf
#   ./download_face_gesture_models.sh --start-server --config /path/to/config_paras.yaml


set -e

START_SERVER=0
LLM_MODEL_NAME=""
CONFIG_FILE=""

while [ $# -gt 0 ]; do
  case "$1" in
    --start-server) START_SERVER=1; shift ;;
    --model) LLM_MODEL_NAME="$2"; shift 2 ;;
    --config) CONFIG_FILE="$2"; shift 2 ;;
    *) shift ;;
  esac
done

# 从配置文件解析 llm.model (简易 YAML 解析，仅在未通过 --model 指定时生效)
if [ -z "$LLM_MODEL_NAME" ] && [ -n "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ]; then
  # 提取 llm.model 字段值，如 "qwen2.5:0.5b"
  CFG_MODEL=$(grep -A5 '^llm:' "$CONFIG_FILE" | grep '^\s*model:' | head -1 \
    | sed 's/.*model:\s*["'\'']\?\([^"'\'']*\)["'\'']\?.*/\1/' | tr -d ' ')
  if [ -n "$CFG_MODEL" ]; then
    # 将 llm.model 格式 (如 "qwen2.5:0.5b") 映射为 gguf 文件名
    case "$CFG_MODEL" in
      qwen2.5:0.5b|qwen2.5-0.5b*)
        LLM_MODEL_NAME="qwen2.5-0.5b-instruct-q4_0.gguf" ;;
      qwen2.5:1.5b|qwen2.5-1.5b*)
        LLM_MODEL_NAME="qwen2.5-1.5b-instruct-q4_0.gguf" ;;
      qwen2.5:3b|qwen2.5-3b*)
        LLM_MODEL_NAME="qwen2.5-3b-instruct-q4_0.gguf" ;;
      glm-edge:1.5b|glm-edge-1.5b*)
        LLM_MODEL_NAME="glm-edge-1.5b-chat-q4_0.gguf" ;;
      qwen3:30b*|Qwen3-30B*)
        LLM_MODEL_NAME="Qwen3-30B-A3B-Q4_0.gguf" ;;
      *.gguf)
        LLM_MODEL_NAME="$CFG_MODEL" ;;
    esac
  fi
fi

# 默认模型
if [ -z "$LLM_MODEL_NAME" ]; then
  LLM_MODEL_NAME="qwen2.5-0.5b-instruct-q4_0.gguf"
fi

# 将简短模型名映射为 gguf 文件名 (支持 CLI --model 传入简写)
case "$LLM_MODEL_NAME" in
  qwen2.5:0.5b|qwen2.5-0.5b)
    LLM_MODEL_NAME="qwen2.5-0.5b-instruct-q4_0.gguf" ;;
  qwen2.5:1.5b|qwen2.5-1.5b)
    LLM_MODEL_NAME="qwen2.5-1.5b-instruct-q4_0.gguf" ;;
  qwen2.5:3b|qwen2.5-3b)
    LLM_MODEL_NAME="qwen2.5-3b-instruct-q4_0.gguf" ;;
  glm-edge:1.5b|glm-edge-1.5b)
    LLM_MODEL_NAME="glm-edge-1.5b-chat-q4_0.gguf" ;;

    # 已经是完整文件名
esac

echo "[ModelSetup] LLM model selected: $LLM_MODEL_NAME"

CACHE_BASE="${HOME:-/tmp}/.cache/models/vision"

download() {
  dir="$1"
  url="$2"
  name="$3"
  mkdir -p "$dir"
  if [ -f "$dir/$name" ]; then
    echo "Exists: $dir/$name"
    return 0
  fi
  echo "Downloading $name ..."
  tmp="$dir/$name.tmp.$$"
  if command -v curl >/dev/null 2>&1; then
    curl -fL -o "$tmp" "$url"
  else
    wget -O "$tmp" "$url"
  fi
  rc=$?
  if [ $rc -ne 0 ] || [ ! -s "$tmp" ]; then
    rm -f "$tmp"
    echo "ERROR: Failed to download $name" >&2
    return 1
  fi
  mv "$tmp" "$dir/$name"
}

# yolov5_gesture
download "$CACHE_BASE/yolov5" \
  "https://archive.spacemit.com/spacemit-ai/model_zoo/vision/yolov5/yolov5_gesture.q.onnx" \
  "yolov5_gesture.q.onnx"

# yolov5-face
download "$CACHE_BASE/yolov5-face" \
  "https://archive.spacemit.com/spacemit-ai/model_zoo/vision/yolov5-face/yolov5n-face_cut.q.onnx" \
  "yolov5n-face_cut.q.onnx"

# LLM 模型下载 (仅下载本地 .gguf 模型，云端 API 模型跳过)
LLM_CACHE="${HOME:-/tmp}/.cache/models/llm"
LLM_BASE_URL="https://archive.spacemit.com/spacemit-ai/model_zoo/llm"

# 检测是否为本地模型（.gguf 后缀）还是云端 API 模型
case "$LLM_MODEL_NAME" in
  *.gguf)
    # 本地 GGUF 模型，执行下载
    download "$LLM_CACHE" "$LLM_BASE_URL/$LLM_MODEL_NAME" "$LLM_MODEL_NAME"
    echo ""
    echo "Done. Models downloaded:"
    echo "  $CACHE_BASE/yolov5/yolov5_gesture.q.onnx"
    echo "  $CACHE_BASE/yolov5-face/yolov5n-face_cut.q.onnx"
    echo "  $LLM_CACHE/$LLM_MODEL_NAME"
    ;;
  *)
    # 云端 API 模型（如 gemini-embedding-001, gpt-4o-mini 等），跳过下载
    echo ""
    echo "Done. Models downloaded:"
    echo "  $CACHE_BASE/yolov5/yolov5_gesture.q.onnx"
    echo "  $CACHE_BASE/yolov5-face/yolov5n-face_cut.q.onnx"
    echo "  [Skipped] Cloud API model: $LLM_MODEL_NAME (no local download needed)"
    ;;
esac

# --------------------------------------------------------------------------
# 启动 llama-server (仅在 --start-server 模式下且为本地模型时)
# --------------------------------------------------------------------------
LLAMA_PID_FILE="/tmp/reachy_llama_server.pid"
LLM_MODEL="$LLM_CACHE/$LLM_MODEL_NAME"
LLAMA_PORT=8080

if [ "$START_SERVER" -eq 1 ]; then
  # 检查是否为云端 API 模型（不含 .gguf 后缀）
  case "$LLM_MODEL_NAME" in
    *.gguf)
      # 本地模型，继续启动 llama-server
      ;;
    *)
      # 云端 API 模型，跳过 llama-server 启动
      echo "Using cloud API model: $LLM_MODEL_NAME (skip local llama-server)"
      exit 0
      ;;
  esac

  # 检查是否已有 llama-server 在监听目标端口
  if [ -f "$LLAMA_PID_FILE" ]; then
    OLD_PID=$(cat "$LLAMA_PID_FILE" 2>/dev/null)
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
      echo "llama-server already running (PID: $OLD_PID), skip."
      exit 0
    fi
  fi

  if ! command -v llama-server >/dev/null 2>&1; then
    echo "WARNING: llama-server not found in PATH, skip starting LLM service." >&2
    exit 0
  fi

  if [ ! -f "$LLM_MODEL" ]; then
    echo "ERROR: LLM model not found: $LLM_MODEL" >&2
    exit 1
  fi

  # 内存预检: 模型文件大小 vs 可用内存 (需预留约 500MB 给系统)
  MODEL_SIZE_KB=$(du -k "$LLM_MODEL" | awk '{print $1}')
  AVAIL_MEM_KB=$(grep MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}')
  RESERVE_KB=524288  # 预留 512MB
  if [ -n "$AVAIL_MEM_KB" ] && [ "$MODEL_SIZE_KB" -gt $((AVAIL_MEM_KB - RESERVE_KB)) ]; then
    MODEL_SIZE_MB=$((MODEL_SIZE_KB / 1024))
    AVAIL_MEM_MB=$((AVAIL_MEM_KB / 1024))
    echo "ERROR: 内存不足，无法加载模型" >&2
    echo "  模型大小: ${MODEL_SIZE_MB}MB" >&2
    echo "  可用内存: ${AVAIL_MEM_MB}MB" >&2
    echo "  请选择更小的模型，如 qwen2.5:0.5b 或 qwen2.5:1.5b" >&2
    exit 1
  fi

  echo "Starting llama-server on port $LLAMA_PORT ..."
  echo "  Model: $LLM_MODEL_NAME"
  llama-server -m "$LLM_MODEL" -t 4 --port "$LLAMA_PORT" \
    > /tmp/reachy_llama_server.log 2>&1 &
  LLAMA_PID=$!
  echo "$LLAMA_PID" > "$LLAMA_PID_FILE"
  echo "llama-server started (PID: $LLAMA_PID), log: /tmp/reachy_llama_server.log"

  # 等待服务就绪 (最多 30 秒)
  printf "Waiting for llama-server to be ready"
  for _ in $(seq 1 60); do
    if command -v curl >/dev/null 2>&1; then
      if curl -sf "http://127.0.0.1:$LLAMA_PORT/health" >/dev/null 2>&1; then
        echo " OK"
        exit 0
      fi
    else
      if wget -q -O /dev/null "http://127.0.0.1:$LLAMA_PORT/health" 2>/dev/null; then
        echo " OK"
        exit 0
      fi
    fi
    # 检查进程是否还活着
    if ! kill -0 "$LLAMA_PID" 2>/dev/null; then
      echo " FAILED"
      echo "ERROR: llama-server exited unexpectedly, check /tmp/reachy_llama_server.log" >&2
      rm -f "$LLAMA_PID_FILE"
      exit 1
    fi
    printf "."
    sleep 0.5
  done
  echo " TIMEOUT (server may still be loading)"
fi

