#!/bin/sh
# 下载 yolov5_gesture 和 yolov5-face 模型
# 模型保存路径: ~/.cache/models/vision/yolov5/ 和 ~/.cache/models/vision/yolov5-face/
# 使用: sh download_gesture_face_models.sh

set -e
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
  if command -v curl >/dev/null 2>&1; then
    curl -L -o "$dir/$name" "$url"
  else
    wget -O "$dir/$name" "$url"
  fi
}

# yolov5_gesture
download "$CACHE_BASE/yolov5" \
  "https://archive.spacemit.com/spacemit-ai/model_zoo/vision/yolov5/yolov5_gesture.q.onnx" \
  "yolov5_gesture.q.onnx"

# yolov5-face
download "$CACHE_BASE/yolov5-face" \
  "https://archive.spacemit.com/spacemit-ai/model_zoo/vision/yolov5-face/yolov5n-face_cut.q.onnx" \
  "yolov5n-face_cut.q.onnx"

echo ""
echo "Done. Models downloaded:"
echo "  $CACHE_BASE/yolov5/yolov5_gesture.q.onnx"
echo "  $CACHE_BASE/yolov5-face/yolov5n-face_cut.q.onnx"

