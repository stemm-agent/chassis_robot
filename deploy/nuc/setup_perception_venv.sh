#!/usr/bin/env bash
set -euo pipefail

VENV_PATH=${VENV_PATH:-$HOME/.venvs/stemm-perception}
PIP_INDEX_URL=${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}

python3 -m venv --system-site-packages "$VENV_PATH"
"$VENV_PATH/bin/python" -m pip install \
  --index-url "$PIP_INDEX_URL" \
  'numpy==1.26.4' \
  'onnxruntime==1.23.2'
"$VENV_PATH/bin/python" -c \
  'import cv2, cv_bridge, numpy, onnxruntime, rclpy; print("STEMM perception runtime OK")'
