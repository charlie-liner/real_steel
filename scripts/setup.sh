#!/bin/bash
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

echo "Creating Python virtual environment..."
python3 -m venv real_steel
source real_steel/bin/activate

echo "Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

echo "Downloading MediaPipe pose model..."
if [ ! -f data/pose_landmarker_lite.task ]; then
    curl -L -o data/pose_landmarker_lite.task \
        "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"
else
    echo "Model already exists, skipping download."
fi

echo "Verifying imports..."
python -c "import cv2; print(f'OpenCV {cv2.__version__}')"
python -c "import mediapipe; print(f'MediaPipe {mediapipe.__version__}')"
python -c "import numpy; print(f'NumPy {numpy.__version__}')"
python -c "import pybullet; print('PyBullet OK')"
python -c "import serial; print(f'pyserial {serial.VERSION}')"
python -c "import yaml; print(f'PyYAML {yaml.__version__}')"

echo ""
echo "Setup complete. Activate with: source real_steel/bin/activate"
