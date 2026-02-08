# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Real Steel is a shadow boxing humanoid robot that tracks and reproduces human boxing movements in real-time. The robot mirrors human punching motions with <500ms latency using computer vision (MediaPipe) for pose estimation and either PyBullet simulation or real hardware (ESP32 + PCA9685 + MG996R servos) for actuation.

The full PRD lives in `docs/PROJECT.md`.

## Architecture

The system is a linear pipeline running on macOS (Python):

```
Camera (OpenCV) → Pose Estimation (MediaPipe) → Angle Extraction → Motion Mapping → Robot Interface
```

`RobotInterface` is an abstract base class with two implementations:
- `SimulatedRobot` — PyBullet simulation (development/testing, uses URDF model)
- `RealRobot` — Serial communication to ESP32 controller (production hardware)

The ESP32 receives ASCII serial commands at 115200 baud and drives 6 servos (3 per arm) via a PCA9685 PWM driver over I2C.

**Key conventions:**
- All internal angle processing uses **radians**; conversion to degrees only at the ESP32 serial boundary
- Robot has 7 DOF: shoulder tilt, shoulder pan, elbow for each arm + torso yaw
- Control loop target: 50 Hz (20ms update rate)
- Mirroring mode: human left arm maps to robot right arm (shadow boxing)

## Directory Layout

```
src/              # Python modules: camera, pose_estimator, angle_calculator, motion_mapper, robot_interface, simulated_robot, real_robot, main.py
tests/            # pytest tests
config/           # settings.yaml
urdf/             # URDF robot model and optional meshes
esp32/            # Arduino/ESP-IDF firmware for ESP32
  real_steel_controller/  # Main sketch + headers (servo_controller.h, serial_handler.h, config.h)
data/             # Calibration data and motion recordings
docs/milestones/  # Spec-driven milestone plans (M0.md, M1.md, ...)
```

## Build & Run Commands

### Python Environment Setup
```bash
python3 -m venv real_steel
source real_steel/bin/activate
pip install -r requirements.txt
```

### Dependencies
```
opencv-python>=4.8.0
mediapipe>=0.10.0
numpy>=1.24.0
pyserial>=3.5
pybullet>=3.2.5
pyyaml>=6.0
matplotlib>=3.7.0
```

### Running
```bash
# Simulation mode
python src/main.py --sim

# Real hardware mode
python src/main.py --port /dev/tty.usbserial-XXXX

# With config file
python src/main.py --sim --config config/settings.yaml

# No visualization
python src/main.py --sim --no-viz
```

### Tests
```bash
# Run all tests
pytest tests/

# Run a single test file
pytest tests/test_pose.py

# Run a single test
pytest tests/test_pose.py::test_function_name
```

### ESP32 Firmware
Build and flash using Arduino IDE or PlatformIO. The main sketch is `esp32/real_steel_controller/real_steel_controller.ino`.

## Serial Protocol (Mac ↔ ESP32)

Commands (Mac → ESP32):
- `J:<a0>,<a1>,<a2>,<a3>,<a4>,<a5>,<a6>` — Set all 7 joints (degrees)
- `S:<joint>:<angle>` — Set single joint
- `H` — Home position
- `Q` — Query current positions
- `E:<0|1>` — Enable/disable servos

Responses (ESP32 → Mac):
- `OK` — Success
- `ERR:<code>:<msg>` — Error (codes: 1=invalid format, 2=angle OOR, 3=bad joint, 4=servo error)
- `P:<a0>,...,<a5>` — Position report
- `READY` — Boot complete

## Joint Configuration

| Joint | Index | Range | URDF Name |
|-------|-------|-------|-----------|
| L Shoulder Tilt | 0 | -90° to +90° | `l_shoulder_tilt_joint` |
| L Shoulder Pan | 1 | -90° to +90° | `l_shoulder_pan_joint` |
| L Elbow | 2 | 0° to +135° | `l_elbow_joint` |
| R Shoulder Tilt | 3 | -90° to +90° | `r_shoulder_tilt_joint` |
| R Shoulder Pan | 4 | -90° to +90° | `r_shoulder_pan_joint` |
| R Elbow | 5 | 0° to +135° | `r_elbow_joint` |
| Torso Yaw | 6 | -90° to +90° | `torso_yaw_joint` |

## Safety

- ESC key sends `E:0` to disable servos
- Serial disconnect: ESP32 holds position 2s then homes
- Pose estimation failure: freeze last valid positions
- Angles are always clamped to joint limits before sending
- Max continuous runtime: 10 minutes (servo thermal limit)
- Safe startup: connect serial → verify `READY` → `E:0` → power servos → `H` → `E:1`

## MediaPipe API (Important)

The PRD references the legacy `mp.solutions.pose` API which no longer exists in MediaPipe >= 0.10.30. Use the tasks API instead:

```python
from mediapipe.tasks.python import BaseOptions, vision

options = vision.PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="data/pose_landmarker_lite.task"),
    num_poses=1,
)
landmarker = vision.PoseLandmarker.create_from_options(options)
result = landmarker.detect(mp_image)  # returns result.pose_landmarks
```

The model file is downloaded by `scripts/setup.sh` to `data/pose_landmarker_lite.task`.

## Design Principles

1. **Simulation-first** — Always validate in PyBullet before hardware
2. **Incremental complexity** — MVP has no ROS2; Phase B migrates to ROS2 Humble on Ubuntu
3. **Data-centric** — Treat sensor streams as data pipelines
4. **Working system first** — Build end-to-end, then iterate
