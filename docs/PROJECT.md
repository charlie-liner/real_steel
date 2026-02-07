# Real Steel: Shadow Boxing Robot PRD

**Project Name:** Real Steel  
**Document Version:** 1.1  
**Last Updated:** 2025-01-30  
**Author:** Charlie  

---

## 1. Executive Summary

### 1.1 Vision
Build a shadow boxing humanoid robot that tracks and reproduces human boxing movements in real-time, inspired by "ATOM" from the movie Real Steel.

### 1.2 Purpose
- Transition from software/data engineering to embodied intelligence
- Explore robotics research problems from a data engineering perspective
- Build a technically rigorous portfolio for robotics graduate school applications
- Identify intersections between data engineering and robotics research

### 1.3 Success Criteria (MVP)
A dual-arm robot that mirrors human punching motions with less than 500ms latency, validated in both simulation and real hardware.

---

## 2. Project Philosophy

| Principle | Description |
|-----------|-------------|
| **Problem-first learning** | Learn only what's necessary to solve concrete problems |
| **Black-box → First-principles** | Start with libraries, then re-implement core algorithms |
| **Data-centric robotics** | Treat sensor streams and motion data as data pipelines |
| **Performance-aware design** | Always consider latency, throughput, and real-time constraints |
| **Working system first** | Build end-to-end, then iterate and improve |
| **Incremental complexity** | Start simple (no ROS2), add architecture later |
| **Simulation-first development** | Test in PyBullet before deploying to hardware |

---

## 3. System Architecture

### 3.1 MVP Architecture (Phase A — No ROS2)

```
┌────────────────────────────────────────────────────────────────────────────┐
│                              MAC (PYTHON)                                   │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌───────────────┐ │
│  │  Camera  │───▶│ Pose         │───▶│ Joint Angle │───▶│ Motion        │ │
│  │  Input   │    │ Estimation   │    │ Extraction  │    │ Mapping       │ │
│  │ (OpenCV) │    │ (RTMW3D/     │    │             │    │               │ │
│  │          │    │  MMPose)     │    │             │    │               │ │
│  └──────────┘    └──────────────┘    └─────────────┘    └───────┬───────┘ │
│                                                                  │         │
│                                                    ┌─────────────┴───────┐ │
│                                                    │  Robot Interface    │ │
│                                                    │  (Abstract)         │ │
│                                                    └──┬──────────────┬───┘ │
│                                                       │              │     │
│                              ┌────────────────────────┘              │     │
│                              │                                       │     │
│                              ▼                                       ▼     │
│                    ┌──────────────────┐               ┌─────────────────┐ │
│                    │  PyBullet        │               │  Serial Comm    │ │
│                    │  Simulator       │               │  (pyserial)     │ │
│                    │                  │               │                 │ │
│                    │  ┌────────────┐  │               └────────┬────────┘ │
│                    │  │   URDF     │  │                        │         │
│                    │  │   Model    │  │                        │         │
│                    │  └────────────┘  │                        │         │
│                    └──────────────────┘                        │         │
│                      (Development/Test)                        │         │
│                                                                │         │
└────────────────────────────────────────────────────────────────┼─────────┘
                                                                 │
                                                        USB Serial
                                                                 │
┌────────────────────────────────────────────────────────────────┼─────────┐
│                              ESP32                             ▼         │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────────┐ │
│  │ Serial       │───▶│ Command      │───▶│ PCA9685 PWM Driver           │ │
│  │ Receiver     │    │ Parser       │    │ (I2C)                        │ │
│  └──────────────┘    └──────────────┘    └──────────────┬───────────────┘ │
│                                                          │                 │
│                                           ┌──────────────┼──────────────┐  │
│                                           ▼              ▼              ▼  │
│                                       [Servo 0-2]   [Servo 3-5]            │
│                                       Left Arm      Right Arm              │
│                                                                             │
└────────────────────────────────────────────────────────────────────────────┘
                                         (Production)
```

### 3.2 Development Workflow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         DEVELOPMENT WORKFLOW                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   STEP 1: Develop & Test in Simulation                                      │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                                                                      │   │
│   │   Camera ──▶ Pose ──▶ Angles ──▶ Mapping ──▶ PyBullet Robot         │   │
│   │                                                                      │   │
│   │   • Fast iteration (no hardware setup)                              │   │
│   │   • Test edge cases safely                                          │   │
│   │   • Visualize robot movement in 3D                                  │   │
│   │   • Record & replay sessions                                        │   │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                         │
│                                    ▼                                         │
│   STEP 2: Deploy to Real Hardware                                           │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                                                                      │   │
│   │   Camera ──▶ Pose ──▶ Angles ──▶ Mapping ──▶ ESP32 ──▶ Real Robot   │   │
│   │                                                                      │   │
│   │   • Same code, different robot interface                            │   │
│   │   • Fine-tune for real-world physics                                │   │
│   │   • Calibrate servo positions                                       │   │
│   │                                                                      │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.3 Data Flow (MVP)

```
Camera Frame (30fps)
       │
       ▼
┌─────────────────┐
│ RTMW3D (MMPose) │  ~10-30ms
└────────┬────────┘
         │ 133 keypoints (x, y, z) with dedicated z-axis branch
         ▼
┌─────────────────┐
│ Angle Extractor │  ~1ms
└────────┬────────┘
         │ 6 joint angles (radians) ← Internal standard unit
         ▼
┌─────────────────┐
│ Motion Mapper   │  ~1ms
│ (mirror, scale) │
└────────┬────────┘
         │ 6 servo angles (radians)
         ▼
┌─────────────────┐
│ Robot Interface │  Abstraction layer
└────────┬────────┘
         │
    ┌────┴────┐
    ▼         ▼
┌───────┐ ┌────────┐
│PyBullet│ │ Serial │  ← Converts to degrees at ESP32 boundary
│  ~1ms │ │  ~5ms  │
└───────┘ └────────┘
```

**Unit Convention:** All internal processing uses **radians**. Conversion to degrees happens only at the ESP32 serial interface boundary.

### 3.4 Future Architecture (Phase B — With ROS2)

```
┌────────────────────────────────────────────────────────────────────────────┐
│                         LINUX (NATIVE OR DOCKER)                            │
├────────────────────────────────────────────────────────────────────────────┤
│                              ROS2 HUMBLE                                    │
│                                                                             │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌───────────────┐ │
│  │ Camera   │───▶│ Perception   │───▶│ Angle       │───▶│ Mapping       │ │
│  │ Node     │    │ Node         │    │ Node        │    │ Node          │ │
│  └──────────┘    └──────────────┘    └─────────────┘    └───────┬───────┘ │
│       │                │                    │                    │         │
│       ▼                ▼                    ▼                    ▼         │
│  /camera/image    /skeleton           /human_joints       /robot_joints    │
│                                                                  │         │
│                                                    ┌─────────────┴───────┐ │
│                                                    │ Serial Bridge Node  │ │
│                                                    │ (ROS2 ↔ ESP32)      │ │
│                                                    └──────────┬──────────┘ │
└───────────────────────────────────────────────────────────────┼────────────┘
                                                                │
                                                       USB Serial
                                                                │
┌───────────────────────────────────────────────────────────────┼────────────┐
│                              ESP32                            ▼            │
│                      (Same as MVP - no changes)                            │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Robot Specifications

### 4.1 Joint Configuration (6 DOF)

```
                    [FRONT VIEW]
                    
                         * (nose - reference)
                        /|\
                       / | \
    L_SHOULDER_PAN ──●   |   ●── R_SHOULDER_PAN
    L_SHOULDER_TILT ─┼───┼───┼── R_SHOULDER_TILT
                     │   │   │
    L_ELBOW ─────────●   │   ●── R_ELBOW
                     │   │   │
                     ▼   │   ▼
                   (fist)│(fist)
                         │
                        ═══ (base/torso - fixed for MVP)
```

| Joint | Axis | Range | Servo # | URDF Joint Name |
|-------|------|-------|---------|-----------------|
| L_SHOULDER_PAN | Z (horizontal) | -90° to +90° | 0 | `l_shoulder_pan_joint` |
| L_SHOULDER_TILT | Y (vertical) | -45° to +135° | 1 | `l_shoulder_tilt_joint` |
| L_ELBOW | Y | 0° to +135° | 2 | `l_elbow_joint` |
| R_SHOULDER_PAN | Z (horizontal) | -90° to +90° | 3 | `r_shoulder_pan_joint` |
| R_SHOULDER_TILT | Y (vertical) | -45° to +135° | 4 | `r_shoulder_tilt_joint` |
| R_ELBOW | Y | 0° to +135° | 5 | `r_elbow_joint` |

### 4.2 Robot Dimensions (for URDF)

```
                 [SIDE VIEW]                      [FRONT VIEW]
                 
                    0.05m                            0.30m
                   ┌─────┐                    ┌───────────────────┐
                   │ HEAD│                    │                   │
                   └──┬──┘                    │    ┌───────┐      │
                      │                       │    │ TORSO │      │
              ┌───────┴───────┐               │    │0.1x0.2│      │  0.25m
              │     TORSO     │               │    └───┬───┘      │
              │   0.10 deep   │               │        │          │
              └───────┬───────┘               │   ┌────┴────┐     │
                      │                       │   ▼         ▼     │
              ┌───────┴───────┐               │  ARM       ARM    │
              │   UPPER ARM   │ 0.15m         │  0.15m     0.15m  │
              │   0.04 wide   │               │   │         │     │
              └───────┬───────┘               │   ●         ●     │ (elbow)
                      │                       │   │         │     │
              ┌───────┴───────┐               │  0.12m     0.12m  │
              │   FOREARM     │ 0.12m         │   │         │     │
              │   0.035 wide  │               │   ▼         ▼     │
              └───────┬───────┘               │ (fist)   (fist)   │
                      │                       │                   │
                   (fist)                     └───────────────────┘
```

| Link | Dimensions (m) | Mass (kg) | Notes |
|------|----------------|-----------|-------|
| base_link | 0.10 × 0.20 × 0.25 | 0.5 | Fixed torso |
| l/r_upper_arm | 0.04 × 0.04 × 0.15 | 0.1 | Shoulder to elbow |
| l/r_forearm | 0.035 × 0.035 × 0.12 | 0.08 | Elbow to fist |
| l/r_hand | 0.04 × 0.04 × 0.04 | 0.05 | Fist (simplified) |

### 4.3 Hardware Specifications

| Specification | Value | Notes |
|---------------|-------|-------|
| Degrees of Freedom | 6 DOF (dual arm) | 3 per arm |
| Servos | 6× MG996R (or similar) | ~10kg·cm torque |
| Controller | ESP32 DevKit C | Existing hardware |
| PWM Driver | PCA9685 | 16-channel, I2C |
| Power Supply | 6V 5A (servos) | Separate from ESP32 |
| Control Frequency | 50 Hz | 20ms update rate |
| Communication | USB Serial (115200 baud) | Mac ↔ ESP32 |

### 4.4 Performance Targets

| Metric | MVP Target | Stretch Goal |
|--------|------------|--------------|
| Pose estimation FPS | ≥15 Hz | ≥30 Hz |
| End-to-end latency | <500ms | <200ms |
| Software latency | <100ms | <50ms |
| Joint position accuracy | ±10° | ±5° |
| Continuous operation | 10 min | 30 min |
| Simulation FPS | ≥60 Hz | ≥240 Hz |

---

## 5. Technology Stack

### 5.1 MVP Stack (Phase A)

| Component | Technology | Rationale |
|-----------|------------|-----------|
| Dev Machine | Mac (native) | Available, no VM overhead |
| Language | Python 3.10+ | Rapid prototyping |
| Camera | Built-in webcam / USB | Simplest option |
| Pose Estimation | RTMW3D (MMPose) | Real-time 3D pose with dedicated z-axis branch; replaces MediaPipe which had unreliable monocular depth for forward punches |
| **Simulation** | **PyBullet** | No ROS required, Mac native, URDF support |
| **Robot Model** | **URDF** | Standard format, reusable in Gazebo later |
| Visualization | OpenCV, Matplotlib, PyBullet GUI | Debugging |
| Serial Communication | pyserial | Simple, reliable |
| Microcontroller | ESP32 DevKit C | Available, sufficient |
| PWM Driver | PCA9685 via I2C | Clean servo control |
| ESP32 Framework | Arduino or ESP-IDF | Arduino for simplicity |

### 5.2 Python Dependencies

```
# requirements.txt
opencv-python>=4.8.0
mmpose>=1.3.0
mmengine>=0.10.0
mmdet>=3.2.0
mmcv>=2.1.0
numpy>=1.24.0
pyserial>=3.5
pybullet>=3.2.5
pyyaml>=6.0
matplotlib>=3.7.0  # optional, for plotting
```

> **Note:** MediaPipe was used in earlier milestones but has been replaced by RTMW3D (MMPose) due to unreliable monocular z-depth estimation. MediaPipe's z-axis noise (~3x worse than x/y) made forward punch detection (jabs, crosses) impossible to track accurately. RTMW3D provides a dedicated z-axis prediction branch that resolves this limitation. See Decisions Log (Section 11) for details.

### 5.3 Future Stack (Phase B)

| Component | Technology | Rationale |
|-----------|------------|-----------|
| OS | Ubuntu 22.04 (or Docker) | ROS2 compatibility |
| Middleware | ROS2 Humble | Industry standard |
| Simulation | Gazebo (uses same URDF) | ROS2 integration |
| Language | Python + C++ | Performance optimization |
| Visualization | RViz2 | ROS2 native |

---

## 6. Communication Protocol

### 6.1 Serial Protocol Specification

**Baud Rate:** 115200  
**Line Ending:** `\n` (newline)  
**Format:** ASCII text

#### Commands (Mac → ESP32)

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Joint Position | `J:<a0>,<a1>,<a2>,<a3>,<a4>,<a5>` | `J:45.0,-30.0,90.0,45.0,-30.0,90.0` | Set all 6 joints (degrees) |
| Single Joint | `S:<joint>:<angle>` | `S:2:90.0` | Set single joint |
| Home | `H` | `H` | Move all to home position |
| Query | `Q` | `Q` | Request current positions |
| Enable | `E:<0\|1>` | `E:1` | Enable/disable servos |

#### Responses (ESP32 → Mac)

| Response | Format | Example | Description |
|----------|--------|---------|-------------|
| OK | `OK` | `OK` | Command executed |
| Error | `ERR:<code>:<msg>` | `ERR:1:Invalid angle` | Error with details |
| Position | `P:<a0>,<a1>,<a2>,<a3>,<a4>,<a5>` | `P:45.0,-30.0,90.0,...` | Current positions |
| Ready | `READY` | `READY` | ESP32 boot complete |

#### Error Codes

| Code | Meaning |
|------|---------|
| 1 | Invalid command format |
| 2 | Angle out of range |
| 3 | Invalid joint index |
| 4 | Servo communication error |

### 6.2 Message Timing

```
Mac                          ESP32
 │                             │
 │  J:45.0,-30.0,90.0,...\n   │
 │ ───────────────────────▶   │
 │                             │ parse (~1ms)
 │                             │ update PWM (~1ms)
 │            OK\n             │
 │ ◀───────────────────────   │
 │                             │
 
Round-trip target: < 10ms
```

---

## 7. Milestone Breakdown

### Milestone 0: Development Environment Setup
**Duration:** 3-5 days  
**Goal:** Ready to write and test code

#### Deliverables
- [ ] Python 3.10+ environment (venv or conda)
- [ ] Install dependencies: opencv-python, mmpose, mmengine, mmdet, mmcv, pyserial, numpy, pybullet
- [ ] ESP32 Arduino IDE or PlatformIO setup
- [ ] Git repository initialized
- [ ] Camera test script working
- [ ] PyBullet test script working (load sample robot)
- [ ] ESP32 "hello world" (blink LED)

#### Verification Scripts

**Test Camera:**
```python
# test_camera.py
import cv2
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    cv2.imshow('Camera Test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
```

**Test PyBullet:**
```python
# test_pybullet.py
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load sample robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
print(f"Loaded robot with {p.getNumJoints(robot)} joints")

# Simulate for 5 seconds
for _ in range(240 * 5):
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()
```

#### Directory Structure
```
real-steel/
├── README.md
├── docs/
│   ├── prd.md
│   ├── wiring-diagram.md
│   └── urdf-spec.md
├── mac/
│   ├── requirements.txt
│   ├── src/
│   │   ├── main.py
│   │   ├── camera.py
│   │   ├── pose_estimator.py
│   │   ├── angle_calculator.py
│   │   ├── motion_mapper.py
│   │   ├── robot_interface.py      # Abstract interface
│   │   ├── simulated_robot.py      # PyBullet implementation
│   │   ├── real_robot.py           # Serial/ESP32 implementation
│   │   └── visualizer.py
│   ├── tests/
│   │   ├── test_camera.py
│   │   ├── test_pybullet.py
│   │   ├── test_pose.py
│   │   └── test_serial.py
│   └── config/
│       └── settings.yaml
├── urdf/
│   ├── real_steel.urdf
│   └── meshes/                     # Optional: custom 3D models
│       ├── base.stl
│       ├── upper_arm.stl
│       └── forearm.stl
├── esp32/
│   ├── real_steel_controller/
│   │   ├── real_steel_controller.ino
│   │   ├── servo_controller.h
│   │   ├── serial_handler.h
│   │   └── config.h
│   └── test_sketches/
│       ├── test_servo/
│       └── test_serial/
├── hardware/
│   ├── bom.md
│   ├── wiring/
│   └── 3d_models/
├── data/
│   ├── calibration/
│   └── recordings/
└── scripts/
    ├── setup.sh
    └── run_sim.sh
```

#### Acceptance Criteria
- Can capture webcam frame and display with OpenCV
- RTMW3D pose estimation runs on webcam feed
- PyBullet GUI opens and displays sample robot
- ESP32 responds to serial commands
- Git repo has initial commit

---

### Milestone 1: Robot Model & Simulation
**Duration:** 1-1.5 weeks  
**Goal:** Controllable robot in PyBullet simulation

#### 1.1 URDF Model Creation
**Duration:** 3-4 days

##### Tasks
- [ ] Create base URDF structure
- [ ] Define all 6 joints with correct axes and limits
- [ ] Add visual geometries (boxes/cylinders for MVP)
- [ ] Add collision geometries
- [ ] Add inertial properties
- [ ] Validate URDF loads in PyBullet
- [ ] Test joint movements manually

##### URDF File: `urdf/real_steel.urdf`
```xml
<?xml version="1.0"?>
<robot name="real_steel">
  
  <!-- ==================== MATERIALS ==================== -->
  <material name="blue">
    <color rgba="0.2 0.4 0.8 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.2 0.2 1.0"/>
  </material>
  
  <!-- ==================== BASE/TORSO ==================== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.10 0.20 0.25"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.10 0.20 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- ==================== LEFT ARM ==================== -->
  
  <!-- Left Shoulder Pan (horizontal rotation) -->
  <link name="l_shoulder_pan_link">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="l_shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="l_shoulder_pan_link"/>
    <origin xyz="0.0 0.12 0.10" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- Left Shoulder Tilt (vertical rotation) -->
  <link name="l_shoulder_tilt_link">
    <visual>
      <origin xyz="0 0 -0.075"/>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075"/>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="l_shoulder_tilt_joint" type="revolute">
    <parent link="l_shoulder_pan_link"/>
    <child link="l_shoulder_tilt_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7854" upper="2.3562" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- Left Elbow -->
  <link name="l_forearm_link">
    <visual>
      <origin xyz="0 0 -0.06"/>
      <geometry>
        <cylinder radius="0.0175" length="0.12"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.06"/>
      <geometry>
        <cylinder radius="0.0175" length="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.08"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00005"/>
    </inertial>
  </link>
  
  <joint name="l_elbow_joint" type="revolute">
    <parent link="l_shoulder_tilt_link"/>
    <child link="l_forearm_link"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.3562" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- Left Hand -->
  <link name="l_hand_link">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00002" ixy="0" ixz="0" iyy="0.00002" iyz="0" izz="0.00002"/>
    </inertial>
  </link>
  
  <joint name="l_hand_joint" type="fixed">
    <parent link="l_forearm_link"/>
    <child link="l_hand_link"/>
    <origin xyz="0 0 -0.12" rpy="0 0 0"/>
  </joint>
  
  <!-- ==================== RIGHT ARM ==================== -->
  
  <!-- Right Shoulder Pan -->
  <link name="r_shoulder_pan_link">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="r_shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="r_shoulder_pan_link"/>
    <origin xyz="0.0 -0.12 0.10" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- Right Shoulder Tilt -->
  <link name="r_shoulder_tilt_link">
    <visual>
      <origin xyz="0 0 -0.075"/>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075"/>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="r_shoulder_tilt_joint" type="revolute">
    <parent link="r_shoulder_pan_link"/>
    <child link="r_shoulder_tilt_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7854" upper="2.3562" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- Right Elbow -->
  <link name="r_forearm_link">
    <visual>
      <origin xyz="0 0 -0.06"/>
      <geometry>
        <cylinder radius="0.0175" length="0.12"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.06"/>
      <geometry>
        <cylinder radius="0.0175" length="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.08"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00005"/>
    </inertial>
  </link>
  
  <joint name="r_elbow_joint" type="revolute">
    <parent link="r_shoulder_tilt_link"/>
    <child link="r_forearm_link"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.3562" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- Right Hand -->
  <link name="r_hand_link">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00002" ixy="0" ixz="0" iyy="0.00002" iyz="0" izz="0.00002"/>
    </inertial>
  </link>
  
  <joint name="r_hand_joint" type="fixed">
    <parent link="r_forearm_link"/>
    <child link="r_hand_link"/>
    <origin xyz="0 0 -0.12" rpy="0 0 0"/>
  </joint>

</robot>
```

##### Acceptance Criteria
- URDF loads in PyBullet without errors
- Robot appears correctly positioned (torso upright, arms at sides)
- All 6 joints can be controlled individually
- Joint limits are respected

#### 1.2 Simulation Interface
**Duration:** 2-3 days

##### Tasks
- [ ] Create `RobotInterface` abstract class
- [ ] Implement `SimulatedRobot` class using PyBullet
- [ ] Add joint position control
- [ ] Add joint state reading
- [ ] Add simulation stepping with configurable timestep
- [ ] Create test script for manual joint control

##### Code: `robot_interface.py`
```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np

@dataclass
class JointState:
    positions: np.ndarray  # 6 joint angles (radians)
    velocities: np.ndarray  # 6 joint velocities
    timestamp: float

class RobotInterface(ABC):
    """Abstract interface for robot control - works with sim or real hardware"""
    
    NUM_JOINTS = 6
    JOINT_NAMES = [
        'l_shoulder_pan', 'l_shoulder_tilt', 'l_elbow',
        'r_shoulder_pan', 'r_shoulder_tilt', 'r_elbow'
    ]
    
    @abstractmethod
    def connect(self) -> bool:
        """Initialize connection to robot"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """Clean up connection"""
        pass
    
    @abstractmethod
    def set_joint_positions(self, positions: np.ndarray):
        """Set target positions for all joints (radians)"""
        pass
    
    @abstractmethod
    def get_joint_state(self) -> JointState:
        """Get current joint positions and velocities"""
        pass
    
    @abstractmethod
    def home(self):
        """Move robot to home position"""
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """Check if robot is connected and responsive"""
        pass
```

##### Code: `simulated_robot.py`
```python
import pybullet as p
import pybullet_data
import numpy as np
import time
from robot_interface import RobotInterface, JointState

class SimulatedRobot(RobotInterface):
    """PyBullet simulation implementation of RobotInterface"""
    
    def __init__(self, urdf_path: str, gui: bool = True, timestep: float = 1/240):
        self.urdf_path = urdf_path
        self.gui = gui
        self.timestep = timestep
        self.client = None
        self.robot_id = None
        self.joint_indices = []  # Maps our joint order to URDF joint indices
        
        # Control parameters
        self.max_force = 10.0
        self.position_gain = 0.3
        self.velocity_gain = 1.0
        
    def connect(self) -> bool:
        try:
            # Connect to PyBullet
            self.client = p.connect(p.GUI if self.gui else p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.8)
            p.setTimeStep(self.timestep)
            
            # Load robot
            self.robot_id = p.loadURDF(
                self.urdf_path, 
                basePosition=[0, 0, 0.5],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=True
            )
            
            # Map joint names to indices
            self._map_joints()
            
            # Set initial position
            self.home()
            
            # Configure camera view
            if self.gui:
                p.resetDebugVisualizerCamera(
                    cameraDistance=1.0,
                    cameraYaw=45,
                    cameraPitch=-30,
                    cameraTargetPosition=[0, 0, 0.5]
                )
            
            return True
            
        except Exception as e:
            print(f"Failed to connect to simulation: {e}")
            return False
    
    def _map_joints(self):
        """Map our joint names to PyBullet joint indices"""
        joint_name_to_index = {}
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode('utf-8')
            joint_name_to_index[name] = i
        
        # Map in our expected order
        urdf_joint_names = [
            'l_shoulder_pan_joint', 'l_shoulder_tilt_joint', 'l_elbow_joint',
            'r_shoulder_pan_joint', 'r_shoulder_tilt_joint', 'r_elbow_joint'
        ]
        
        self.joint_indices = []
        for name in urdf_joint_names:
            if name in joint_name_to_index:
                self.joint_indices.append(joint_name_to_index[name])
            else:
                raise ValueError(f"Joint {name} not found in URDF")
    
    def disconnect(self):
        if self.client is not None:
            p.disconnect(self.client)
            self.client = None
    
    def set_joint_positions(self, positions: np.ndarray):
        """Set target positions for all joints (radians)"""
        if len(positions) != self.NUM_JOINTS:
            raise ValueError(f"Expected {self.NUM_JOINTS} positions, got {len(positions)}")
        
        for i, pos in enumerate(positions):
            p.setJointMotorControl2(
                self.robot_id,
                self.joint_indices[i],
                p.POSITION_CONTROL,
                targetPosition=pos,
                force=self.max_force,
                positionGain=self.position_gain,
                velocityGain=self.velocity_gain
            )
    
    def get_joint_state(self) -> JointState:
        positions = []
        velocities = []
        
        for idx in self.joint_indices:
            state = p.getJointState(self.robot_id, idx)
            positions.append(state[0])
            velocities.append(state[1])
        
        return JointState(
            positions=np.array(positions),
            velocities=np.array(velocities),
            timestamp=time.time()
        )
    
    def home(self):
        """Move to home position (arms down at sides)"""
        home_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.set_joint_positions(home_positions)
        
        # Step simulation to reach position
        for _ in range(100):
            self.step()
    
    def step(self):
        """Advance simulation by one timestep"""
        p.stepSimulation()
    
    def is_connected(self) -> bool:
        return self.client is not None and p.isConnected(self.client)
    
    def run_realtime(self, duration: float = None):
        """Run simulation in real-time (useful for visualization)"""
        start = time.time()
        while duration is None or (time.time() - start) < duration:
            self.step()
            time.sleep(self.timestep)
```

##### Test Script: `tests/test_simulation.py`
```python
import numpy as np
import time
import sys
sys.path.append('../src')

from simulated_robot import SimulatedRobot

def test_joint_control():
    robot = SimulatedRobot(urdf_path='../urdf/real_steel.urdf', gui=True)
    
    if not robot.connect():
        print("Failed to connect!")
        return
    
    print("Testing joint control...")
    print("Press Ctrl+C to exit")
    
    try:
        # Test sequence: wave left arm
        sequences = [
            # [l_pan, l_tilt, l_elbow, r_pan, r_tilt, r_elbow]
            [0.0, 0.5, 0.5, 0.0, 0.0, 0.0],   # Raise left arm
            [0.5, 0.5, 1.0, 0.0, 0.0, 0.0],   # Swing left
            [-0.5, 0.5, 1.0, 0.0, 0.0, 0.0],  # Swing right
            [0.0, 0.5, 0.5, 0.0, 0.0, 0.0],   # Center
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # Home
        ]
        
        for target in sequences:
            print(f"Moving to: {target}")
            robot.set_joint_positions(np.array(target))
            
            # Run simulation for 1 second
            for _ in range(240):
                robot.step()
                time.sleep(1/240)
            
            state = robot.get_joint_state()
            print(f"Current position: {state.positions}")
        
        print("\nTest complete! Simulation will close in 3 seconds...")
        time.sleep(3)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        robot.disconnect()

if __name__ == '__main__':
    test_joint_control()
```

##### Acceptance Criteria
- Robot spawns correctly in PyBullet
- All 6 joints respond to position commands
- Joint state feedback matches commanded positions
- Simulation runs smoothly at real-time speed

#### 1.3 Interactive Control (Optional Enhancement)
**Duration:** 1 day

##### Tasks
- [ ] Add keyboard control for manual testing
- [ ] Add PyBullet debug sliders for each joint
- [ ] Add reset button

##### Code Addition to `simulated_robot.py`
```python
def add_debug_controls(self):
    """Add GUI sliders for manual joint control"""
    self.sliders = []
    joint_limits = [
        (-90, 90), (-45, 135), (0, 135),  # Left arm (degrees)
        (-90, 90), (-45, 135), (0, 135),  # Right arm (degrees)
    ]
    
    for i, name in enumerate(self.JOINT_NAMES):
        low, high = joint_limits[i]
        slider = p.addUserDebugParameter(name, low, high, 0)
        self.sliders.append(slider)
    
    # Reset button
    self.reset_button = p.addUserDebugParameter("RESET", 1, 0, 0)

def read_debug_controls(self) -> np.ndarray:
    """Read current slider values and return as radians"""
    positions_deg = [p.readUserDebugParameter(s) for s in self.sliders]
    return np.deg2rad(positions_deg)
```

---

### Milestone 2: Perception Pipeline (Mac)
**Duration:** 1.5-2 weeks  
**Goal:** Extract joint angles from camera feed

#### 2.1 Camera Setup
**Duration:** 1-2 days

##### Tasks
- [ ] Implement camera capture class (OpenCV VideoCapture)
- [ ] Handle camera selection (built-in vs USB)
- [ ] Configure resolution and FPS (640×480, 30fps recommended)
- [ ] Add frame timestamp for latency tracking
- [ ] Basic error handling (camera disconnect)

##### Code: `camera.py`
```python
import cv2
import time
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

@dataclass
class Frame:
    image: np.ndarray
    timestamp: float
    frame_number: int

class Camera:
    def __init__(self, device_id: int = 0, width: int = 640, height: int = 480, fps: int = 30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.cap: Optional[cv2.VideoCapture] = None
        self.frame_count = 0
        
    def open(self) -> bool:
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            return False
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        return True
    
    def read(self) -> Optional[Frame]:
        if self.cap is None:
            return None
        
        ret, image = self.cap.read()
        if not ret:
            return None
        
        self.frame_count += 1
        return Frame(
            image=image,
            timestamp=time.time(),
            frame_number=self.frame_count
        )
    
    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
    
    def is_opened(self) -> bool:
        return self.cap is not None and self.cap.isOpened()
```

##### Acceptance Criteria
- Camera opens without error
- Frames captured at expected FPS
- Graceful handling when camera unavailable

#### 2.2 Pose Estimation
**Duration:** 2-3 days

> **Migration Note:** This section originally specified MediaPipe BlazePose. The project has migrated to **RTMW3D (MMPose)** because MediaPipe's monocular z-depth estimation was unreliable for forward punch detection (jabs, crosses). RTMW3D provides a dedicated z-axis prediction branch with significantly better depth accuracy. See Decisions Log (Section 11).

##### Tasks
- [ ] Integrate RTMW3D from MMPose
- [ ] Download RTMW3D pretrained checkpoint (rtmw3d-l for accuracy, rtmw3d-s for speed)
- [ ] Extract upper body keypoints (shoulders, elbows, wrists) from RTMW3D's COCO-format output
- [ ] Handle detection confidence threshold
- [ ] Add visualization overlay

##### Keypoints of Interest (COCO format used by RTMW3D)
```python
KEYPOINTS = {
    5: 'left_shoulder',
    6: 'right_shoulder',
    7: 'left_elbow',
    8: 'right_elbow',
    9: 'left_wrist',
    10: 'right_wrist',
}
```

##### Code: `pose_estimator.py`
```python
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional
import cv2

# MMPose inference API
from mmpose.apis import init_model, inference_topdown
from mmpose.structures import PoseDataSample

@dataclass
class Point3D:
    x: float            # normalized [0, 1]
    y: float
    z: float            # depth (from RTMW3D's dedicated z-axis branch)
    visibility: float
    world_x: float      # meters (estimated from 3D prediction)
    world_y: float
    world_z: float

@dataclass
class PoseResult:
    keypoints: Dict[str, Point3D]
    is_valid: bool
    timestamp: float

    def get_point(self, name: str) -> Optional[Point3D]:
        return self.keypoints.get(name)

class PoseEstimator:
    # COCO keypoint indices used by RTMW3D
    KEYPOINT_INDICES = {
        5: 'left_shoulder',
        6: 'right_shoulder',
        7: 'left_elbow',
        8: 'right_elbow',
        9: 'left_wrist',
        10: 'right_wrist',
    }

    def __init__(self, config_path: str, checkpoint_path: str,
                 min_visibility: float = 0.5, device: str = 'cpu'):
        self.min_visibility = min_visibility
        self._last_valid_result: PoseResult | None = None
        self.model = init_model(config_path, checkpoint_path, device=device)

    def process(self, image: np.ndarray, timestamp: float) -> PoseResult:
        # RTMW3D inference — returns 3D keypoints with dedicated z-axis predictions
        results = inference_topdown(self.model, image)

        keypoints: Dict[str, Point3D] = {}
        is_valid = False

        if results and len(results) > 0:
            pred = results[0].pred_instances
            kpts_2d = pred.keypoints[0]       # (K, 2) — x, y in pixels
            kpts_3d = pred.keypoints_3d[0]    # (K, 3) — x, y, z in mm (from z-axis branch)
            scores = pred.keypoint_scores[0]  # (K,)

            h, w = image.shape[:2]
            for idx, name in self.KEYPOINT_INDICES.items():
                keypoints[name] = Point3D(
                    x=kpts_2d[idx][0] / w,
                    y=kpts_2d[idx][1] / h,
                    z=kpts_3d[idx][2] / 1000.0,   # mm to meters
                    visibility=float(scores[idx]),
                    world_x=kpts_3d[idx][0] / 1000.0,
                    world_y=kpts_3d[idx][1] / 1000.0,
                    world_z=kpts_3d[idx][2] / 1000.0,
                )

            is_valid = all(
                keypoints[name].visibility >= self.min_visibility
                for name in self.KEYPOINT_INDICES.values()
            )

        result = PoseResult(keypoints=keypoints, is_valid=is_valid, timestamp=timestamp)

        if is_valid:
            self._last_valid_result = result
            return result

        if self._last_valid_result is not None:
            return PoseResult(
                keypoints=self._last_valid_result.keypoints,
                is_valid=True,
                timestamp=timestamp,
            )
        return result

    def draw(self, image: np.ndarray, pose: PoseResult) -> np.ndarray:
        """Draw pose overlay on image"""
        output = image.copy()
        if not pose.keypoints:
            return output
        h, w = image.shape[:2]
        for name, point in pose.keypoints.items():
            if point.visibility >= self.min_visibility:
                cx, cy = int(point.x * w), int(point.y * h)
                color = (0, 255, 0) if point.visibility > 0.8 else (0, 165, 255)
                cv2.circle(output, (cx, cy), 5, color, -1)
                cv2.putText(output, name.split('_')[0][0].upper(),
                           (cx+5, cy-5), cv2.FONT_HERSHEY_SIMPLEX,
                           0.4, color, 1)
        bones = [
            ('left_shoulder', 'left_elbow'),
            ('left_elbow', 'left_wrist'),
            ('right_shoulder', 'right_elbow'),
            ('right_elbow', 'right_wrist'),
            ('left_shoulder', 'right_shoulder'),
        ]
        for start_name, end_name in bones:
            start = pose.keypoints.get(start_name)
            end = pose.keypoints.get(end_name)
            if start and end:
                if start.visibility >= self.min_visibility and end.visibility >= self.min_visibility:
                    pt1 = (int(start.x * w), int(start.y * h))
                    pt2 = (int(end.x * w), int(end.y * h))
                    cv2.line(output, pt1, pt2, (0, 255, 0), 2)
        return output

    def close(self) -> None:
        pass  # MMPose models don't require explicit cleanup
```

##### Why RTMW3D over MediaPipe
| Aspect | MediaPipe BlazePose | RTMW3D (MMPose) |
|--------|-------------------|-----------------|
| Z-depth accuracy | Poor (~3x noisier than x/y) | Dedicated z-axis prediction branch |
| Forward punch detection | Unreliable (depth ambiguity) | Reliable (trained on 3D motion capture data) |
| Real-time capable | Yes (~30ms CPU) | Yes (~10ms GPU, real-time on CPU) |
| Model architecture | Single-frame, per-frame z guess | SimCC decoder with z-axis head |
| Actively maintained | Limited updates | Active (OpenMMLab ecosystem) |

##### Acceptance Criteria
- Pose detected reliably when person in frame
- Processing time < 50ms per frame
- Visualization shows skeleton overlay correctly
- **Forward punch (jab/cross) detected with accurate z-axis movement**

#### 2.3 Joint Angle Calculation
**Duration:** 3-4 days

##### Tasks
- [ ] Implement 3D vector math utilities
- [ ] Calculate shoulder pan angle (horizontal rotation)
- [ ] Calculate shoulder tilt angle (vertical rotation)
- [ ] Calculate elbow angle (flexion)
- [ ] Handle coordinate system conversion
- [ ] Implement noise filtering (exponential moving average)
- [ ] Add angle visualization

##### Code: `angle_calculator.py`
```python
import numpy as np
from dataclasses import dataclass
from typing import Optional
from pose_estimator import PoseResult, Point3D

@dataclass
class JointAngles:
    """Joint angles in radians"""
    left_shoulder_pan: float
    left_shoulder_tilt: float
    left_elbow: float
    right_shoulder_pan: float
    right_shoulder_tilt: float
    right_elbow: float
    timestamp: float
    valid: np.ndarray  # 6 booleans indicating validity
    
    def to_array(self) -> np.ndarray:
        return np.array([
            self.left_shoulder_pan,
            self.left_shoulder_tilt,
            self.left_elbow,
            self.right_shoulder_pan,
            self.right_shoulder_tilt,
            self.right_elbow
        ])

class AngleCalculator:
    def __init__(self, smoothing_factor: float = 0.3):
        """
        Args:
            smoothing_factor: EMA factor (0-1). Higher = more smoothing
        """
        self.smoothing_factor = smoothing_factor
        self.prev_angles: Optional[JointAngles] = None
        
    def calculate(self, pose: PoseResult) -> Optional[JointAngles]:
        if not pose.is_valid:
            return None
        
        # Extract points
        ls = self._to_vec(pose.keypoints['left_shoulder'])
        le = self._to_vec(pose.keypoints['left_elbow'])
        lw = self._to_vec(pose.keypoints['left_wrist'])
        rs = self._to_vec(pose.keypoints['right_shoulder'])
        re = self._to_vec(pose.keypoints['right_elbow'])
        rw = self._to_vec(pose.keypoints['right_wrist'])
        
        # Calculate angles
        l_shoulder_pan = self._calc_shoulder_pan(ls, le, rs)
        l_shoulder_tilt = self._calc_shoulder_tilt(ls, le)
        l_elbow = self._calc_elbow_angle(ls, le, lw)
        
        r_shoulder_pan = self._calc_shoulder_pan(rs, re, ls)
        r_shoulder_tilt = self._calc_shoulder_tilt(rs, re)
        r_elbow = self._calc_elbow_angle(rs, re, rw)
        
        angles = JointAngles(
            left_shoulder_pan=l_shoulder_pan,
            left_shoulder_tilt=l_shoulder_tilt,
            left_elbow=l_elbow,
            right_shoulder_pan=r_shoulder_pan,
            right_shoulder_tilt=r_shoulder_tilt,
            right_elbow=r_elbow,
            timestamp=pose.timestamp,
            valid=np.ones(6, dtype=bool)
        )
        
        # Apply smoothing
        if self.prev_angles is not None:
            angles = self._smooth(angles, self.prev_angles)
        
        self.prev_angles = angles
        return angles
    
    def _to_vec(self, p: Point3D) -> np.ndarray:
        """Convert Point3D to numpy array"""
        return np.array([p.x, p.y, p.z])
    
    def _calc_shoulder_pan(self, shoulder: np.ndarray, elbow: np.ndarray, 
                           other_shoulder: np.ndarray) -> float:
        """Calculate horizontal rotation of upper arm"""
        # Torso direction (shoulder to shoulder)
        torso_dir = other_shoulder - shoulder
        torso_dir[2] = 0  # Project to XY plane
        torso_dir = torso_dir / (np.linalg.norm(torso_dir) + 1e-6)
        
        # Upper arm direction
        upper_arm = elbow - shoulder
        upper_arm[2] = 0  # Project to XY plane
        upper_arm = upper_arm / (np.linalg.norm(upper_arm) + 1e-6)
        
        # Angle between them
        dot = np.clip(np.dot(torso_dir, upper_arm), -1.0, 1.0)
        angle = np.arccos(dot)
        
        # Determine sign using cross product
        cross = np.cross(torso_dir, upper_arm)
        if cross[2] < 0:
            angle = -angle
            
        return angle
    
    def _calc_shoulder_tilt(self, shoulder: np.ndarray, elbow: np.ndarray) -> float:
        """Calculate vertical angle of upper arm from horizontal"""
        upper_arm = elbow - shoulder
        
        # Angle from horizontal plane (negative Y is down in image coords)
        horizontal_dist = np.sqrt(upper_arm[0]**2 + upper_arm[2]**2)
        angle = np.arctan2(-upper_arm[1], horizontal_dist)
        
        return angle
    
    def _calc_elbow_angle(self, shoulder: np.ndarray, elbow: np.ndarray, 
                          wrist: np.ndarray) -> float:
        """Calculate elbow flexion angle"""
        upper_arm = elbow - shoulder
        forearm = wrist - elbow
        
        # Normalize
        upper_arm = upper_arm / (np.linalg.norm(upper_arm) + 1e-6)
        forearm = forearm / (np.linalg.norm(forearm) + 1e-6)
        
        # Angle between vectors
        dot = np.clip(np.dot(upper_arm, forearm), -1.0, 1.0)
        angle = np.arccos(dot)
        
        # Elbow angle is typically measured as flexion from straight (180° - angle)
        return np.pi - angle
    
    def _smooth(self, current: JointAngles, previous: JointAngles) -> JointAngles:
        """Apply exponential moving average smoothing"""
        alpha = self.smoothing_factor
        
        return JointAngles(
            left_shoulder_pan=alpha * previous.left_shoulder_pan + (1-alpha) * current.left_shoulder_pan,
            left_shoulder_tilt=alpha * previous.left_shoulder_tilt + (1-alpha) * current.left_shoulder_tilt,
            left_elbow=alpha * previous.left_elbow + (1-alpha) * current.left_elbow,
            right_shoulder_pan=alpha * previous.right_shoulder_pan + (1-alpha) * current.right_shoulder_pan,
            right_shoulder_tilt=alpha * previous.right_shoulder_tilt + (1-alpha) * current.right_shoulder_tilt,
            right_elbow=alpha * previous.right_elbow + (1-alpha) * current.right_elbow,
            timestamp=current.timestamp,
            valid=current.valid & previous.valid
        )
    
    def reset(self):
        """Reset smoothing state"""
        self.prev_angles = None
```

##### Acceptance Criteria
- Angles update smoothly (no jitter > 5°)
- Angles match visual inspection (raise arm = angle increases)
- Handles partial occlusion gracefully

#### 2.4 Motion Mapping
**Duration:** 2-3 days

##### Tasks
- [ ] Implement human-to-robot angle mapping
- [ ] Add mirroring mode (left↔right swap)
- [ ] Apply joint limits clamping
- [ ] Add configurable scale factors
- [ ] Implement dead zone (small movements ignored)

##### Code: `motion_mapper.py`
```python
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Tuple
from angle_calculator import JointAngles

@dataclass
class MappingConfig:
    mirror_mode: bool = True
    dead_zone: float = 0.05  # radians (~3 degrees)
    
    # Joint limits in radians
    joint_limits: Dict[str, Tuple[float, float]] = field(default_factory=lambda: {
        'shoulder_pan': (-np.pi/2, np.pi/2),      # -90 to 90 deg
        'shoulder_tilt': (-np.pi/4, 3*np.pi/4),   # -45 to 135 deg  
        'elbow': (0, 3*np.pi/4),                   # 0 to 135 deg
    })
    
    # Scale factors (human angle * scale = robot angle)
    scale_factors: Dict[str, float] = field(default_factory=lambda: {
        'shoulder_pan': 1.0,
        'shoulder_tilt': 1.0,
        'elbow': 1.0,
    })

@dataclass
class ServoAngles:
    """Servo angles ready to send to robot (radians)"""
    angles: np.ndarray  # 6 values
    timestamp: float
    
    def to_degrees(self) -> np.ndarray:
        return np.rad2deg(self.angles)

class MotionMapper:
    def __init__(self, config: MappingConfig = None):
        self.config = config or MappingConfig()
        self.prev_output: np.ndarray = None
        
    def map(self, human_angles: JointAngles) -> ServoAngles:
        # Start with human angles
        if self.config.mirror_mode:
            # Swap left and right, and invert pan direction
            angles = np.array([
                -human_angles.right_shoulder_pan,  # Mirror pan
                human_angles.right_shoulder_tilt,
                human_angles.right_elbow,
                -human_angles.left_shoulder_pan,   # Mirror pan
                human_angles.left_shoulder_tilt,
                human_angles.left_elbow,
            ])
        else:
            angles = human_angles.to_array()
        
        # Apply scale factors
        scales = np.array([
            self.config.scale_factors['shoulder_pan'],
            self.config.scale_factors['shoulder_tilt'],
            self.config.scale_factors['elbow'],
            self.config.scale_factors['shoulder_pan'],
            self.config.scale_factors['shoulder_tilt'],
            self.config.scale_factors['elbow'],
        ])
        angles = angles * scales
        
        # Apply joint limits
        limits = [
            self.config.joint_limits['shoulder_pan'],
            self.config.joint_limits['shoulder_tilt'],
            self.config.joint_limits['elbow'],
            self.config.joint_limits['shoulder_pan'],
            self.config.joint_limits['shoulder_tilt'],
            self.config.joint_limits['elbow'],
        ]
        for i, (low, high) in enumerate(limits):
            angles[i] = np.clip(angles[i], low, high)
        
        # Apply dead zone
        if self.prev_output is not None:
            diff = np.abs(angles - self.prev_output)
            mask = diff < self.config.dead_zone
            angles[mask] = self.prev_output[mask]
        
        self.prev_output = angles.copy()
        
        return ServoAngles(
            angles=angles,
            timestamp=human_angles.timestamp
        )
    
    def reset(self):
        self.prev_output = None
```

##### Acceptance Criteria
- Mirrored movement looks natural
- No angles exceed servo limits
- Small movements don't cause jitter

---

### Milestone 3: ESP32 Motor Controller
**Duration:** 1.5-2 weeks  
**Goal:** ESP32 controls 6 servos via serial commands

#### 3.1 Basic ESP32 Setup
**Duration:** 2-3 days

##### Tasks
- [ ] Set up PlatformIO or Arduino IDE project
- [ ] Install PCA9685 library (Adafruit PWM Servo Driver)
- [ ] Test I2C communication with PCA9685
- [ ] Test single servo movement
- [ ] Calibrate servo pulse widths

##### Wiring Diagram
```
ESP32 DevKit C          PCA9685              Servos
┌─────────────┐      ┌──────────┐       ┌─────────────┐
│         3V3 │──────│ VCC      │       │             │
│         GND │──────│ GND      │───────│ GND (brown) │
│      GPIO21 │──────│ SDA      │       │             │
│      GPIO22 │──────│ SCL      │       │             │
│             │      │          │       │             │
│             │      │ V+   ────┼───────│ VCC (red)   │
│             │      │ (6V ext) │       │             │
│             │      │          │       │             │
│             │      │ PWM 0 ───┼───────│ SIG (orange)│ L_SHOULDER_PAN
│             │      │ PWM 1 ───┼───────│ ...         │ L_SHOULDER_TILT
│             │      │ PWM 2 ───┼───────│             │ L_ELBOW
│             │      │ PWM 3 ───┼───────│             │ R_SHOULDER_PAN
│             │      │ PWM 4 ───┼───────│             │ R_SHOULDER_TILT
│             │      │ PWM 5 ───┼───────│             │ R_ELBOW
└─────────────┘      └──────────┘       └─────────────┘
                           │
                     ┌─────┴─────┐
                     │ 6V 5A PSU │
                     │ (Servos)  │
                     └───────────┘
```

##### Acceptance Criteria
- I2C scan detects PCA9685 at address 0x40
- Single servo moves to commanded position
- Pulse width calibration documented

#### 3.2 Serial Command Handler
**Duration:** 3-4 days

##### Code: `real_steel_controller.ino`
```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== CONFIGURATION ====================
#define SERIAL_BAUD 115200
#define NUM_SERVOS 6
#define I2C_SDA 21
#define I2C_SCL 22

// Servo pulse width limits (microseconds)
// Adjust these per servo during calibration!
const int SERVO_MIN_US[NUM_SERVOS] = {500, 500, 500, 500, 500, 500};
const int SERVO_MAX_US[NUM_SERVOS] = {2500, 2500, 2500, 2500, 2500, 2500};

// Angle limits (degrees)
const float ANGLE_MIN[NUM_SERVOS] = {-90, -45, 0, -90, -45, 0};
const float ANGLE_MAX[NUM_SERVOS] = {90, 135, 135, 90, 135, 135};

// Home positions (degrees)
const float HOME_ANGLES[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};

// ==================== GLOBALS ====================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
float currentAngles[NUM_SERVOS] = {0};
bool servosEnabled = true;
String inputBuffer = "";

// ==================== SETUP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard servo frequency
  
  delay(100);
  
  // Move to home position
  homePosition();
  
  Serial.println("READY");
}

// ==================== MAIN LOOP ====================
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

// ==================== COMMAND HANDLER ====================
void handleCommand(String cmd) {
  char cmdType = cmd.charAt(0);
  
  switch (cmdType) {
    case 'J':  // Joint positions: J:a0,a1,a2,a3,a4,a5
      handleJointCommand(cmd.substring(2));
      break;
      
    case 'S':  // Single joint: S:idx:angle
      handleSingleJoint(cmd.substring(2));
      break;
      
    case 'H':  // Home
      homePosition();
      Serial.println("OK");
      break;
      
    case 'Q':  // Query positions
      queryPositions();
      break;
      
    case 'E':  // Enable/disable: E:0 or E:1
      servosEnabled = (cmd.charAt(2) == '1');
      Serial.println("OK");
      break;
      
    default:
      Serial.println("ERR:1:Unknown command");
  }
}

void handleJointCommand(String data) {
  float angles[NUM_SERVOS];
  int idx = 0;
  int lastComma = -1;
  
  for (int i = 0; i <= data.length() && idx < NUM_SERVOS; i++) {
    if (i == data.length() || data.charAt(i) == ',') {
      String part = data.substring(lastComma + 1, i);
      angles[idx] = part.toFloat();
      idx++;
      lastComma = i;
    }
  }
  
  if (idx != NUM_SERVOS) {
    Serial.println("ERR:1:Expected 6 angles");
    return;
  }
  
  // Validate and set
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (angles[i] < ANGLE_MIN[i] || angles[i] > ANGLE_MAX[i]) {
      Serial.print("ERR:2:Angle out of range for joint ");
      Serial.println(i);
      return;
    }
  }
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, angles[i]);
  }
  
  Serial.println("OK");
}

void handleSingleJoint(String data) {
  int colonIdx = data.indexOf(':');
  if (colonIdx < 0) {
    Serial.println("ERR:1:Invalid format");
    return;
  }
  
  int jointIdx = data.substring(0, colonIdx).toInt();
  float angle = data.substring(colonIdx + 1).toFloat();
  
  if (jointIdx < 0 || jointIdx >= NUM_SERVOS) {
    Serial.println("ERR:3:Invalid joint index");
    return;
  }
  
  if (angle < ANGLE_MIN[jointIdx] || angle > ANGLE_MAX[jointIdx]) {
    Serial.println("ERR:2:Angle out of range");
    return;
  }
  
  setServoAngle(jointIdx, angle);
  Serial.println("OK");
}

void queryPositions() {
  Serial.print("P:");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(currentAngles[i], 1);
    if (i < NUM_SERVOS - 1) Serial.print(",");
  }
  Serial.println();
}

void homePosition() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, HOME_ANGLES[i]);
  }
}

// ==================== SERVO CONTROL ====================
void setServoAngle(int servo, float angle) {
  if (!servosEnabled) return;
  
  currentAngles[servo] = angle;
  
  // Map angle to pulse width
  int pulseUs = map(
    angle * 10,  // Use 10x for better precision
    ANGLE_MIN[servo] * 10, 
    ANGLE_MAX[servo] * 10,
    SERVO_MIN_US[servo], 
    SERVO_MAX_US[servo]
  );
  
  // Convert microseconds to PCA9685 tick value
  // PCA9685 has 4096 ticks per 20ms cycle
  int tick = (pulseUs * 4096L) / 20000;
  
  pwm.setPWM(servo, 0, tick);
}
```

##### Acceptance Criteria
- All command types parsed correctly
- Invalid commands return appropriate error
- No buffer overflow on long messages

#### 3.3 Integration Test
**Duration:** 2 days

##### Test Script: `tests/test_esp32.py`
```python
import serial
import time

def main():
    # Update with your actual port
    port = '/dev/tty.usbserial-0001'  # Mac example
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(2)  # Wait for ESP32 reset
    
    # Read ready message
    print("Waiting for READY...")
    ready = ser.readline().decode().strip()
    print(f"Received: {ready}")
    
    # Test home
    print("\nTesting HOME command...")
    ser.write(b'H\n')
    print(f"Response: {ser.readline().decode().strip()}")
    
    # Test query
    print("\nTesting QUERY command...")
    ser.write(b'Q\n')
    print(f"Response: {ser.readline().decode().strip()}")
    
    # Test joint command
    print("\nTesting JOINT command...")
    ser.write(b'J:45,-30,90,45,-30,90\n')
    print(f"Response: {ser.readline().decode().strip()}")
    time.sleep(1)
    
    # Query again
    ser.write(b'Q\n')
    print(f"Current positions: {ser.readline().decode().strip()}")
    
    # Test error handling
    print("\nTesting error handling...")
    ser.write(b'J:1000,0,0,0,0,0\n')  # Out of range
    print(f"Response: {ser.readline().decode().strip()}")
    
    # Home and close
    ser.write(b'H\n')
    ser.readline()
    ser.close()
    print("\nTest complete!")

if __name__ == '__main__':
    main()
```

##### Acceptance Criteria
- Round-trip latency < 10ms
- 50Hz command rate sustained without drops
- Stable operation for 10+ minutes

---

### Milestone 4: End-to-End Integration
**Duration:** 1-1.5 weeks  
**Goal:** Camera input controls robot (simulation and real)

#### 4.1 Real Robot Interface
**Duration:** 2-3 days

##### Code: `real_robot.py`
```python
import serial
import time
import numpy as np
from typing import Optional
from robot_interface import RobotInterface, JointState

class RealRobot(RobotInterface):
    """Serial communication with ESP32 hardware"""
    
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.serial: Optional[serial.Serial] = None
        self.timeout = 1.0
        
    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)  # Wait for ESP32 reset
            
            # Wait for READY
            response = self.serial.readline().decode().strip()
            if response != 'READY':
                print(f"Unexpected response: {response}")
                return False
            
            return True
            
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        if self.serial is not None:
            self.home()
            self.serial.close()
            self.serial = None
    
    def set_joint_positions(self, positions: np.ndarray):
        """Set target positions for all joints (radians input, sent as degrees)"""
        if len(positions) != self.NUM_JOINTS:
            raise ValueError(f"Expected {self.NUM_JOINTS} positions")
        
        # Convert radians to degrees
        positions_deg = np.rad2deg(positions)
        
        # Format command
        angles_str = ','.join(f'{a:.1f}' for a in positions_deg)
        cmd = f'J:{angles_str}\n'
        
        self.serial.write(cmd.encode())
        response = self.serial.readline().decode().strip()
        
        if response != 'OK':
            print(f"Warning: {response}")
    
    def get_joint_state(self) -> JointState:
        self.serial.write(b'Q\n')
        response = self.serial.readline().decode().strip()
        
        if response.startswith('P:'):
            angles_deg = [float(x) for x in response[2:].split(',')]
            angles_rad = np.deg2rad(angles_deg)
            
            return JointState(
                positions=np.array(angles_rad),
                velocities=np.zeros(self.NUM_JOINTS),
                timestamp=time.time()
            )
        else:
            raise RuntimeError(f"Invalid response: {response}")
    
    def home(self):
        self.serial.write(b'H\n')
        self.serial.readline()  # Consume OK
    
    def is_connected(self) -> bool:
        return self.serial is not None and self.serial.is_open
```

#### 4.2 Main Control Loop
**Duration:** 3-4 days

##### Code: `main.py`
```python
#!/usr/bin/env python3
import argparse
import cv2
import time
import yaml
import numpy as np
from pathlib import Path

from camera import Camera
from pose_estimator import PoseEstimator
from angle_calculator import AngleCalculator
from motion_mapper import MotionMapper, MappingConfig
from simulated_robot import SimulatedRobot
from real_robot import RealRobot

def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)

def main():
    parser = argparse.ArgumentParser(description='Real Steel Shadow Boxing Robot')
    parser.add_argument('--sim', action='store_true', help='Use PyBullet simulation')
    parser.add_argument('--config', default='config/settings.yaml', help='Config file path')
    parser.add_argument('--port', default=None, help='Serial port (overrides config)')
    parser.add_argument('--no-viz', action='store_true', help='Disable visualization')
    args = parser.parse_args()
    
    # Load config
    config = load_config(args.config)
    
    # Initialize components
    print("Initializing camera...")
    camera = Camera(
        device_id=config['camera']['device_id'],
        width=config['camera']['width'],
        height=config['camera']['height'],
        fps=config['camera']['fps']
    )
    
    if not camera.open():
        print("Failed to open camera!")
        return
    
    print("Initializing pose estimator...")
    pose_estimator = PoseEstimator(
        min_detection_confidence=config['pose']['min_detection_confidence'],
        min_tracking_confidence=config['pose']['min_tracking_confidence']
    )
    
    print("Initializing angle calculator...")
    angle_calculator = AngleCalculator(
        smoothing_factor=config['angles']['smoothing_factor']
    )
    
    print("Initializing motion mapper...")
    mapping_config = MappingConfig(
        mirror_mode=config['mapping']['mirror_mode'],
        dead_zone=np.deg2rad(config['mapping']['dead_zone_deg'])
    )
    motion_mapper = MotionMapper(mapping_config)
    
    # Initialize robot
    if args.sim:
        print("Initializing PyBullet simulation...")
        robot = SimulatedRobot(
            urdf_path=config['simulation']['urdf_path'],
            gui=not args.no_viz
        )
    else:
        port = args.port or config['serial']['port']
        print(f"Connecting to ESP32 on {port}...")
        robot = RealRobot(port=port, baud=config['serial']['baud'])
    
    if not robot.connect():
        print("Failed to connect to robot!")
        camera.release()
        return
    
    print("Starting main loop. Press 'q' to quit.")
    
    # Performance tracking
    frame_times = []
    last_print = time.time()
    
    try:
        while True:
            loop_start = time.time()
            
            # Capture frame
            frame = camera.read()
            if frame is None:
                continue
            
            # Estimate pose
            pose = pose_estimator.process(frame.image, frame.timestamp)
            
            # Calculate angles and move robot
            if pose.is_valid:
                angles = angle_calculator.calculate(pose)
                if angles is not None:
                    servo_angles = motion_mapper.map(angles)
                    robot.set_joint_positions(servo_angles.angles)
            
            # Step simulation (if applicable)
            if args.sim:
                robot.step()
            
            # Visualization
            if not args.no_viz:
                viz_frame = pose_estimator.draw(frame.image, pose)
                
                # Add FPS display
                fps = len(frame_times) / (sum(frame_times) + 1e-6)
                cv2.putText(viz_frame, f'FPS: {fps:.1f}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow('Real Steel', viz_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            # Performance tracking
            frame_time = time.time() - loop_start
            frame_times.append(frame_time)
            if len(frame_times) > 30:
                frame_times.pop(0)
            
            # Print stats periodically
            if time.time() - last_print > 5.0:
                avg_time = sum(frame_times) / len(frame_times)
                print(f"Avg frame time: {avg_time*1000:.1f}ms ({1/avg_time:.1f} FPS)")
                last_print = time.time()
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        print("Cleaning up...")
        robot.home()
        robot.disconnect()
        camera.release()
        cv2.destroyAllWindows()
        print("Done!")

if __name__ == '__main__':
    main()
```

##### Configuration: `config/settings.yaml`
```yaml
camera:
  device_id: 0
  width: 640
  height: 480
  fps: 30

pose:
  min_detection_confidence: 0.5
  min_tracking_confidence: 0.5

angles:
  smoothing_factor: 0.3

mapping:
  mirror_mode: true
  dead_zone_deg: 3.0

serial:
  port: /dev/tty.usbserial-0001
  baud: 115200

simulation:
  urdf_path: ../urdf/real_steel.urdf
```

#### 4.3 Testing & Tuning
**Duration:** 2-3 days

##### Test Scenarios
| Test | Command | Expected Result |
|------|---------|-----------------|
| Simulation only | `python main.py --sim` | PyBullet window + camera, robot mirrors |
| Real hardware | `python main.py --port /dev/tty.usbserial-XXX` | Physical robot mirrors |
| No visualization | `python main.py --sim --no-viz` | Console output only |

##### Acceptance Criteria
- End-to-end latency < 500ms
- All basic punches recognizable
- 10-minute continuous operation stable
- Smooth transition between sim and real

---

### Milestone 5: Demo & Documentation
**Duration:** 1 week  
**Goal:** Polished demo and complete documentation

#### 5.1 Demo Modes
**Duration:** 2-3 days

##### Tasks
- [ ] Real-time mirroring mode (default)
- [ ] Recording mode (save motion to file)
- [ ] Playback mode (replay recorded motion)
- [ ] Mode switching via keyboard

#### 5.2 Documentation
**Duration:** 2-3 days

##### Tasks
- [ ] Complete README with setup instructions
- [ ] Hardware assembly guide with photos
- [ ] Wiring diagram
- [ ] Troubleshooting guide

#### 5.3 Demo Video & Portfolio
**Duration:** 2 days

##### Tasks
- [ ] Record demo video
- [ ] Create GitHub repo with proper README
- [ ] Performance metrics summary

---

## 8. ROS2 Migration Plan (Phase B)

### 8.1 Why Migrate to ROS2?

| Benefit | Description |
|---------|-------------|
| **Modularity** | Each component as separate node |
| **Standard interfaces** | Use standard message types |
| **Tooling** | RViz visualization, rosbag recording |
| **Simulation** | Gazebo integration (uses same URDF!) |
| **Community** | Leverage existing packages |
| **Portfolio** | ROS2 experience valued in industry |

### 8.2 Prerequisites

- [ ] MVP working and stable
- [ ] Linux machine or Docker setup
- [ ] Basic ROS2 tutorials completed

### 8.3 Migration Steps

#### Step B.1: Environment Setup (1 week)
- [ ] Ubuntu 22.04 or Docker
- [ ] ROS2 Humble installation
- [ ] Verify camera in Linux

#### Step B.2: Create ROS2 Packages (2-3 weeks)

##### Package Structure
```
ros2_ws/src/real_steel/
├── real_steel_msgs/           # Custom messages
├── real_steel_perception/     # Camera + Pose nodes
├── real_steel_control/        # Angle + Mapping nodes
├── real_steel_hardware/       # Serial bridge node
├── real_steel_description/    # URDF (copy from MVP!)
└── real_steel_bringup/        # Launch files
```

##### Key Point: URDF Reuse
The URDF created for PyBullet works directly in Gazebo and RViz!

#### Step B.3: Port Code to ROS2 Nodes (1-2 weeks)

| MVP Module | ROS2 Node | Topic |
|------------|-----------|-------|
| camera.py | camera_node | /camera/image |
| pose_estimator.py | pose_node | /skeleton |
| angle_calculator.py | angle_node | /human_joints |
| motion_mapper.py | mapping_node | /servo_commands |
| real_robot.py | serial_bridge_node | /joint_states |

#### Step B.4: Gazebo Simulation (1-2 weeks)
- Same URDF, just add Gazebo plugins
- ros2_control for joint control
- Compare Gazebo vs PyBullet behavior

### 8.4 Migration Timeline

| Phase | Duration |
|-------|----------|
| B.1 Environment | 1 week |
| B.2 Packages | 2-3 weeks |
| B.3 Port Code | 1-2 weeks |
| B.4 Gazebo | 1-2 weeks |
| **Total** | **5-8 weeks** |

---

## 9. Bill of Materials

### 9.1 MVP Hardware

| Component | Qty | Price | Notes |
|-----------|-----|-------|-------|
| ESP32 DevKit C | 1 | $0 | Already owned |
| PCA9685 PWM Driver | 1 | $8 | 16-channel I2C |
| MG996R Servo | 6 | $30 | ~$5 each |
| 6V 5A Power Supply | 1 | $15 | For servos |
| Jumper wires | 1 | $5 | |
| Breadboard | 1 | $5 | |
| USB cable | 1 | $5 | |
| Hardboard/cardboard | - | $5 | Simple frame or mount servos directly |
| **Total** | | **~$73** | |

### 9.2 Optional Upgrades (Post-MVP)

| Component | Price | When to Consider |
|-----------|-------|------------------|
| 3D printed frame | $20-50 | After servo positions validated |
| Aluminum extrusion frame | $30-50 | For rigidity |
| External USB camera | $30-50 | Dedicated setup |
| Raspberry Pi 4 + camera | $80-100 | Standalone system |
| Dynamixel AX-12A (×6) | $150 | If MG996R torque insufficient |

---

## 10. Timeline Summary

### Phase A: MVP

```
Week:  1    2    3    4    5    6    7    8
M0:    ███
M1:    ░░░░░████████
M2:         ████████████
M3:              ████████████
M4:                        ████████████
M5:                                  █████
```

| Milestone | Duration |
|-----------|----------|
| M0: Environment Setup | 3-5 days |
| M1: Robot Model & Simulation | 1-1.5 weeks |
| M2: Perception Pipeline | 1.5-2 weeks |
| M3: ESP32 Controller | 1.5-2 weeks |
| M4: Integration | 1-1.5 weeks |
| M5: Demo & Docs | 1 week |
| **Total MVP** | **~7-8 weeks** |

### Phase B: ROS2 Migration
| Duration | ~5-8 weeks |

---

## 11. Decisions Log

| Question | Decision | Notes |
|----------|----------|-------|
| Simulation | PyBullet for Phase A | No ROS required, Mac native |
| ROS2 | Phase B (later) | After MVP works |
| Frame design | **Hardboard or no frame** | Simplest first, iterate later |
| Servo strength | **MG996R** | Sufficient for first iteration |
| Camera | **Mac built-in** | Upgrade to RPi/external later |

### Future Hardware Upgrades (Post-MVP)
- Frame: Design proper 3D printed or aluminum frame
- Camera: Raspberry Pi camera or USB webcam for dedicated setup
- Servos: Upgrade to Dynamixel if torque insufficient

---

## 12. Safety Considerations

### 12.1 Emergency Stop

| Scenario | Behavior |
|----------|----------|
| Press 'ESC' key | Send `E:0` to disable servos, robot holds position |
| Serial disconnect | ESP32 holds last position for 2 seconds, then moves to home |
| Pose estimation fails | Freeze last valid servo positions |
| Angle out of range | Clamp to limits, log warning |

### 12.2 Hardware Safety

| Risk | Mitigation |
|------|------------|
| Servo overheating | Limit continuous operation to 10 min, add 5 min cooldown |
| Power supply overload | Use 5A supply for 6 servos (each draws ~0.5-1A under load) |
| Servo stall | ESP32 monitors for stall (no position change), disables after 3 seconds |
| Wiring short | Use breadboard carefully, verify connections before power-on |

### 12.3 Software Safety

```python
# Safety limits in config
safety:
  max_velocity: 2.0        # rad/s - limit joint speed
  position_tolerance: 0.1  # rad - acceptable error
  watchdog_timeout: 2.0    # seconds - disable if no command received
  max_continuous_runtime: 600  # seconds (10 min)
```

### 12.4 Safe Startup Sequence

1. Power on ESP32 (no servo power yet)
2. Verify serial connection (`READY` received)
3. Send `E:0` to ensure servos disabled
4. Power on servo power supply
5. Send `H` (home) command
6. Send `E:1` to enable servos
7. Begin normal operation

---

## 13. Calibration Procedures

### 13.1 Servo Zero Position Calibration

Each servo has slight variation in pulse width response. Calibrate once during initial setup.

**Procedure:**
1. Disconnect servo horn from arm linkage
2. Send servo to "0 degrees" position: `S:0:0`
3. Physically observe servo horn position
4. If not at expected position, adjust `SERVO_MIN_US` / `SERVO_MAX_US` in ESP32 code
5. Repeat for each servo
6. Document final calibration values

**Calibration Table Template:**

| Servo # | Joint | Min μs | Max μs | Zero Offset (°) |
|---------|-------|--------|--------|-----------------|
| 0 | L_SHOULDER_PAN | 500 | 2500 | 0 |
| 1 | L_SHOULDER_TILT | 500 | 2500 | 0 |
| 2 | L_ELBOW | 500 | 2500 | 0 |
| 3 | R_SHOULDER_PAN | 500 | 2500 | 0 |
| 4 | R_SHOULDER_TILT | 500 | 2500 | 0 |
| 5 | R_ELBOW | 500 | 2500 | 0 |

### 13.2 Camera Calibration (Optional)

For improved 3D pose accuracy:

1. Print checkerboard pattern (8×6 squares, 25mm each)
2. Capture 15-20 images from different angles
3. Run OpenCV calibration:

```python
# calibrate_camera.py
import cv2
import numpy as np
import glob

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*8, 3), np.float32)
objp[:,:2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * 0.025  # 25mm squares

objpoints, imgpoints = [], []
images = glob.glob('calibration/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
np.savez('calibration/camera_params.npz', mtx=mtx, dist=dist)
print(f"Reprojection error: {ret:.4f} pixels")
```

### 13.3 Motion Mapping Calibration

Fine-tune the human-to-robot mapping:

1. Stand in T-pose (arms extended horizontally)
2. Verify robot mirrors T-pose
3. If not aligned, adjust `scale_factors` in config
4. Test punch motion, adjust `dead_zone` if jittery
5. Test full range of motion, verify no joint limit violations

---

## 14. Testing Checklist

### 14.1 Pre-Integration Tests

#### Hardware Tests
- [ ] ESP32 boots and sends `READY` over serial
- [ ] I2C scan detects PCA9685 at address 0x40
- [ ] Each servo responds to individual commands (`S:0:45`, `S:1:45`, etc.)
- [ ] All servos move to home position (`H` command)
- [ ] Servo power supply delivers 6V under load
- [ ] No visible overheating after 5 minutes continuous movement

#### Software Tests (Mac)
- [ ] Camera captures frames at ≥25 FPS
- [ ] MediaPipe detects pose when person in frame
- [ ] Pose estimation runs at ≥15 FPS
- [ ] Joint angles calculated correctly (visual verification)
- [ ] PyBullet simulation loads URDF without errors
- [ ] Simulated robot responds to joint commands

#### Communication Tests
- [ ] Serial connection established
- [ ] Round-trip latency < 10ms
- [ ] 50Hz command rate sustained for 1 minute
- [ ] Error responses received for invalid commands
- [ ] Graceful handling of disconnect/reconnect

### 14.2 Integration Tests

| Test | Procedure | Expected Result |
|------|-----------|-----------------|
| Simulation mirror | Run with `--sim`, move arms | Robot mirrors in PyBullet |
| Hardware mirror | Run with real ESP32, move arms | Physical robot mirrors |
| Latency check | Punch quickly, observe robot | Visible delay < 0.5s |
| Endurance | Run for 10 minutes continuously | No crashes, no overheating |
| Recovery | Disconnect USB, reconnect | System recovers gracefully |

### 14.3 Demo Readiness Checklist

- [ ] All integration tests pass
- [ ] Demo video recorded
- [ ] README completed
- [ ] Code committed and pushed
- [ ] Hardware assembled and stable
- [ ] Backup servo available

---

## 15. Glossary

| Term | Definition |
|------|------------|
| **DOF** | Degrees of Freedom — number of independent movements a robot can make |
| **URDF** | Unified Robot Description Format — XML format for describing robot structure |
| **PWM** | Pulse Width Modulation — technique for controlling servo position via pulse timing |
| **PCA9685** | 16-channel PWM driver IC, controlled via I2C |
| **I2C** | Inter-Integrated Circuit — two-wire communication protocol |
| **MediaPipe** | Google's ML framework for pose estimation |
| **PyBullet** | Physics simulation library for robotics |
| **ROS2** | Robot Operating System 2 — middleware for robotics software |
| **EMA** | Exponential Moving Average — smoothing technique for noisy data |
| **Keypoint** | Detected body landmark (e.g., shoulder, elbow, wrist) |
| **Radians** | Angular unit where 2π = 360° (π/2 = 90°) |
| **Torque** | Rotational force, measured in kg·cm for servos |
| **Baud Rate** | Serial communication speed in bits per second |
| **URDF Joint** | Connection between two links allowing relative motion |
| **URDF Link** | Rigid body segment in robot model |

---

## 16. References

- PyBullet: https://pybullet.org/
- MediaPipe Pose: https://google.github.io/mediapipe/solutions/pose
- ESP32 Arduino: https://docs.espressif.com/projects/arduino-esp32/
- PCA9685: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
- URDF Tutorial: https://wiki.ros.org/urdf/Tutorials
- ROS2 Humble: https://docs.ros.org/en/humble/

---

## 17. Changelog

| Version | Date | Changes |
|---------|------|---------|
| 0.1 | 2025-01-30 | Initial draft |
| 0.2 | 2025-01-30 | ESP32, dual arm, Mac dev, ROS2 migration plan |
| 0.3 | 2025-01-30 | Added PyBullet simulation, URDF model, unified robot interface |
| 1.0 | 2025-01-30 | Final MVP spec: hardboard frame, MG996R servos, Mac camera |
| **1.1** | **2025-01-30** | **Added: Safety section, Calibration procedures, Testing checklist, Glossary. Fixed: dimension inconsistency, unit conventions, code cleanup** |