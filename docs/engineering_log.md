# Engineering Log

This document tracks key technical decisions, experiments, and lessons learned during development. Intended to demonstrate systematic engineering thinking for graduate school applications.

---

## 2025-02-07: Motion Tracking & Servo Integration

### Session Goals
- Test single servo motor control via MediaPipe pose tracking
- Investigate shaking/jitter in simulation
- Understand limitations of current pose estimation

### Experiments Conducted

#### 1. Single Servo Elbow Tracking

**Setup:** ESP32 + PCA9685 PWM driver + MG996R servo on channel 0

**Initial Problem:** Servo didn't respond to serial commands.

**Root Cause:** Firmware was using direct GPIO PWM (`ledcWrite`) but hardware uses PCA9685 I2C PWM driver.

**Solution:** Rewrote `single_servo_test.ino` to use `Adafruit_PWMServoDriver` library:
```cpp
// Before (wrong)
ledcWrite(SERVO_PIN, duty);

// After (correct)
pca.setPWM(SERVO_CH, 0, pulse);
```

**Result:** Servo now tracks left elbow angle in real-time. Latency ~50ms.

#### 2. Reducing Simulation Jitter

**Problem:** Robot in PyBullet simulation shakes continuously even when human stands still.

**Analysis:** Simple EMA smoothing (`α=0.3`) was insufficient for MediaPipe noise.

**Solution:** Implemented **One Euro Filter** — adaptive low-pass filter used in VR/AR applications.

```python
# Key parameters
min_cutoff = 0.3  # Lower = more smoothing when still
beta = 0.01       # Higher = faster response when moving
```

**Result:** 76% reduction in joint angle variance.

**Reference:** Casiez et al., "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input" (CHI 2012)

#### 3. Forward Punch Detection (Z-Axis Problem)

**Problem:** Jab and cross (forward punches) are poorly tracked. Robot arm doesn't extend when human punches toward camera.

**Hypothesis:** MediaPipe's Z (depth) estimation is unreliable because it's inferred from 2D image, not measured.

**Data Collection:** Captured landmark positions for neutral pose vs. left jab vs. right cross.

**Key Finding from Data:**

| Pose | L_wrist.y | L_shoulder.y | Difference |
|------|-----------|--------------|------------|
| Neutral | 0.77 | 0.22 | Wrist 0.55 below shoulder |
| Left Jab | 0.14 | 0.30 | Wrist 0.16 **above** shoulder |

**Insight:** Forward punches cause wrist to rise above shoulder in Y-axis. This is a reliable 2D signal that doesn't depend on noisy Z.

**Solution:** Created `PunchDetector` class:
```python
# Detect punch by wrist rise above shoulder
wrist_rise = shoulder.y - wrist.y
forward_amount = wrist_rise / upper_arm_length
```

When punch detected, boost shoulder tilt angle to compensate for undetectable Z motion.

### Technical Decisions

| Decision | Rationale |
|----------|-----------|
| Keep MediaPipe for now | Free, works on Mac, sufficient for MVP |
| Use One Euro Filter | Industry standard for motion tracking, adaptive |
| Detect punch via Y-axis | Avoids noisy Z, simpler than ML classifier |
| 8-DOF robot model | Roll joint needed for realistic arm abduction |

### Limitations Identified

1. **MediaPipe Z accuracy:** ~3x noisier than X/Y. Cannot reliably track depth.
2. **Single camera:** No stereo depth information.
3. **30 FPS limit:** Boxing punches are 100-200ms; higher frame rate would help.

### Future Options Considered

| Approach | Pros | Cons | Cost |
|----------|------|------|------|
| Wearable IMUs on gloves | True acceleration, no occlusion, 100+ Hz | Requires hardware integration | ~$60 |
| OAK-D Lite stereo camera | Real depth, on-device NN | Additional hardware | ~$150 |
| Two-camera triangulation | True 3D with standard webcams | Complex calibration | ~$50 |
| iPhone ARKit | LiDAR depth, high quality | iOS development needed | Free (if owned) |

**Recommendation:** Sensor fusion (camera + IMUs) would be most robust and demonstrates multi-modal integration skills.

### Files Modified

- `src/angle_calculator.py` — Added One Euro Filter, switched elbow to 2D calculation
- `src/punch_detector.py` — New module for forward punch detection
- `src/pose_estimator.py` — Added hip and nose keypoints (9 total, was 6)
- `src/main.py` — Integrated punch detection into control loop
- `esp32/test_sketches/single_servo_test/` — Fixed for PCA9685 I2C
- `tests/test_simulation.py` — Corrected 6-DOF to 8-DOF
- `CLAUDE.md` — Updated joint configuration documentation

### Metrics

| Metric | Before | After |
|--------|--------|-------|
| Joint angle jitter (std) | 0.094 rad | 0.022 rad |
| Elbow dead zone | 46° | 20° |
| Punch detection | None | Wrist-rise heuristic |

### Next Steps

1. Receive remaining 7 servo motors
2. Assemble full 8-DOF dual-arm system
3. Collect labeled motion dataset (jab, cross, hook, uppercut, guard)
4. Evaluate need for IMU or depth camera upgrade

---

## 2026-02-07: RTMW3D Pose Estimator Integration

### Goals
- Replace MediaPipe with RTMW3D for better depth (Z-axis) accuracy
- Enable accurate tracking of forward punches (jab, cross) without heuristics

### Background

MediaPipe's Z-depth estimation is ~3x noisier than X/Y because it infers depth from a 2D monocular image. RTMW3D (Real-Time Multi-person Wholebody 3D) from MMPose has a **dedicated Z-axis prediction branch** trained specifically for depth estimation, providing much better accuracy for forward motion detection.

### Implementation

#### 1. Added RTMW3D Pose Estimator

Created `src/pose_estimator_rtmw.py` using `rtmlib` (lightweight wrapper, no mmcv/mmpose dependencies):

```python
from rtmlib import Wholebody3d

class PoseEstimatorRTMW:
    def __init__(self, device: str = "cpu", min_confidence: float = 0.3):
        self.model = Wholebody3d(device=device, mode="balanced")

    def process(self, image: np.ndarray, timestamp: float) -> PoseResult:
        result = self.model(image)
        keypoints_3d = result[0]  # (N, 133, 3) - includes depth
        # ... process into PoseResult with Point3D keypoints
```

Key differences from MediaPipe:
- **133 keypoints** (COCO-WholeBody): body + feet + face + hands
- **True 3D predictions** with trained Z-axis branch
- Runs on CPU, CUDA, or MPS (Mac)

#### 2. Adaptive Depth Weighting

Updated `AngleCalculator` with `use_full_depth` parameter:

```python
def __init__(self, smoothing_factor: float = 0.5, use_full_depth: bool = False):
    # RTMW3D has accurate Z → use full depth weight (1.0)
    # MediaPipe has noisy Z → down-weight depth (0.1)
    self.depth_weight = 1.0 if use_full_depth else 0.1
```

Elbow and shoulder tilt calculations now respect this setting:
- **MediaPipe mode**: 2D elbow calculation, 10% Z weight for tilt
- **RTMW3D mode**: Full 3D calculations

#### 3. CLI Integration

Added `--rtmw` flag to main.py:

```bash
# MediaPipe (default)
python src/main.py --sim

# RTMW3D with better depth
python src/main.py --sim --rtmw

# RTMW3D on GPU (if available)
python src/main.py --sim --rtmw --device cuda
```

### Technical Decisions

| Decision | Rationale |
|----------|-----------|
| Use rtmlib instead of full mmpose | Lightweight, no complex dependencies, works on Mac |
| Keep MediaPipe as default | Backward compatibility, RTMW3D is optional |
| Parameterize depth weight | Same angle calculator works for both estimators |
| Mode="balanced" for RTMW3D | Good accuracy/speed tradeoff |

### Files Modified

- `src/pose_estimator_rtmw.py` — New RTMW3D pose estimator
- `src/angle_calculator.py` — Added `use_full_depth` parameter
- `src/main.py` — Added `--rtmw` and `--device` CLI flags
- `tests/test_pipeline.py` — Updated tests for new parameter
- `requirements.txt` — Added rtmlib>=0.0.13

### Benchmark Results

| Model | Latency | FPS | Notes |
|-------|---------|-----|-------|
| MediaPipe | 8ms | **130+** | Fast, optimized for real-time |
| rtmlib Body lightweight | 33ms | 30 | 2D only, no depth |
| rtmlib Body balanced | 200ms | 5 | 2D only, no depth |
| **rtmlib Wholebody3d** | 350ms | **3** | Has Z-depth, too slow |

**Conclusion:** RTMW3D is ~40x slower than MediaPipe. For real-time shadow boxing, **MediaPipe + punch detection heuristic** is the correct approach. RTMW3D can be used for offline data collection or validation.

### Recommendation

Stick with MediaPipe + punch detection for real-time tracking:
- MediaPipe: Fast (130+ FPS), sufficient X/Y accuracy
- Punch detection: Compensates for noisy Z using wrist-rise heuristic
- RTMW3D: Use only for offline analysis or ground-truth data collection

### Next Steps

1. Keep MediaPipe as default for real-time
2. Use RTMW3D (`--rtmw`) for offline data collection only
3. Consider future options if faster 3D models become available:
   - DepthAnything + MediaPipe (depth map + 2D pose)
   - LiDAR on iPhone/iPad for true depth

---

## 2026-02-08: Move Classifier Approach

### Insight

For a boxing mirror robot, we don't need full 3D pose tracking. We need to:
1. **Classify** a small set of moves (jab, cross, hook, uppercut, guard)
2. **Play** pre-recorded motions that look like those moves

With a fixed front-facing camera, moves are distinguishable in 2D:
- **Jab**: Wrist extends toward center, arm straightens
- **Cross**: Same but opposite arm, with torso rotation
- **Hook**: Wrist arcs horizontally at shoulder height
- **Uppercut**: Wrist moves upward quickly
- **Guard**: Both wrists near face

### Architecture Change

**Before (over-engineered):**
```
Camera → 3D Pose → Exact angles → Mirror joints
         ↑ fighting Z noise
```

**After (simpler):**
```
Camera → 2D Pose → Classify move → Play pre-recorded motion
         ↑ fast, reliable
```

### Implementation

Created two new modules:

**`src/move_classifier.py`**
- Takes 2D keypoint trajectories (wrist position, velocity)
- Classifies into: NEUTRAL, GUARD, JAB, CROSS, HOOK_LEFT, HOOK_RIGHT, UPPERCUT_LEFT, UPPERCUT_RIGHT
- Rule-based detection using:
  - Wrist velocity (speed and direction)
  - Wrist position relative to shoulder/nose
  - Arm extension patterns

**`src/move_player.py`**
- Pre-defined motion keyframes for each move
- Interpolates between keyframes for smooth playback
- Auto-returns to guard stance after punch completion

### Usage

```bash
# New classifier mode (recommended)
python src/main.py --sim --classify

# Original tracking mode
python src/main.py --sim
```

### Benefits

| Aspect | Tracking Mode | Classifier Mode |
|--------|---------------|-----------------|
| Speed | 130+ FPS | 130+ FPS |
| Z-depth needed | Yes (noisy) | No |
| Motion quality | Matches human (with noise) | Clean pre-recorded |
| Latency | ~8ms | ~8ms + motion playback |

### Files Created

- `src/move_classifier.py` — 2D trajectory-based move classifier
- `src/move_player.py` — Pre-recorded motion playback

### Next Steps

1. Test classifier accuracy with real boxing movements
2. Tune velocity/position thresholds
3. Add combo detection (jab-cross, etc.)
4. Record smoother motion keyframes on real robot

---

## Template for Future Entries

```markdown
## YYYY-MM-DD: Title

### Goals
-

### Experiments
#### 1. Experiment Name
**Problem:**
**Hypothesis:**
**Method:**
**Result:**

### Decisions Made
| Decision | Rationale |

### Metrics
| Metric | Before | After |

### Next Steps
-
```
