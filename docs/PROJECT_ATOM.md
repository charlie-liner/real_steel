# Project Atom — Building a Sparring Boxing Robot

> Inspired by the movie *Real Steel*, this project aims to build a humanoid boxing robot that can serve as a real sparring partner for fighters — and eventually bring robot sports to life.

---

## Vision

### Ultimate Goal: A Real Sparring Partner

The core mission is to build a boxing robot that can genuinely help fighters train.

**Near-term (this project):**
Build an upper-body robot that can mirror a human's boxing movements in real-time (shadow mode), then progressively learn to react, defend, and spar autonomously.

**Mid-term:**
A robot capable of serving as a legitimate sparring partner for amateur and professional boxers — reading opponent movements, adapting fight style, and adjusting difficulty.

**Long-term (beyond this project):**
- **Sports Humanoids** — Expand beyond boxing to other sports. Robots that act as training partners, coaches, or opponents across disciplines (fencing, table tennis, martial arts, etc.)
- **Robot Sports** — Real Steel-style robot fighting competitions. Robot vs robot, controlled or autonomous, as a new category of sport and entertainment.

### Practical Goal: Grad School Portfolio

This project serves as:
- **Proof of engineering ability** — design, build, and program a physical robot from scratch
- **Proof of systematic thinking** — leveraging DE background to build proper data pipelines for robot learning
- **A research foundation** — sim2real pipeline, motion prediction, and intent classification are publishable research topics

**Target:** Korean robotics labs (KAIST CLVR, HuboLab, SNU DYROS/RLLAB, etc.)

---

## Project Phases

### Phase 1: Shadow Mode MVP — IN PROGRESS

**Goal:** 8-DOF dual-arm upper-body robot that mirrors boxing movements in real-time.

**Status:**
- [x] MediaPipe pose detection → servo control pipeline
- [x] Single servo test — left elbow tracking via PCA9685 (working!)
- [x] Basic sim2real setup with PyBullet
- [ ] Remaining 7 servo motors (waiting for hardware)
- [ ] Full 8-DOF dual-arm assembly
- [ ] Physical robot frame construction

**Hardware:**
- ESP32 + PCA9685 PWM driver
- 8x servo motors (MG996R or similar)
- 5V 6A external power supply

---

### Phase 2: Robot Data Pipeline

**Goal:** Production-grade data infrastructure for robot motion data.

#### 2.1 Real-time Motion Data Collection
- Stream pose data (landmarks, joint angles, timestamps)
- Servo telemetry (commanded vs actual positions, latency)
- Real-time ingestion pipeline

#### 2.2 Sim2Real Gap Quantification
- Measure discrepancies between simulated and real robot behavior
- Time-series dashboards for sim2real drift
- Data quality monitoring

#### 2.3 Experiment & Training Data Management
- Dataset versioning
- Data lineage tracking
- Reproducible experiment pipeline

---

### Phase 3: Shadow Mode Refinement

**Goal:** Improve mirroring accuracy and reduce latency.

**Key Tasks:**
- Calibration system for servo-to-angle mapping
- Latency compensation via motion prediction (LSTM/Transformer)
- End-to-end latency optimization (camera → prediction → servo)

---

### Phase 4: Reactive Mode — Understanding the Opponent

**Goal:** Robot stops copying and starts responding.

**Key Tasks:**
- Action classifier on pose time-series
  - Classes: jab, cross, hook, uppercut, guard, slip, duck, idle
- Rule-based response mapping (jab → guard, hook → duck)
- Boxing movement pattern library

---

### Phase 5: Learning-Based Sparring

**Goal:** Robot learns optimal responses through RL.

**Key Tasks:**
- RL agent in MuJoCo simulation
- Reward design (dodge, block, hit, balance)
- Sim2real transfer
- Self-play training
- Difficulty adjustment

---

### Phase 6: Autonomous Sparring Partner

**Goal:** A robot that genuinely helps someone train boxing.

**Key Tasks:**
- Adjustable difficulty levels
- Adaptation to fighting styles
- Multi-round memory
- Safety mechanisms

---

## Tech Stack

| Layer | Tools |
|-------|-------|
| Perception | MediaPipe, OpenCV |
| Simulation | PyBullet → MuJoCo |
| Control | Python, ESP32, PCA9685 |
| Data Pipeline | TBD (Kafka/Pub-Sub, DuckDB/BigQuery) |
| ML/RL | PyTorch, Stable-Baselines3 |

---

## Current Hardware Setup

```
Mac (Python)
    │
    │ USB Serial (115200 baud)
    ▼
ESP32
    │
    │ I2C (GPIO 21/22)
    ▼
PCA9685 PWM Driver
    │
    │ PWM Channels 0-7
    ▼
Servo Motors (8x)
    - Channel 0: Left Elbow (working)
    - Channel 1-7: TBD
```

---

*Project started: 2026*
*Maintainer: Charlie*
