# Project Atom — Building a Sparring Boxing Robot

> Inspired by the movie *Real Steel*, this project aims to build a boxing robot that progresses from mirroring human movements to autonomously sparring as a real training partner.

---

## Vision

### Ultimate Goal: A Real Sparring Partner

A boxing robot that genuinely helps fighters train — reads the opponent, adapts its style, and adjusts difficulty.

### What This Project Actually Delivers

Be honest about scope. This project is a stepping stone, not the final product.

| Horizon | What it is | Hardware |
|---------|-----------|----------|
| **This project** | Upper-body robot that recognizes boxing moves and responds | ESP32 + 7x MG996R hobby servos |
| **Next hardware** | Faster, stronger arms that can take light contact | Dynamixel or brushless actuators, rigid frame |
| **Long-term** | Full sparring partner with mobility | Research-grade platform (beyond this project) |

The current hardware (hobby servos, pan-tilt brackets) can demonstrate the intelligence but cannot physically spar. That's fine — the research value is in the perception and decision-making, not the actuators.

### Grad School Positioning

**Target:** Korean robotics labs (KAIST CLVR/HuboLab, SNU DYROS/RLLAB, etc.)

| What this project demonstrates | Why it matters |
|-------------------------------|---------------|
| Real-time action recognition from monocular video | Core perception research |
| Sim2real transfer for motion execution | Active research area |
| Full system integration (vision → decision → actuation) | Rare in applicants |
| Systematic engineering with data pipelines | DE background as differentiator |
| Physical hardware that works | Not just simulation papers |

**Research contributions** that could become publications:
1. Real-time boxing move classification from 2D pose sequences
2. Sim2real calibration methodology for low-cost servo systems
3. Rule-based → learned response selection for human-robot sparring

---

## Architecture Decision: Why Move Classification

The project explored three approaches for controlling the robot:

| Approach | How it works | Verdict |
|----------|-------------|---------|
| Exact angle mirroring | Copy every joint angle from MediaPipe | Z-depth is 3x noisier than X/Y. Forward punches invisible. |
| Punch compensator | Heuristic patches on mirroring | Band-aids. Still fundamentally fighting noise. |
| **Move classification + playback** | Classify the move, play a clean pre-recorded motion | **Winner.** Avoids Z-noise entirely. Cleaner output. Matches how real boxing works. |

A boxing robot doesn't need to replicate exact joint angles. It needs to recognize "that's a jab" and respond appropriately. This is also the architecture that scales to reactive/autonomous sparring.

```
┌────────────────────────────────────────────────────────────────┐
│                        CORE PIPELINE                           │
│                                                                │
│   Camera ──▶ 2D Pose ──▶ Move Classifier ──▶ Decision ──▶ Robot│
│              (MediaPipe)   (what move?)     (what to do?)       │
│                                                                │
│   Phase 1: Decision = mirror the same move                     │
│   Phase 2: Decision = respond with counter-move                │
│   Phase 3: Decision = learned policy (RL)                      │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

The pipeline stays the same across all phases. Only the **decision module** changes.

---

## Project Phases

### Phase 1: Shadow Mode MVP — IN PROGRESS

**Goal:** Assembled 7-DOF robot that recognizes and mirrors basic boxing moves.

**What "done" looks like:** You throw a jab → robot throws a jab. You throw a hook → robot throws a hook. You hold guard → robot holds guard. Clean, recognizable movements with <500ms latency.

#### 1A: Hardware Assembly & Validation
- [x] ESP32 + PCA9685 + 7 servos connected and responding
- [x] Motor sweep test passing (all 7 channels)
- [x] Serial protocol working (J/S/H/Q/E commands)
- [x] DTR reset for reliable connection
- [ ] Assemble frame with pan-tilt + U-brackets
- [ ] Joint mapping verification (correct servo → correct joint)
- [ ] Servo direction calibration (invert where needed)
- [ ] Pulse width calibration per servo
- [ ] Home position = boxing guard stance (bladed 45°)

#### 1B: Move Classification Pipeline
- [x] MediaPipe 2D pose extraction (working, 130+ FPS)
- [x] Move classifier prototype (`src/move_classifier.py`)
- [x] Move player prototype (`src/move_player.py`)
- [ ] Record clean motion keyframes on real hardware for each move:
  - Guard (default stance, 45° bladed)
  - Jab (lead hand straight)
  - Cross (rear hand straight)
  - Left hook
  - Right hook
  - Left uppercut
  - Right uppercut
- [ ] Tune classifier thresholds with real boxing movements
- [ ] Validate in sim: throw 10 of each move, measure classification accuracy
- [ ] Deploy to hardware: same test on real robot

#### 1C: Data Recording (lightweight, not a separate phase)
- [x] Frame-by-frame JSONL recorder (`src/evaluation/recorder.py`)
- [ ] Record classified moves with timestamps for later training data
- [ ] Log classifier confidence + actual move for accuracy tracking

**Key metric:** Classification accuracy ≥ 80% on the 7 move classes.

---

### Phase 2: Reactive Mode — The Robot Responds

**Goal:** Robot stops mirroring and starts counter-punching. This is where it becomes a training partner.

**What "done" looks like:** You throw a jab → robot slips and counters with a cross. You throw a hook → robot ducks. The responses are appropriate and recognizable.

#### 2A: Response Rule Engine
- [ ] Define response table:

| Human throws | Robot responds |
|-------------|---------------|
| Jab | Slip + counter cross |
| Cross | Parry + counter jab |
| Hook (L) | Duck / weave right |
| Hook (R) | Duck / weave left |
| Uppercut | Step back + guard |
| Guard | Feint or pressure |

- [ ] Implement response selector (rule-based first)
- [ ] Pre-record response motion keyframes on hardware
- [ ] Measure response latency (target: <300ms from move detection to first servo movement)

#### 2B: Combo Recognition
- [ ] Detect multi-move sequences (jab-cross, jab-jab-hook, etc.)
- [ ] Response to combos (not just individual punches)
- [ ] Sliding window over classified move history

#### 2C: Difficulty Levels
- [ ] Easy: slow responses, obvious tells, bigger defensive movements
- [ ] Medium: faster responses, smaller movements
- [ ] Hard: minimal telegraph, tight counters

**Key metric:** Appropriate response rate ≥ 70% (human judges if response makes boxing sense).

---

### Phase 3: Learned Responses — From Rules to Intelligence

**Goal:** Replace rule-based responses with a learned policy that adapts to the opponent.

**What "done" looks like:** The robot develops its own fighting style. Different opponents get different responses. It exploits patterns in your behavior.

#### 3A: Simulation Environment
- [ ] MuJoCo boxing environment (more accurate dynamics than PyBullet)
- [ ] Simplified opponent model (replay recorded human sequences)
- [ ] Reward function:
  - +1 for successful block/dodge
  - +2 for clean counter-punch
  - -1 for getting hit
  - -0.5 for excessive movement (energy conservation)

#### 3B: Policy Learning
- [ ] Train RL agent (PPO/SAC) in simulation
- [ ] Opponent curriculum: start with single moves, progress to combos
- [ ] Self-play option: two agents sparring
- [ ] Ablation: compare learned policy vs rule-based responses

#### 3C: Sim2Real Transfer
- [ ] Domain randomization (servo speed, latency, pose noise)
- [ ] System identification: measure real servo dynamics
- [ ] Calibration pipeline: record sim vs real for same commands, minimize gap
- [ ] Deploy learned policy to real hardware

**Key metric:** Learned policy outperforms rule-based responses on counter-punch success rate.

**Research output:** This phase is the publishable work. Real-time opponent modeling + sim2real for reactive robotics.

---

### Hardware Evolution (not a phase — runs in parallel)

The current hardware is a prototype. As the software gets smarter, the hardware needs to keep up.

| Stage | Hardware | Capability | When |
|-------|----------|-----------|------|
| **Now** | MG996R + pan-tilt brackets | Demonstrate moves, no contact | Phase 1-2 |
| **Next** | Dynamixel AX-12A or XL330 | Faster, position feedback, light contact | Phase 2-3 |
| **Future** | Brushless actuators, rigid aluminum frame | Real sparring force | Phase 3+ |

**Don't upgrade hardware until the software justifies it.** The MG996R setup is sufficient for Phases 1-2. Upgrade when you need speed/torque that hobby servos can't deliver.

---

## Current Hardware

```
Mac (Python, M3 Pro)
    │
    │ USB Serial (115200 baud)
    ▼
ESP32 DevKit C
    │
    │ I2C (GPIO 21=SDA, 22=SCL)
    ▼
PCA9685 PWM Driver (50Hz)
    │
    │ PWM Channels 0-6
    ▼
7x MG996R Servo Motors
    0: L_shoulder_tilt    3: R_shoulder_tilt
    1: L_shoulder_pan     4: R_shoulder_pan
    2: L_elbow            5: R_elbow
    6: Torso_yaw
```

**Frame:** Pan-tilt brackets (shoulder roll+tilt) + U-brackets (pan+upper arm) + servo horn (elbow). Arriving.

**Power:** External 6V PSU for servos, USB power for ESP32.

---

## Tech Stack

| Layer | Current | Future |
|-------|---------|--------|
| Perception | MediaPipe (2D pose, 130+ FPS) | + depth camera if needed |
| Classification | Rule-based move classifier | CNN/LSTM on pose sequences |
| Simulation | PyBullet | MuJoCo (Phase 3) |
| Control | Python → ESP32 serial → PCA9685 | ROS2 (if complexity justifies it) |
| ML/RL | — | PyTorch, Stable-Baselines3 |
| Data | JSONL recording | DuckDB for analysis |

---

## Key Engineering Lessons Learned

Documented in `docs/engineering_log.md`. Highlights:

1. **MediaPipe Z-axis is unreliable** — 3x noisier than X/Y. Don't fight it. Use 2D signals instead.
2. **Move classification > angle mirroring** — for boxing, classifying discrete moves produces cleaner, more useful output than copying noisy continuous angles.
3. **One Euro Filter** — adaptive smoothing that balances stability (when still) and responsiveness (when moving). Industry standard for motion tracking.
4. **Watchdog timing matters** — ESP32 watchdog + Python initialization delay = servos silently disabled. Always re-enable before main loop.
5. **DTR reset required** — macOS serial doesn't auto-reset ESP32. Must toggle DTR line.

---

## What This Project Is NOT

Being honest about scope prevents over-promising and keeps the work focused.

- **Not a product.** It's a research prototype and grad school portfolio piece.
- **Not a safe sparring partner.** MG996R servos can't take or deliver real punches. Don't hit the robot.
- **Not real-time RL.** Phase 3 RL trains in simulation. Hardware runs the trained policy.
- **Not a full humanoid.** Upper body only, fixed base, no legs/footwork.

---

*Project started: 2026*
*Maintainer: Charlie*
*Architecture: Move Classification + Playback (decided 2026-02-08)*
