"""Punch compensator module. Detects forward punches using 2D signals and adjusts joint angles."""

from dataclasses import dataclass

import numpy as np

from src.angle_calculator import JointAngles
from src.pose_estimator import PoseResult


@dataclass
class PunchState:
    """Per-arm punch detection state for debug/visualization."""

    left_intensity: float = 0.0
    right_intensity: float = 0.0
    left_wrist_rise: float = 0.0
    right_wrist_rise: float = 0.0
    left_foreshortening: float = 0.0
    right_foreshortening: float = 0.0
    left_velocity: float = 0.0
    right_velocity: float = 0.0


# Target punch pose in human angle space (before motion mapper offset).
# tilt=0 → horizontal → robot tilt = π/2 (max forward).
# elbow=0 → fully straight.
PUNCH_TARGET_TILT = 0.0
PUNCH_TARGET_ELBOW = 0.0


class PunchCompensator:
    """Detects forward punches from 2D pose signals and compensates joint angles.

    Uses three signals as an AND gate:
    1. Wrist rise: fist at or above shoulder height
    2. Foreshortening: arm appears shorter (forward extension)
    3. Wrist velocity: fast motion distinguishes punch from guard

    When detected, blends joint angles toward a known punch pose
    (arm horizontal forward, elbow straight) proportionally to intensity.
    """

    def __init__(
        self,
        wrist_rise_threshold: float = 0.0,
        foreshortening_threshold: float = 0.7,
        velocity_threshold: float = 0.02,
        attack_alpha: float = 0.4,
        decay_alpha: float = 0.3,
    ):
        self.wrist_rise_threshold = wrist_rise_threshold
        self.foreshortening_threshold = foreshortening_threshold
        self.velocity_threshold = velocity_threshold
        self.attack_alpha = attack_alpha
        self.decay_alpha = decay_alpha

        # Per-arm state
        self._prev_wrist: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._prev_timestamp: float | None = None
        self._intensity: dict[str, float] = {"left": 0.0, "right": 0.0}
        self._state = PunchState()

    def compensate(
        self, pose: PoseResult, joint_angles: JointAngles | None
    ) -> JointAngles | None:
        if joint_angles is None:
            return None

        if not pose.is_valid or not pose.keypoints:
            self._prev_wrist = {"left": None, "right": None}
            self._prev_timestamp = None
            return joint_angles

        dt = 0.0
        if self._prev_timestamp is not None:
            dt = joint_angles.timestamp - self._prev_timestamp

        # Compute signals for each arm
        left_raw = self._compute_raw_intensity(pose, "left", dt)
        right_raw = self._compute_raw_intensity(pose, "right", dt)

        # Update wrist history
        self._update_wrist_history(pose, joint_angles.timestamp)

        # Asymmetric EMA smoothing
        self._intensity["left"] = self._ema(self._intensity["left"], left_raw)
        self._intensity["right"] = self._ema(self._intensity["right"], right_raw)

        # Update debug state
        self._state.left_intensity = self._intensity["left"]
        self._state.right_intensity = self._intensity["right"]

        # Apply compensation
        return self._apply_compensation(joint_angles)

    def get_punch_state(self) -> PunchState:
        return self._state

    def _compute_raw_intensity(
        self, pose: PoseResult, side: str, dt: float
    ) -> float:
        shoulder = pose.keypoints.get(f"{side}_shoulder")
        elbow = pose.keypoints.get(f"{side}_elbow")
        wrist = pose.keypoints.get(f"{side}_wrist")
        other_shoulder = pose.keypoints.get(
            "right_shoulder" if side == "left" else "left_shoulder"
        )

        if not all([shoulder, elbow, wrist, other_shoulder]):
            return 0.0

        # Signal 1: Wrist rise — wrist at or above shoulder height (y inverted)
        wrist_rise = shoulder.y - wrist.y
        rise_active = wrist_rise > self.wrist_rise_threshold
        # Strength ramps from threshold to threshold + 0.15
        rise_strength = max(0.0, (wrist_rise - self.wrist_rise_threshold) / 0.15)
        rise_strength = min(rise_strength, 1.0)

        # Signal 2: Foreshortening — arm appears short relative to shoulder width
        shoulder_width = abs(shoulder.x - other_shoulder.x)
        if shoulder_width < 1e-4:
            return 0.0
        arm_2d_len = np.sqrt(
            (wrist.x - shoulder.x) ** 2 + (wrist.y - shoulder.y) ** 2
        )
        arm_ratio = arm_2d_len / shoulder_width
        foreshorten_active = arm_ratio < self.foreshortening_threshold
        foreshorten_strength = max(
            0.0, (self.foreshortening_threshold - arm_ratio) / 0.3
        )
        foreshorten_strength = min(foreshorten_strength, 1.0)

        # Signal 3: Wrist velocity — frame-to-frame wrist displacement / dt
        velocity = 0.0
        wrist_pos = np.array([wrist.x, wrist.y])
        if self._prev_wrist[side] is not None and dt > 1e-4:
            displacement = np.linalg.norm(wrist_pos - self._prev_wrist[side])
            velocity = displacement / dt
        velocity_active = velocity > self.velocity_threshold
        velocity_strength = max(0.0, (velocity - self.velocity_threshold) / 0.05)
        velocity_strength = min(velocity_strength, 1.0)

        # Store debug signals
        if side == "left":
            self._state.left_wrist_rise = wrist_rise
            self._state.left_foreshortening = arm_ratio
            self._state.left_velocity = velocity
        else:
            self._state.right_wrist_rise = wrist_rise
            self._state.right_foreshortening = arm_ratio
            self._state.right_velocity = velocity

        # AND gate: all three must be active
        if not (rise_active and foreshorten_active and velocity_active):
            return 0.0

        # Combined intensity = product of individual strengths
        return rise_strength * foreshorten_strength * velocity_strength

    def _update_wrist_history(self, pose: PoseResult, timestamp: float) -> None:
        for side in ("left", "right"):
            wrist = pose.keypoints.get(f"{side}_wrist")
            if wrist is not None:
                self._prev_wrist[side] = np.array([wrist.x, wrist.y])
            else:
                self._prev_wrist[side] = None
        self._prev_timestamp = timestamp

    def _ema(self, current: float, target: float) -> float:
        """Asymmetric EMA: fast attack, slow decay."""
        if target > current:
            alpha = self.attack_alpha
        else:
            alpha = self.decay_alpha
        return current + alpha * (target - current)

    def _apply_compensation(self, angles: JointAngles) -> JointAngles:
        """Blend joint angles toward punch target pose proportional to intensity.

        Instead of additive boost (which gets clamped by joint limits),
        blend toward the known-correct punch pose:
        - tilt → 0 (horizontal in human space → π/2 forward after mapper offset)
        - elbow → 0 (fully straight)
        """
        left_i = self._intensity["left"]
        right_i = self._intensity["right"]

        if left_i < 1e-3 and right_i < 1e-3:
            return angles

        return JointAngles(
            left_shoulder_tilt=(1.0 - left_i) * angles.left_shoulder_tilt
            + left_i * PUNCH_TARGET_TILT,
            left_shoulder_pan=angles.left_shoulder_pan,
            left_elbow=(1.0 - left_i) * angles.left_elbow
            + left_i * PUNCH_TARGET_ELBOW,
            right_shoulder_tilt=(1.0 - right_i) * angles.right_shoulder_tilt
            + right_i * PUNCH_TARGET_TILT,
            right_shoulder_pan=angles.right_shoulder_pan,
            right_elbow=(1.0 - right_i) * angles.right_elbow
            + right_i * PUNCH_TARGET_ELBOW,
            torso_yaw=angles.torso_yaw,
            timestamp=angles.timestamp,
            valid=angles.valid,
        )
