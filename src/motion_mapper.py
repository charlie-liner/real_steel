"""Motion mapping module. Maps human joint angles to robot servo angles with mirroring and scaling."""

from dataclasses import dataclass, field

import numpy as np

from src.angle_calculator import JointAngles


DEFAULT_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "l_shoulder_tilt": (-np.pi / 2, np.pi / 2),
    "l_shoulder_pan": (-np.pi / 2, np.pi / 2),
    "l_elbow": (0.0, 3 * np.pi / 4),
    "r_shoulder_tilt": (-np.pi / 2, np.pi / 2),
    "r_shoulder_pan": (-np.pi / 2, np.pi / 2),
    "r_elbow": (0.0, 3 * np.pi / 4),
    "torso_yaw": (-np.pi / 2, np.pi / 2),
}

JOINT_ORDER = [
    "l_shoulder_tilt",
    "l_shoulder_pan",
    "l_elbow",
    "r_shoulder_tilt",
    "r_shoulder_pan",
    "r_elbow",
    "torso_yaw",
]


@dataclass
class MappingConfig:
    mirror_mode: bool = True
    dead_zone: float = 0.05  # radians (~3 degrees)
    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: dict(DEFAULT_JOINT_LIMITS)
    )
    scale_factors: dict[str, float] = field(
        default_factory=lambda: {
            "l_shoulder_tilt": 1.0,
            "l_shoulder_pan": 1.0,
            "l_elbow": 1.3,
            "r_shoulder_tilt": 1.0,
            "r_shoulder_pan": 1.0,
            "r_elbow": 1.3,
            "torso_yaw": 1.0,
        }
    )
    offsets: dict[str, float] = field(
        default_factory=lambda: {
            "l_shoulder_tilt": np.pi / 2,  # human: 0=horizontal, -90=down; robot: 0=down, +90=forward
            "l_shoulder_pan": 0.0,
            "l_elbow": 0.0,
            "r_shoulder_tilt": np.pi / 2,  # same offset for right arm
            "r_shoulder_pan": 0.0,
            "r_elbow": 0.0,
            "torso_yaw": 0.0,
        }
    )


@dataclass
class ServoAngles:
    angles: np.ndarray  # shape (7,), radians
    timestamp: float


class MotionMapper:
    def __init__(self, config: MappingConfig | None = None):
        self.config = config or MappingConfig()
        self.prev_output: np.ndarray | None = None

        # Pre-compute numpy arrays from config dicts for vectorized operations
        self._scale = np.array([self.config.scale_factors[n] for n in JOINT_ORDER])
        self._offset = np.array([self.config.offsets[n] for n in JOINT_ORDER])
        self._limits_low = np.array([self.config.joint_limits[n][0] for n in JOINT_ORDER])
        self._limits_high = np.array([self.config.joint_limits[n][1] for n in JOINT_ORDER])

    def map(self, human_angles: JointAngles) -> ServoAngles:
        # Step 1: Mirror (swap left/right, negate pan, negate torso_yaw)
        if self.config.mirror_mode:
            angles = np.array([
                human_angles.right_shoulder_tilt,
                -human_angles.right_shoulder_pan,
                human_angles.right_elbow,
                human_angles.left_shoulder_tilt,
                -human_angles.left_shoulder_pan,
                human_angles.left_elbow,
                -human_angles.torso_yaw,
            ])
        else:
            angles = human_angles.to_array()

        # Step 2: Scale + offset (vectorized)
        angles *= self._scale
        angles += self._offset

        # Step 3: Clamp to joint limits (vectorized)
        np.clip(angles, self._limits_low, self._limits_high, out=angles)

        # Step 4: Dead zone — keep previous value if change is small (vectorized)
        if self.prev_output is not None:
            mask = np.abs(angles - self.prev_output) < self.config.dead_zone
            angles[mask] = self.prev_output[mask]

        self.prev_output = angles.copy()
        return ServoAngles(angles=angles, timestamp=human_angles.timestamp)

    def reset(self) -> None:
        self.prev_output = None
