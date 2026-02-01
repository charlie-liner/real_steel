"""Motion mapping module. Maps human joint angles to robot servo angles with mirroring and scaling."""

from dataclasses import dataclass, field

import numpy as np

from src.angle_calculator import JointAngles


DEFAULT_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "l_shoulder_roll": (-0.35, 2.36),
    "l_shoulder_tilt": (-np.pi / 2, np.pi / 2),
    "l_shoulder_pan": (-np.pi / 2, np.pi / 2),
    "l_elbow": (0.0, 3 * np.pi / 4),
    "r_shoulder_roll": (-0.35, 2.36),
    "r_shoulder_tilt": (-np.pi / 2, np.pi / 2),
    "r_shoulder_pan": (-np.pi / 2, np.pi / 2),
    "r_elbow": (0.0, 3 * np.pi / 4),
}

JOINT_ORDER = [
    "l_shoulder_roll",
    "l_shoulder_tilt",
    "l_shoulder_pan",
    "l_elbow",
    "r_shoulder_roll",
    "r_shoulder_tilt",
    "r_shoulder_pan",
    "r_elbow",
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
            "l_shoulder_roll": 1.0,
            "l_shoulder_tilt": 1.0,
            "l_shoulder_pan": 1.0,
            "l_elbow": 1.3,
            "r_shoulder_roll": 1.0,
            "r_shoulder_tilt": 1.0,
            "r_shoulder_pan": 1.0,
            "r_elbow": 1.3,
        }
    )
    offsets: dict[str, float] = field(
        default_factory=lambda: {
            "l_shoulder_roll": 0.0,
            "l_shoulder_tilt": np.pi / 2,  # human: 0=horizontal, -90=down; robot: 0=down, +90=forward
            "l_shoulder_pan": 0.0,
            "l_elbow": 0.0,
            "r_shoulder_roll": 0.0,
            "r_shoulder_tilt": np.pi / 2,  # same offset for right arm
            "r_shoulder_pan": 0.0,
            "r_elbow": 0.0,
        }
    )


@dataclass
class ServoAngles:
    angles: np.ndarray  # shape (8,), radians
    timestamp: float


class MotionMapper:
    def __init__(self, config: MappingConfig | None = None):
        self.config = config or MappingConfig()
        self.prev_output: np.ndarray | None = None

    def map(self, human_angles: JointAngles) -> ServoAngles:
        # Step 1: Mirror (swap left/right, negate pan)
        if self.config.mirror_mode:
            angles = np.array([
                human_angles.right_shoulder_roll,
                human_angles.right_shoulder_tilt,
                -human_angles.right_shoulder_pan,
                human_angles.right_elbow,
                human_angles.left_shoulder_roll,
                human_angles.left_shoulder_tilt,
                -human_angles.left_shoulder_pan,
                human_angles.left_elbow,
            ])
        else:
            angles = human_angles.to_array()

        # Step 2: Scale + offset
        for i, name in enumerate(JOINT_ORDER):
            angles[i] *= self.config.scale_factors[name]
            angles[i] += self.config.offsets[name]

        # Step 3: Clamp to joint limits
        for i, name in enumerate(JOINT_ORDER):
            low, high = self.config.joint_limits[name]
            angles[i] = np.clip(angles[i], low, high)

        # Step 4: Dead zone — keep previous value if change is small
        if self.prev_output is not None:
            for i in range(len(angles)):
                if abs(angles[i] - self.prev_output[i]) < self.config.dead_zone:
                    angles[i] = self.prev_output[i]

        self.prev_output = angles.copy()
        return ServoAngles(angles=angles, timestamp=human_angles.timestamp)

    def reset(self) -> None:
        self.prev_output = None
