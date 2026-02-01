"""Headless tests for angle calculation and motion mapping."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pytest

from src.angle_calculator import AngleCalculator, JointAngles
from src.motion_mapper import MappingConfig, MotionMapper, ServoAngles
from src.pose_estimator import Point3D, PoseResult


def make_point(wx: float, wy: float, wz: float) -> Point3D:
    """Helper to create a Point3D with world coordinates."""
    return Point3D(x=0.5, y=0.5, z=0.0, visibility=1.0, world_x=wx, world_y=wy, world_z=wz)


def make_pose(keypoints: dict[str, Point3D], timestamp: float = 0.0) -> PoseResult:
    """Helper to create a valid PoseResult from keypoints dict."""
    return PoseResult(keypoints=keypoints, is_valid=True, timestamp=timestamp)


def make_joint_angles(**kwargs) -> JointAngles:
    """Helper to create JointAngles with defaults of 0."""
    defaults = dict(
        left_shoulder_roll=0.0,
        left_shoulder_tilt=0.0,
        left_shoulder_pan=0.0,
        left_elbow=0.0,
        right_shoulder_roll=0.0,
        right_shoulder_tilt=0.0,
        right_shoulder_pan=0.0,
        right_elbow=0.0,
        timestamp=0.0,
        valid=np.ones(8, dtype=bool),
    )
    defaults.update(kwargs)
    return JointAngles(**defaults)


class TestElbowAngle:
    def test_elbow_angle_straight(self):
        """Arm fully extended — elbow angle should be ~0."""
        keypoints = {
            "left_shoulder": make_point(0.0, 0.0, 0.0),
            "left_elbow": make_point(0.0, 0.0, -0.15),
            "left_wrist": make_point(0.0, 0.0, -0.30),
            "right_shoulder": make_point(0.0, 0.0, 0.0),
            "right_elbow": make_point(0.0, 0.0, -0.15),
            "right_wrist": make_point(0.0, 0.0, -0.30),
        }
        pose = make_pose(keypoints)
        calc = AngleCalculator(smoothing_factor=0.0)
        result = calc.calculate(pose)
        assert result is not None
        assert abs(result.left_elbow) < 0.1
        assert abs(result.right_elbow) < 0.1

    def test_elbow_angle_bent(self):
        """Arm bent at 90 degrees — elbow angle should be ~pi/2."""
        keypoints = {
            "left_shoulder": make_point(0.0, 0.0, 0.0),
            "left_elbow": make_point(0.0, 0.0, -0.15),
            "left_wrist": make_point(0.15, 0.0, -0.15),
            "right_shoulder": make_point(0.0, 0.0, 0.0),
            "right_elbow": make_point(0.0, 0.0, -0.15),
            "right_wrist": make_point(0.15, 0.0, -0.15),
        }
        pose = make_pose(keypoints)
        calc = AngleCalculator(smoothing_factor=0.0)
        result = calc.calculate(pose)
        assert result is not None
        assert abs(result.left_elbow - np.pi / 2) < 0.1
        assert abs(result.right_elbow - np.pi / 2) < 0.1


class TestShoulderTilt:
    def test_shoulder_tilt_horizontal(self):
        """Arm pointing horizontally — shoulder tilt should be ~0."""
        keypoints = {
            "left_shoulder": make_point(0.0, 0.0, 0.0),
            "left_elbow": make_point(0.15, 0.0, 0.0),
            "left_wrist": make_point(0.30, 0.0, 0.0),
            "right_shoulder": make_point(0.0, 0.0, 0.0),
            "right_elbow": make_point(-0.15, 0.0, 0.0),
            "right_wrist": make_point(-0.30, 0.0, 0.0),
        }
        pose = make_pose(keypoints)
        calc = AngleCalculator(smoothing_factor=0.0)
        result = calc.calculate(pose)
        assert result is not None
        assert abs(result.left_shoulder_tilt) < 0.1
        assert abs(result.right_shoulder_tilt) < 0.1

    def test_shoulder_tilt_raised(self):
        """Arm pointing straight up — shoulder tilt should be ~pi/2."""
        # MediaPipe Y points down, so "up" means negative Y
        keypoints = {
            "left_shoulder": make_point(0.0, 0.0, 0.0),
            "left_elbow": make_point(0.0, -0.15, 0.0),
            "left_wrist": make_point(0.0, -0.30, 0.0),
            "right_shoulder": make_point(0.0, 0.0, 0.0),
            "right_elbow": make_point(0.0, -0.15, 0.0),
            "right_wrist": make_point(0.0, -0.30, 0.0),
        }
        pose = make_pose(keypoints)
        calc = AngleCalculator(smoothing_factor=0.0)
        result = calc.calculate(pose)
        assert result is not None
        assert abs(result.left_shoulder_tilt - np.pi / 2) < 0.1
        assert abs(result.right_shoulder_tilt - np.pi / 2) < 0.1


class TestMirrorMode:
    def test_mirror_mode(self):
        """Mirror mode swaps left/right and negates pan angles.

        JOINT_ORDER: [l_roll(0), l_tilt(1), l_pan(2), l_elbow(3),
                      r_roll(4), r_tilt(5), r_pan(6), r_elbow(7)]

        Mirror mapping:
          robot l_roll  = human r_roll
          robot l_tilt  = human r_tilt
          robot l_pan   = -human r_pan
          robot l_elbow = human r_elbow
          robot r_roll  = human l_roll
          robot r_tilt  = human l_tilt
          robot r_pan   = -human l_pan
          robot r_elbow = human l_elbow
        """
        human = make_joint_angles(
            left_shoulder_roll=0.2,
            left_shoulder_tilt=-0.5,
            left_shoulder_pan=0.3,
            left_elbow=1.0,
            right_shoulder_roll=0.1,
            right_shoulder_tilt=-0.6,
            right_shoulder_pan=0.4,
            right_elbow=0.8,
        )
        mapper = MotionMapper(MappingConfig(mirror_mode=True, dead_zone=0.0))
        result = mapper.map(human)

        scale = mapper.config.scale_factors
        offset = mapper.config.offsets

        # Robot left ← human right (pan negated)
        assert abs(result.angles[0] - (human.right_shoulder_roll * scale["l_shoulder_roll"] + offset["l_shoulder_roll"])) < 0.01
        assert abs(result.angles[1] - (human.right_shoulder_tilt * scale["l_shoulder_tilt"] + offset["l_shoulder_tilt"])) < 0.01
        assert abs(result.angles[2] - (-human.right_shoulder_pan * scale["l_shoulder_pan"] + offset["l_shoulder_pan"])) < 0.01
        assert abs(result.angles[3] - (human.right_elbow * scale["l_elbow"] + offset["l_elbow"])) < 0.01

        # Robot right ← human left (pan negated)
        assert abs(result.angles[4] - (human.left_shoulder_roll * scale["r_shoulder_roll"] + offset["r_shoulder_roll"])) < 0.01
        assert abs(result.angles[5] - (human.left_shoulder_tilt * scale["r_shoulder_tilt"] + offset["r_shoulder_tilt"])) < 0.01
        assert abs(result.angles[6] - (-human.left_shoulder_pan * scale["r_shoulder_pan"] + offset["r_shoulder_pan"])) < 0.01
        assert abs(result.angles[7] - (human.left_elbow * scale["r_elbow"] + offset["r_elbow"])) < 0.01


class TestJointLimits:
    def test_joint_limits_clamping(self):
        """Values beyond limits should be clamped."""
        human = make_joint_angles(
            left_shoulder_roll=5.0,
            left_shoulder_tilt=5.0,
            left_shoulder_pan=5.0,
            left_elbow=5.0,
            right_shoulder_roll=-5.0,
            right_shoulder_tilt=-5.0,
            right_shoulder_pan=-5.0,
            right_elbow=-5.0,
        )
        config = MappingConfig(mirror_mode=False, dead_zone=0.0)
        mapper = MotionMapper(config)
        result = mapper.map(human)

        from src.motion_mapper import JOINT_ORDER
        for i, name in enumerate(JOINT_ORDER):
            low, high = config.joint_limits[name]
            assert result.angles[i] >= low - 1e-6, f"{name} below lower limit"
            assert result.angles[i] <= high + 1e-6, f"{name} above upper limit"


class TestDeadZone:
    def test_dead_zone(self):
        """Small changes within dead zone should not update output."""
        config = MappingConfig(mirror_mode=False, dead_zone=0.05)
        mapper = MotionMapper(config)

        # l_shoulder_pan is at index 2 in JOINT_ORDER
        PAN_IDX = 2

        # Initial mapping
        initial = make_joint_angles(left_shoulder_pan=0.5, timestamp=0.0)
        result1 = mapper.map(initial)

        # Tiny change (within dead zone)
        small_change = make_joint_angles(left_shoulder_pan=0.52, timestamp=1.0)
        result2 = mapper.map(small_change)
        assert abs(result2.angles[PAN_IDX] - result1.angles[PAN_IDX]) < 1e-6, "Dead zone failed: small change got through"

        # Large change (beyond dead zone)
        big_change = make_joint_angles(left_shoulder_pan=0.7, timestamp=2.0)
        result3 = mapper.map(big_change)
        assert abs(result3.angles[PAN_IDX] - 0.7) < 1e-6, "Dead zone failed: large change was blocked"


class TestSmoothing:
    def test_smoothing(self):
        """EMA smoothing should follow alpha * prev + (1-alpha) * current."""
        alpha = 0.3
        calc = AngleCalculator(smoothing_factor=alpha)

        # First frame: arm horizontal, no smoothing
        kp1 = {
            "left_shoulder": make_point(0.0, 0.0, 0.0),
            "left_elbow": make_point(0.15, 0.0, 0.0),
            "left_wrist": make_point(0.30, 0.0, 0.0),
            "right_shoulder": make_point(0.0, 0.0, 0.0),
            "right_elbow": make_point(-0.15, 0.0, 0.0),
            "right_wrist": make_point(-0.30, 0.0, 0.0),
        }
        pose1 = make_pose(kp1, timestamp=0.0)
        result1 = calc.calculate(pose1)
        assert result1 is not None

        # Second frame: arm raised (different angles)
        kp2 = {
            "left_shoulder": make_point(0.0, 0.0, 0.0),
            "left_elbow": make_point(0.0, -0.15, 0.0),
            "left_wrist": make_point(0.0, -0.30, 0.0),
            "right_shoulder": make_point(0.0, 0.0, 0.0),
            "right_elbow": make_point(0.0, -0.15, 0.0),
            "right_wrist": make_point(0.0, -0.30, 0.0),
        }
        pose2 = make_pose(kp2, timestamp=0.033)
        result2 = calc.calculate(pose2)
        assert result2 is not None

        # Compute expected: alpha * result1 + (1-alpha) * raw_result2
        # Raw result2 shoulder tilt should be ~pi/2 (arm straight up)
        # Result1 shoulder tilt should be ~0 (arm horizontal)
        # So smoothed = 0.3 * 0 + 0.7 * (pi/2) = 0.7 * pi/2
        expected_tilt = alpha * result1.left_shoulder_tilt + (1 - alpha) * (np.pi / 2)
        assert abs(result2.left_shoulder_tilt - expected_tilt) < 0.15
