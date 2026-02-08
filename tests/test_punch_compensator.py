"""Tests for PunchCompensator — hybrid 2D punch detection."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pytest

from src.angle_calculator import JointAngles
from src.pose_estimator import Point3D, PoseResult
from src.punch_compensator import PunchCompensator, PunchState


def make_point(nx: float, ny: float, wx: float = 0.0, wy: float = 0.0, wz: float = 0.0) -> Point3D:
    """Create a Point3D with normalized (x, y) for punch detection and world coords."""
    return Point3D(x=nx, y=ny, z=0.0, visibility=1.0, world_x=wx, world_y=wy, world_z=wz)


def make_pose(keypoints: dict[str, Point3D], timestamp: float = 0.0) -> PoseResult:
    return PoseResult(keypoints=keypoints, is_valid=True, timestamp=timestamp)


def make_joint_angles(**kwargs) -> JointAngles:
    defaults = dict(
        left_shoulder_roll=0.0,
        left_shoulder_tilt=0.5,
        left_shoulder_pan=0.0,
        left_elbow=1.0,
        right_shoulder_roll=0.0,
        right_shoulder_tilt=0.5,
        right_shoulder_pan=0.0,
        right_elbow=1.0,
        timestamp=0.0,
        valid=np.ones(8, dtype=bool),
    )
    defaults.update(kwargs)
    return JointAngles(**defaults)


def _jab_pose(side: str = "left", timestamp: float = 0.1) -> tuple[PoseResult, dict]:
    """Create a pose where the specified arm is in a jab position.

    Jab characteristics in normalized coords:
    - Wrist raised above shoulder (wrist.y < shoulder.y, y is inverted)
    - Arm appears short (foreshortened — wrist near shoulder in 2D)
    - (Velocity is computed across frames, so we need two frames)
    """
    # Shoulder width: left_shoulder at x=0.35, right_shoulder at x=0.65
    # Shoulder y at 0.4
    kp = {
        "left_shoulder": make_point(0.35, 0.40),
        "right_shoulder": make_point(0.65, 0.40),
    }

    if side == "left":
        # Jab: wrist raised (y=0.20), close to shoulder in 2D (foreshortened)
        kp["left_elbow"] = make_point(0.36, 0.32)
        kp["left_wrist"] = make_point(0.37, 0.22)
        # Other arm relaxed
        kp["right_elbow"] = make_point(0.65, 0.55)
        kp["right_wrist"] = make_point(0.65, 0.70)
    else:
        kp["right_elbow"] = make_point(0.64, 0.32)
        kp["right_wrist"] = make_point(0.63, 0.22)
        kp["left_elbow"] = make_point(0.35, 0.55)
        kp["left_wrist"] = make_point(0.35, 0.70)

    return make_pose(kp, timestamp=timestamp), kp


def _guard_pose(timestamp: float = 0.0) -> PoseResult:
    """Create a guard pose: wrists up, arms foreshortened, but static (no velocity)."""
    kp = {
        "left_shoulder": make_point(0.35, 0.40),
        "right_shoulder": make_point(0.65, 0.40),
        "left_elbow": make_point(0.36, 0.32),
        "left_wrist": make_point(0.37, 0.22),
        "right_elbow": make_point(0.64, 0.32),
        "right_wrist": make_point(0.63, 0.22),
    }
    return make_pose(kp, timestamp=timestamp)


def _hook_pose(side: str = "left", timestamp: float = 0.1) -> PoseResult:
    """Create a hook pose: wrist up, velocity present, but arm NOT foreshortened (sweeps wide)."""
    kp = {
        "left_shoulder": make_point(0.35, 0.40),
        "right_shoulder": make_point(0.65, 0.40),
    }

    if side == "left":
        # Hook: wrist out to the side (large 2D distance = NOT foreshortened)
        kp["left_elbow"] = make_point(0.15, 0.30)
        kp["left_wrist"] = make_point(0.05, 0.22)
        kp["right_elbow"] = make_point(0.65, 0.55)
        kp["right_wrist"] = make_point(0.65, 0.70)
    else:
        kp["right_elbow"] = make_point(0.85, 0.30)
        kp["right_wrist"] = make_point(0.95, 0.22)
        kp["left_elbow"] = make_point(0.35, 0.55)
        kp["left_wrist"] = make_point(0.35, 0.70)

    return make_pose(kp, timestamp=timestamp)


def _relaxed_pose(timestamp: float = 0.0) -> PoseResult:
    """Arms at sides — no punch signals."""
    kp = {
        "left_shoulder": make_point(0.35, 0.40),
        "right_shoulder": make_point(0.65, 0.40),
        "left_elbow": make_point(0.35, 0.55),
        "left_wrist": make_point(0.35, 0.70),
        "right_elbow": make_point(0.65, 0.55),
        "right_wrist": make_point(0.65, 0.70),
    }
    return make_pose(kp, timestamp=timestamp)


class TestJabDetection:
    def test_jab_detected_with_velocity(self):
        """A jab (all 3 signals active) should produce non-zero intensity."""
        comp = PunchCompensator()
        angles = make_joint_angles(left_elbow=1.0, left_shoulder_tilt=0.5)

        # Frame 0: relaxed (establishes wrist history)
        relaxed = _relaxed_pose(timestamp=0.0)
        comp.compensate(relaxed, make_joint_angles(timestamp=0.0))

        # Frame 1: jab position (wrist moved → velocity)
        jab_pose, _ = _jab_pose(side="left", timestamp=0.033)
        result = comp.compensate(jab_pose, make_joint_angles(timestamp=0.033, left_elbow=1.0, left_shoulder_tilt=0.5))

        state = comp.get_punch_state()
        assert state.left_intensity > 0.0, f"Expected positive left intensity, got {state.left_intensity}"

    def test_jab_blends_toward_punch_pose(self):
        """Punch compensation should blend tilt toward 0 (horizontal) and elbow toward 0 (straight)."""
        comp = PunchCompensator()
        original_tilt = -0.5  # arm angled down
        original_elbow = 1.0  # bent

        # Frame 0: relaxed
        relaxed = _relaxed_pose(timestamp=0.0)
        comp.compensate(relaxed, make_joint_angles(timestamp=0.0))

        # Frame 1: jab
        jab_pose, _ = _jab_pose(side="left", timestamp=0.033)
        angles = make_joint_angles(
            timestamp=0.033,
            left_shoulder_tilt=original_tilt,
            left_elbow=original_elbow,
        )
        result = comp.compensate(jab_pose, angles)

        # Tilt should move toward 0 (horizontal) — i.e. increase from -0.5
        assert result.left_shoulder_tilt > original_tilt, \
            f"Tilt should blend toward 0: {result.left_shoulder_tilt} <= {original_tilt}"
        # Elbow should move toward 0 (straight) — i.e. decrease from 1.0
        assert result.left_elbow < original_elbow, \
            f"Elbow should blend toward 0: {result.left_elbow} >= {original_elbow}"


class TestHookNotDetected:
    def test_hook_no_foreshortening(self):
        """A hook has velocity and wrist rise but NOT foreshortening — should not trigger."""
        comp = PunchCompensator()

        # Frame 0: relaxed
        relaxed = _relaxed_pose(timestamp=0.0)
        comp.compensate(relaxed, make_joint_angles(timestamp=0.0))

        # Frame 1: hook (arm sweeps wide, not foreshortened)
        hook = _hook_pose(side="left", timestamp=0.033)
        angles = make_joint_angles(timestamp=0.033, left_shoulder_tilt=0.5, left_elbow=1.0)
        result = comp.compensate(hook, angles)

        state = comp.get_punch_state()
        # Foreshortening ratio should be HIGH (arm extended wide), NOT below threshold
        assert state.left_foreshortening > 0.7, \
            f"Hook arm should NOT be foreshortened, ratio={state.left_foreshortening}"
        assert state.left_intensity == 0.0, \
            f"Hook should not trigger punch, intensity={state.left_intensity}"
        # Angles should be unchanged
        assert result.left_shoulder_tilt == 0.5
        assert result.left_elbow == 1.0


class TestGuardNotDetected:
    def test_guard_no_velocity(self):
        """Guard has wrist rise and foreshortening but NO velocity (static) — should not trigger."""
        comp = PunchCompensator()

        # Two identical guard frames — no velocity between them
        guard1 = _guard_pose(timestamp=0.0)
        comp.compensate(guard1, make_joint_angles(timestamp=0.0))

        guard2 = _guard_pose(timestamp=0.033)
        angles = make_joint_angles(timestamp=0.033, left_shoulder_tilt=0.5, left_elbow=1.0)
        result = comp.compensate(guard2, angles)

        state = comp.get_punch_state()
        assert state.left_velocity < 0.03, \
            f"Guard should have zero velocity, got {state.left_velocity}"
        assert state.left_intensity == 0.0, \
            f"Guard should not trigger punch, intensity={state.left_intensity}"
        assert result.left_shoulder_tilt == 0.5
        assert result.left_elbow == 1.0


class TestCompensationValues:
    def test_unchanged_roll_and_pan(self):
        """Punch compensation should NOT modify roll or pan angles."""
        comp = PunchCompensator()

        relaxed = _relaxed_pose(timestamp=0.0)
        comp.compensate(relaxed, make_joint_angles(timestamp=0.0))

        jab_pose, _ = _jab_pose(side="left", timestamp=0.033)
        angles = make_joint_angles(
            timestamp=0.033,
            left_shoulder_roll=0.3,
            left_shoulder_pan=0.2,
            left_shoulder_tilt=0.5,
            left_elbow=1.0,
        )
        result = comp.compensate(jab_pose, angles)

        assert result.left_shoulder_roll == 0.3
        assert result.left_shoulder_pan == 0.2


class TestNonePassthrough:
    def test_none_angles_returns_none(self):
        """If joint_angles is None, compensate should return None."""
        comp = PunchCompensator()
        pose = _relaxed_pose()
        result = comp.compensate(pose, None)
        assert result is None

    def test_invalid_pose_returns_angles_unchanged(self):
        """If pose is invalid, angles should pass through unchanged."""
        comp = PunchCompensator()
        invalid_pose = PoseResult(keypoints={}, is_valid=False, timestamp=0.0)
        angles = make_joint_angles(left_shoulder_tilt=0.5, left_elbow=1.0)
        result = comp.compensate(invalid_pose, angles)
        assert result.left_shoulder_tilt == 0.5
        assert result.left_elbow == 1.0


class TestMultiFrameSequence:
    def test_intensity_rises_then_falls(self):
        """Over a jab sequence, intensity should rise then decay."""
        comp = PunchCompensator()

        # Frame 0: relaxed
        relaxed = _relaxed_pose(timestamp=0.0)
        comp.compensate(relaxed, make_joint_angles(timestamp=0.0))

        # Frame 1: jab — intensity should rise
        jab_pose, _ = _jab_pose(side="left", timestamp=0.033)
        comp.compensate(jab_pose, make_joint_angles(timestamp=0.033))
        intensity_after_jab = comp.get_punch_state().left_intensity

        # Feed several more jab frames with slight wrist movement (ongoing punch motion)
        for i in range(5):
            t = 0.066 + i * 0.033
            jab_pose, _ = _jab_pose(side="left", timestamp=t)
            # Simulate ongoing forward motion: wrist drifts slightly each frame
            wrist = jab_pose.keypoints["left_wrist"]
            jab_pose.keypoints["left_wrist"] = make_point(
                wrist.x + (i + 1) * 0.002, wrist.y - (i + 1) * 0.002
            )
            comp.compensate(jab_pose, make_joint_angles(timestamp=t))

        peak_intensity = comp.get_punch_state().left_intensity
        assert peak_intensity > intensity_after_jab, \
            f"Intensity should accumulate: {peak_intensity} <= {intensity_after_jab}"

        # Now return to relaxed — intensity should decay
        for i in range(10):
            t = 0.3 + i * 0.033
            relaxed = _relaxed_pose(timestamp=t)
            comp.compensate(relaxed, make_joint_angles(timestamp=t))

        decayed_intensity = comp.get_punch_state().left_intensity
        assert decayed_intensity < peak_intensity, \
            f"Intensity should decay: {decayed_intensity} >= {peak_intensity}"


class TestRightArm:
    def test_right_jab_detected(self):
        """Right arm jab should produce right intensity, not left."""
        comp = PunchCompensator()

        relaxed = _relaxed_pose(timestamp=0.0)
        comp.compensate(relaxed, make_joint_angles(timestamp=0.0))

        jab_pose, _ = _jab_pose(side="right", timestamp=0.033)
        comp.compensate(jab_pose, make_joint_angles(timestamp=0.033))

        state = comp.get_punch_state()
        assert state.right_intensity > 0.0
        assert state.left_intensity == 0.0
