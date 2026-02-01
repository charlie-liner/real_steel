"""Calibration test mode. Guides user through known poses and diagnoses pipeline accuracy."""

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import pybullet as pb

from src.angle_calculator import AngleCalculator, JointAngles
from src.camera import Camera
from src.motion_mapper import DEFAULT_JOINT_LIMITS, JOINT_ORDER, MappingConfig, MotionMapper
from src.pose_estimator import PoseEstimator, PoseResult, Point3D
from src.simulated_robot import SimulatedRobot

# --- Constants ---

N_CAPTURE_FRAMES = 30
SETTLE_STEPS = 200
WINDOW_NAME = "Real Steel - Calibration"
FONT = cv2.FONT_HERSHEY_SIMPLEX
COLOR_PASS = (0, 255, 0)
COLOR_FAIL = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_YELLOW = (0, 255, 255)
COLOR_GRAY = (180, 180, 180)

HUMAN_JOINT_NAMES = [
    "left_shoulder_roll",
    "left_shoulder_tilt",
    "left_shoulder_pan",
    "left_elbow",
    "right_shoulder_roll",
    "right_shoulder_tilt",
    "right_shoulder_pan",
    "right_elbow",
]

HUMAN_JOINT_SHORT = ["LSR", "LST", "LSP", "LE", "RSR", "RST", "RSP", "RE"]
ROBOT_JOINT_SHORT = ["lSR", "lST", "lSP", "lE", "rSR", "rST", "rSP", "rE"]

KEYPOINT_NAMES = [
    "left_shoulder",
    "left_elbow",
    "left_wrist",
    "right_shoulder",
    "right_elbow",
    "right_wrist",
]


# --- Data Structures ---


@dataclass
class CalibrationPose:
    name: str
    description: str
    expected_human: np.ndarray  # shape (8,)
    tolerance_deg: np.ndarray  # shape (8,)
    relevant_joints: list[int]  # indices that matter for this pose


@dataclass
class CalibrationSample:
    frame_number: int
    timestamp: float
    keypoints: dict[str, Point3D] | None
    human_angles: np.ndarray | None  # shape (8,)
    robot_angles: np.ndarray | None  # shape (8,)
    pose_valid: bool


@dataclass
class JointResult:
    name: str
    expected_human_deg: float
    measured_human_deg: float
    std_human_deg: float
    expected_robot_deg: float
    measured_robot_deg: float
    error_human_deg: float
    tolerance_deg: float
    passed: bool
    relevant: bool


@dataclass
class PoseCalibrationResult:
    pose_name: str
    num_samples: int
    num_valid: int
    joint_results: list[JointResult]
    samples: list[CalibrationSample]
    overall_pass: bool


@dataclass
class CalibrationReport:
    pose_results: list[PoseCalibrationResult]
    overall_pass: bool


# --- Helpers ---


def compute_expected_robot_angles(human: np.ndarray, config: MappingConfig | None = None) -> np.ndarray:
    """Apply same transform as MotionMapper to derive expected robot angles."""
    if config is None:
        config = MappingConfig(mirror_mode=True, dead_zone=0.0)
    joint_angles = JointAngles(
        left_shoulder_roll=human[0], left_shoulder_tilt=human[1],
        left_shoulder_pan=human[2], left_elbow=human[3],
        right_shoulder_roll=human[4], right_shoulder_tilt=human[5],
        right_shoulder_pan=human[6], right_elbow=human[7],
        timestamp=0.0, valid=np.ones(8, dtype=bool),
    )
    mapper = MotionMapper(config=config)
    return mapper.map(joint_angles).angles


# --- Pose Definitions ---

CALIBRATION_POSES = [
    CalibrationPose(
        name="T-Pose",
        description="Stand with arms straight out to sides, palms down",
        expected_human=np.array([np.pi / 2, 0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0]),
        tolerance_deg=np.array([20.0, 20.0, 15.0, 10.0, 20.0, 20.0, 15.0, 10.0]),
        relevant_joints=[0, 1, 4, 5],
    ),
    CalibrationPose(
        name="Arms at Sides",
        description="Stand relaxed with arms hanging straight down",
        expected_human=np.array([0.0, -np.pi / 2, 0.0, 0.0, 0.0, -np.pi / 2, 0.0, 0.0]),
        tolerance_deg=np.array([20.0, 20.0, 15.0, 15.0, 20.0, 20.0, 15.0, 15.0]),
        relevant_joints=[0, 1, 4, 5],
    ),
    CalibrationPose(
        name="Left Arm Forward",
        description="Extend LEFT arm straight forward, right arm at side",
        expected_human=np.array([0.0, 0.0, 0.0, 0.0, 0.0, -np.pi / 2, 0.0, 0.0]),
        tolerance_deg=np.array([20.0, 15.0, 15.0, 10.0, 20.0, 20.0, 15.0, 15.0]),
        relevant_joints=[0, 1],
    ),
    CalibrationPose(
        name="Right Arm Forward",
        description="Extend RIGHT arm straight forward, left arm at side",
        expected_human=np.array([0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        tolerance_deg=np.array([20.0, 20.0, 15.0, 15.0, 20.0, 15.0, 15.0, 10.0]),
        relevant_joints=[4, 5],
    ),
    CalibrationPose(
        name="Left Elbow Bent",
        description="Left arm horizontal, bend LEFT elbow 90 degrees (forearm up)",
        expected_human=np.array([np.pi / 2, 0.0, 0.0, 1.2, 0.0, -np.pi / 2, 0.0, 0.0]),
        tolerance_deg=np.array([20.0, 15.0, 15.0, 15.0, 20.0, 20.0, 15.0, 15.0]),
        relevant_joints=[0, 1, 3],
    ),
    CalibrationPose(
        name="Guard Pose",
        description="Boxing guard: both fists up near chin, elbows bent",
        expected_human=np.array([np.pi / 6, -np.pi / 4, 0.0, np.pi / 2, np.pi / 6, -np.pi / 4, 0.0, np.pi / 2]),
        tolerance_deg=np.array([25.0, 25.0, 20.0, 20.0, 25.0, 25.0, 20.0, 20.0]),
        relevant_joints=[0, 1, 3, 4, 5, 7],
    ),
]


# --- Main Class ---


class CalibrationMode:
    def __init__(self, config: dict):
        self.config = config

        # Camera
        cam_cfg = config.get("camera", {})
        self.camera = Camera(
            device_id=cam_cfg.get("device_id", 0),
            width=cam_cfg.get("width", 640),
            height=cam_cfg.get("height", 480),
            fps=cam_cfg.get("fps", 30),
        )

        # Pose estimator
        pose_cfg = config.get("pose", {})
        model_path = pose_cfg.get("model_path", "data/pose_landmarker_lite.task")
        self.pose_estimator = PoseEstimator(
            model_path=model_path,
            min_visibility=pose_cfg.get("min_visibility", 0.5),
        )

        # Angle calculator with NO smoothing
        self.angle_calculator = AngleCalculator(smoothing_factor=0.0)

        # Motion mapper with NO dead zone
        self.motion_mapper = MotionMapper(
            config=MappingConfig(mirror_mode=True, dead_zone=0.0)
        )

        # Robot
        sim_cfg = config.get("simulation", {})
        urdf_path = sim_cfg.get("urdf_path", "urdf/real_steel.urdf")
        self.robot = SimulatedRobot(urdf_path=urdf_path, gui=True)

    def run(self) -> CalibrationReport:
        """Main entry point."""
        if not self.robot.connect():
            print("Failed to connect simulation")
            return CalibrationReport(pose_results=[], overall_pass=False)

        # Initialize PyBullet camera state for keyboard control
        cam_info = pb.getDebugVisualizerCamera()
        self._cam_dist = cam_info[10]
        self._cam_yaw = cam_info[8]
        self._cam_pitch = cam_info[9]
        self._cam_target = list(cam_info[11])

        if not self.camera.open():
            print("Failed to open camera")
            self.robot.disconnect()
            return CalibrationReport(pose_results=[], overall_pass=False)

        print("=" * 72)
        print("CALIBRATION MODE")
        print("  SPACE = capture/retry  |  ENTER = next pose  |  ESC = quit")
        print("  Arrow keys = rotate PyBullet camera  |  +/- = zoom")
        print("=" * 72)

        results = []
        aborted = False

        i = 0
        while i < len(CALIBRATION_POSES):
            pose = CALIBRATION_POSES[i]
            action, result = self._run_single_pose(i, pose)
            if action == "abort":
                aborted = True
                break
            if result is not None:
                self._print_pose_report(i, result, pose)
            if action == "next":
                if result is not None:
                    results.append(result)
                i += 1

        report = CalibrationReport(
            pose_results=results,
            overall_pass=all(r.overall_pass for r in results) and not aborted,
        )
        self._print_final_report(report)
        self._cleanup()
        return report

    def _run_single_pose(self, index: int, pose: CalibrationPose) -> tuple[str, PoseCalibrationResult | None]:
        """Run calibration for a single pose.
        Returns (action, result) where action is 'retry', 'next', or 'abort'."""
        expected_robot = compute_expected_robot_angles(pose.expected_human)

        # Move robot to expected pose
        self.robot.set_joint_positions(expected_robot)
        for _ in range(SETTLE_STEPS):
            self.robot.step()

        # Preview: show instructions, wait for SPACE to capture or ENTER to skip
        while True:
            frame = self.camera.read()
            if frame is None:
                continue

            display = self.pose_estimator.draw(
                frame.image,
                self.pose_estimator.process(frame.image, frame.timestamp),
            )
            self._draw_header(display, index, pose, "SPACE=capture  ENTER=skip  ESC=quit")
            self._draw_stick_figure(display, pose)
            cv2.imshow(WINDOW_NAME, display)
            self._poll_camera_keys()

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                return ("abort", None)
            if key == 13:  # ENTER — skip to next pose
                return ("next", None)
            if key == ord(" "):
                break

        # Countdown before capture
        for countdown in range(3, 0, -1):
            t_start = time.time()
            while time.time() - t_start < 1.0:
                frame = self.camera.read()
                if frame is None:
                    continue
                display = self.pose_estimator.draw(
                    frame.image,
                    self.pose_estimator.process(frame.image, frame.timestamp),
                )
                self._draw_header(display, index, pose, f"Hold pose... {countdown}", COLOR_YELLOW)
                self._draw_stick_figure(display, pose)
                cv2.imshow(WINDOW_NAME, display)
                self._poll_camera_keys()
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    return ("abort", None)

        # Capture frames
        self.angle_calculator.reset()
        self.motion_mapper.reset()
        samples = self._capture_frames(pose, index)

        # Analyze
        result = self._analyze_samples(pose, samples)

        # Show result: SPACE=retry same pose, ENTER=next pose, ESC=quit
        while True:
            frame = self.camera.read()
            if frame is None:
                continue

            display = self.pose_estimator.draw(
                frame.image,
                self.pose_estimator.process(frame.image, frame.timestamp),
            )
            status = "PASS" if result.overall_pass else "FAIL"
            color = COLOR_PASS if result.overall_pass else COLOR_FAIL
            self._draw_header(display, index, pose, f"{status}  |  SPACE=retry  ENTER=next", color)
            self._draw_result_table(display, result, pose)
            cv2.imshow(WINDOW_NAME, display)
            self._poll_camera_keys()

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                return ("abort", result)
            if key == ord(" "):
                return ("retry", result)
            if key == 13:  # ENTER
                return ("next", result)

        return ("next", result)

    def _capture_frames(self, pose: CalibrationPose, index: int) -> list[CalibrationSample]:
        """Capture N frames through the pipeline."""
        samples = []
        for n in range(N_CAPTURE_FRAMES):
            frame = self.camera.read()
            if frame is None:
                continue

            pose_result = self.pose_estimator.process(frame.image, frame.timestamp)
            joint_angles = self.angle_calculator.calculate(pose_result)

            human_arr = None
            robot_arr = None
            if joint_angles is not None:
                human_arr = joint_angles.to_array()
                servo = self.motion_mapper.map(joint_angles)
                robot_arr = servo.angles

            samples.append(CalibrationSample(
                frame_number=n,
                timestamp=frame.timestamp,
                keypoints=pose_result.keypoints if pose_result.is_valid else None,
                human_angles=human_arr,
                robot_angles=robot_arr,
                pose_valid=pose_result.is_valid,
            ))

            # Show progress
            display = self.pose_estimator.draw(frame.image, pose_result)
            self._draw_header(display, index, pose, f"Capturing... {n + 1}/{N_CAPTURE_FRAMES}", COLOR_YELLOW)
            cv2.imshow(WINDOW_NAME, display)
            self._poll_camera_keys()
            cv2.waitKey(1)

        return samples

    def _analyze_samples(self, pose: CalibrationPose, samples: list[CalibrationSample]) -> PoseCalibrationResult:
        """Compute statistics and pass/fail."""
        valid = [s for s in samples if s.pose_valid and s.human_angles is not None]

        if not valid:
            # All invalid — fail everything
            joint_results = []
            expected_robot = compute_expected_robot_angles(pose.expected_human)
            for i in range(len(HUMAN_JOINT_SHORT)):
                joint_results.append(JointResult(
                    name=HUMAN_JOINT_SHORT[i],
                    expected_human_deg=np.rad2deg(pose.expected_human[i]),
                    measured_human_deg=0.0,
                    std_human_deg=0.0,
                    expected_robot_deg=np.rad2deg(expected_robot[i]),
                    measured_robot_deg=0.0,
                    error_human_deg=999.0,
                    tolerance_deg=pose.tolerance_deg[i],
                    passed=False,
                    relevant=i in pose.relevant_joints,
                ))
            return PoseCalibrationResult(
                pose_name=pose.name,
                num_samples=len(samples),
                num_valid=0,
                joint_results=joint_results,
                samples=samples,
                overall_pass=False,
            )

        human_stack = np.array([s.human_angles for s in valid])
        robot_stack = np.array([s.robot_angles for s in valid])

        human_mean = human_stack.mean(axis=0)
        human_std = human_stack.std(axis=0)
        robot_mean = robot_stack.mean(axis=0)

        expected_robot = compute_expected_robot_angles(pose.expected_human)

        joint_results = []
        for i in range(len(HUMAN_JOINT_SHORT)):
            error = abs(np.rad2deg(human_mean[i]) - np.rad2deg(pose.expected_human[i]))
            relevant = i in pose.relevant_joints
            passed = error <= pose.tolerance_deg[i] if relevant else True

            joint_results.append(JointResult(
                name=HUMAN_JOINT_SHORT[i],
                expected_human_deg=np.rad2deg(pose.expected_human[i]),
                measured_human_deg=np.rad2deg(human_mean[i]),
                std_human_deg=np.rad2deg(human_std[i]),
                expected_robot_deg=np.rad2deg(expected_robot[i]),
                measured_robot_deg=np.rad2deg(robot_mean[i]),
                error_human_deg=error,
                tolerance_deg=pose.tolerance_deg[i],
                passed=passed,
                relevant=relevant,
            ))

        overall = all(jr.passed for jr in joint_results)

        return PoseCalibrationResult(
            pose_name=pose.name,
            num_samples=len(samples),
            num_valid=len(valid),
            joint_results=joint_results,
            samples=samples,
            overall_pass=overall,
        )

    # --- PyBullet Camera ---

    def _poll_camera_keys(self):
        """Handle arrow keys and +/- for PyBullet camera rotation/zoom."""
        keys = pb.getKeyboardEvents()
        if pb.B3G_LEFT_ARROW in keys and keys[pb.B3G_LEFT_ARROW] & pb.KEY_IS_DOWN:
            self._cam_yaw -= 2
        if pb.B3G_RIGHT_ARROW in keys and keys[pb.B3G_RIGHT_ARROW] & pb.KEY_IS_DOWN:
            self._cam_yaw += 2
        if pb.B3G_UP_ARROW in keys and keys[pb.B3G_UP_ARROW] & pb.KEY_IS_DOWN:
            self._cam_pitch -= 2
        if pb.B3G_DOWN_ARROW in keys and keys[pb.B3G_DOWN_ARROW] & pb.KEY_IS_DOWN:
            self._cam_pitch += 2
        if ord("=") in keys and keys[ord("=")] & pb.KEY_IS_DOWN:
            self._cam_dist = max(0.2, self._cam_dist - 0.05)
        if ord("-") in keys and keys[ord("-")] & pb.KEY_IS_DOWN:
            self._cam_dist += 0.05
        pb.resetDebugVisualizerCamera(self._cam_dist, self._cam_yaw, self._cam_pitch, self._cam_target)

    # --- Visualization ---

    def _draw_header(self, frame: np.ndarray, index: int, pose: CalibrationPose, status: str, color=COLOR_WHITE):
        """Draw title bar and status at top of frame."""
        h, w = frame.shape[:2]

        # Dark background strip
        cv2.rectangle(frame, (0, 0), (w, 60), (30, 30, 30), -1)

        # Pose title
        title = f"[{index + 1}/{len(CALIBRATION_POSES)}] {pose.name}"
        cv2.putText(frame, title, (10, 22), FONT, 0.6, COLOR_YELLOW, 2)

        # Description
        cv2.putText(frame, pose.description, (10, 42), FONT, 0.45, COLOR_GRAY, 1)

        # Status
        cv2.putText(frame, status, (10, 57), FONT, 0.4, color, 1)

    def _draw_stick_figure(self, frame: np.ndarray, pose: CalibrationPose):
        """Draw a simple stick figure showing target pose in bottom-left corner."""
        h, w = frame.shape[:2]
        ox, oy = 70, h - 70  # origin = torso center

        # Background
        cv2.rectangle(frame, (5, h - 140), (135, h - 5), (30, 30, 30), -1)
        cv2.putText(frame, "Target", (30, h - 125), FONT, 0.4, COLOR_GRAY, 1)

        # Torso
        cv2.line(frame, (ox, oy - 30), (ox, oy + 20), COLOR_WHITE, 2)

        # Shoulders
        cv2.circle(frame, (ox - 20, oy - 25), 3, COLOR_WHITE, -1)
        cv2.circle(frame, (ox + 20, oy - 25), 3, COLOR_WHITE, -1)

        # Draw arms based on expected angles
        # Order: [l_roll(0), l_tilt(1), l_pan(2), l_elbow(3),
        #         r_roll(4), r_tilt(5), r_pan(6), r_elbow(7)]
        arm_len = 30
        forearm_len = 25

        for side in ["left", "right"]:
            if side == "left":
                sx = ox - 20
                roll = pose.expected_human[0]
                tilt = pose.expected_human[1]
                elbow_angle = pose.expected_human[3]
                color = (0, 200, 0)
                sign = -1  # left extends leftward
            else:
                sx = ox + 20
                roll = pose.expected_human[4]
                tilt = pose.expected_human[5]
                elbow_angle = pose.expected_human[7]
                color = (200, 100, 0)
                sign = 1

            sy = oy - 25

            # Blend between "hanging down" and tilt-controlled angle based on roll.
            # roll=0 → arm hangs down (angle = -pi/2), roll=pi/2 → arm horizontal (use tilt).
            roll_factor = np.clip(roll / (np.pi / 2), 0.0, 1.0)
            effective_tilt = roll_factor * tilt + (1.0 - roll_factor) * (-np.pi / 2)

            # Upper arm: tilt from horizontal
            ex = sx + sign * int(arm_len * np.cos(effective_tilt))
            ey = sy - int(arm_len * np.sin(effective_tilt))
            cv2.line(frame, (sx, sy), (ex, ey), color, 2)

            # Forearm: elbow flexion relative to upper arm
            upper_angle = np.arctan2(-(ey - sy), sign * (ex - sx))
            fa_angle = upper_angle - sign * elbow_angle
            wx = ex + sign * int(forearm_len * np.cos(fa_angle))
            wy = ey - int(forearm_len * np.sin(fa_angle))
            cv2.line(frame, (ex, ey), (wx, wy), color, 2)

            # Joints
            cv2.circle(frame, (ex, ey), 3, color, -1)
            cv2.circle(frame, (wx, wy), 3, (0, 0, 200), -1)

    def _draw_result_table(self, frame: np.ndarray, result: PoseCalibrationResult, pose: CalibrationPose):
        """Draw angle comparison table on right side of frame."""
        h, w = frame.shape[:2]

        # Background panel — dynamic height based on joint count
        n_joints = len(HUMAN_JOINT_SHORT)
        panel_height = 65 + 35 + n_joints * 15 + 25 + n_joints * 15 + 10
        panel_x = w - 280
        cv2.rectangle(frame, (panel_x, 65), (w - 5, 65 + panel_height), (30, 30, 30), -1)

        # Header
        y = 82
        cv2.putText(frame, f"Samples: {result.num_valid}/{result.num_samples}", (panel_x + 5, y), FONT, 0.35, COLOR_GRAY, 1)
        y += 18

        cv2.putText(frame, "Joint  Expect  Actual   Err  Tol   ", (panel_x + 5, y), FONT, 0.33, COLOR_YELLOW, 1)
        y += 3
        cv2.line(frame, (panel_x + 5, y), (w - 10, y), COLOR_GRAY, 1)
        y += 14

        # Rows — human angles
        for jr in result.joint_results:
            marker = "*" if jr.relevant else " "
            if jr.relevant:
                color = COLOR_PASS if jr.passed else COLOR_FAIL
            else:
                color = COLOR_GRAY

            text = (
                f"{marker}{jr.name:>3s}  {jr.expected_human_deg:+6.1f}  "
                f"{jr.measured_human_deg:+6.1f}  {jr.error_human_deg:5.1f}  {jr.tolerance_deg:4.0f}"
            )
            cv2.putText(frame, text, (panel_x + 5, y), FONT, 0.33, color, 1)
            y += 15

        # Separator + robot angles header
        y += 5
        cv2.putText(frame, "Robot (after mirror+clamp):", (panel_x + 5, y), FONT, 0.33, COLOR_YELLOW, 1)
        y += 14

        expected_robot = compute_expected_robot_angles(pose.expected_human)
        for i, jr in enumerate(result.joint_results):
            text = (
                f" {ROBOT_JOINT_SHORT[i]:>3s}  {jr.expected_robot_deg:+6.1f}  "
                f"{jr.measured_robot_deg:+6.1f}"
            )
            cv2.putText(frame, text, (panel_x + 5, y), FONT, 0.33, COLOR_GRAY, 1)
            y += 15

    # --- Console Reporting ---

    def _print_pose_report(self, index: int, result: PoseCalibrationResult, pose: CalibrationPose):
        """Print detailed report for one pose."""
        print()
        print("=" * 72)
        print(f"POSE {index + 1}/{len(CALIBRATION_POSES)}: {result.pose_name}")
        print(f"  {pose.description}")
        print(f"  Samples: {result.num_valid}/{result.num_samples} valid")
        print("-" * 72)

        # Stage 1: Raw keypoints
        valid_samples = [s for s in result.samples if s.pose_valid and s.keypoints]
        if valid_samples:
            print("  Stage 1: Raw Keypoints (averaged world coords)")
            for kp_name in KEYPOINT_NAMES:
                wxs, wys, wzs, viss = [], [], [], []
                for s in valid_samples:
                    if kp_name in s.keypoints:
                        pt = s.keypoints[kp_name]
                        wxs.append(pt.world_x)
                        wys.append(pt.world_y)
                        wzs.append(pt.world_z)
                        viss.append(pt.visibility)
                if wxs:
                    print(
                        f"    {kp_name:>16s}: "
                        f"world=({np.mean(wxs):+.3f}, {np.mean(wys):+.3f}, {np.mean(wzs):+.3f})  "
                        f"vis={np.mean(viss):.2f}"
                    )
        else:
            print("  Stage 1: No valid keypoints captured!")

        # Stage 2: Human angles
        print()
        print("  Stage 2: Human Angles (AngleCalculator output)")
        print(f"    {'Joint':>20s}  {'Expected':>8s}  {'Measured':>8s}  {'StdDev':>7s}  {'Error':>6s}  {'Tol':>5s}  Status")
        for jr in result.joint_results:
            marker = "*" if jr.relevant else " "
            status = "PASS" if jr.passed else "FAIL"
            if not jr.relevant:
                status = "---"
            print(
                f"   {marker}{jr.name:>19s}  {jr.expected_human_deg:+7.1f}°  "
                f"{jr.measured_human_deg:+7.1f}°  {jr.std_human_deg:6.1f}°  "
                f"{jr.error_human_deg:5.1f}°  {jr.tolerance_deg:4.0f}°  {status}"
            )

        # Stage 3: Robot angles
        print()
        print("  Stage 3: Robot Angles (MotionMapper output)")
        print(f"    {'Joint':>20s}  {'Expected':>8s}  {'Measured':>8s}  {'Error':>6s}")
        expected_robot = compute_expected_robot_angles(pose.expected_human)
        for i, jr in enumerate(result.joint_results):
            error_robot = abs(jr.measured_robot_deg - jr.expected_robot_deg)
            print(
                f"    {ROBOT_JOINT_SHORT[i]:>20s}  {jr.expected_robot_deg:+7.1f}°  "
                f"{jr.measured_robot_deg:+7.1f}°  {error_robot:5.1f}°"
            )

        status_text = "PASS" if result.overall_pass else "FAIL"
        relevant_count = sum(1 for jr in result.joint_results if jr.relevant)
        passed_count = sum(1 for jr in result.joint_results if jr.relevant and jr.passed)
        print()
        print(f"  RESULT: {status_text} ({passed_count}/{relevant_count} relevant joints within tolerance)")

    def _print_final_report(self, report: CalibrationReport):
        """Print final summary."""
        print()
        print("=" * 72)
        print("CALIBRATION SUMMARY")
        print("=" * 72)

        for i, pr in enumerate(report.pose_results):
            status = "PASS" if pr.overall_pass else "FAIL"
            relevant = sum(1 for jr in pr.joint_results if jr.relevant)
            passed = sum(1 for jr in pr.joint_results if jr.relevant and jr.passed)
            dots = "." * (40 - len(pr.pose_name))
            print(f"  Pose {i + 1}: {pr.pose_name} {dots} {status} ({passed}/{relevant})")

        # Biggest errors
        all_errors = []
        for pr in report.pose_results:
            for jr in pr.joint_results:
                if jr.relevant:
                    all_errors.append((jr.error_human_deg, jr.name, pr.pose_name, jr.tolerance_deg))

        all_errors.sort(reverse=True)
        print()
        print("  Biggest errors:")
        for rank, (err, jname, pname, tol) in enumerate(all_errors[:5]):
            flag = " <<<" if err > tol else ""
            print(f"    {rank + 1}. {jname} @ \"{pname}\": {err:.1f}° (tolerance: {tol:.0f}°){flag}")

        overall = "PASS" if report.overall_pass else "FAIL"
        print()
        print(f"  OVERALL: {overall}")
        print("=" * 72)

    def _cleanup(self):
        """Release all resources."""
        self.camera.release()
        self.pose_estimator.close()
        self.robot.home()
        self.robot.disconnect()
        cv2.destroyAllWindows()
