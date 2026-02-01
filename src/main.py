"""Main entry point. Runs the shadow boxing control loop."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import argparse
import time

import cv2
import numpy as np
import pybullet as pb
import yaml

from src.angle_calculator import AngleCalculator
from src.camera import Camera
from src.motion_mapper import MappingConfig, MotionMapper
from src.pose_estimator import PoseEstimator
from src.simulated_robot import SimulatedRobot


def load_config(path: str) -> dict:
    config_path = Path(path)
    if not config_path.exists():
        print(f"Config not found: {path}, using defaults")
        return {}
    with open(config_path) as f:
        return yaml.safe_load(f) or {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Real Steel — Shadow Boxing Robot")
    parser.add_argument("--sim", action="store_true", help="Use PyBullet simulation")
    parser.add_argument(
        "--config", default="config/settings.yaml", help="Config file path"
    )
    parser.add_argument("--port", default=None, help="Serial port override")
    parser.add_argument(
        "--no-viz", action="store_true", help="Disable camera visualization"
    )
    parser.add_argument(
        "--calibrate", action="store_true", help="Run calibration test mode"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cfg = load_config(args.config)

    if args.calibrate:
        from src.calibration import CalibrationMode
        cal = CalibrationMode(cfg)
        report = cal.run()
        sys.exit(0 if report.overall_pass else 1)

    # Camera setup
    cam_cfg = cfg.get("camera", {})
    camera = Camera(
        device_id=cam_cfg.get("device_id", 0),
        width=cam_cfg.get("width", 640),
        height=cam_cfg.get("height", 480),
        fps=cam_cfg.get("fps", 30),
    )

    # Pose estimator setup
    pose_cfg = cfg.get("pose", {})
    model_path = pose_cfg.get("model_path", "data/pose_landmarker_lite.task")
    pose_estimator = PoseEstimator(
        model_path=model_path,
        min_visibility=pose_cfg.get("min_visibility", 0.5),
    )

    # Angle calculator
    angle_cfg = cfg.get("angles", {})
    angle_calculator = AngleCalculator(
        smoothing_factor=angle_cfg.get("smoothing_factor", 0.3),
    )

    # Motion mapper
    map_cfg = cfg.get("mapping", {})
    dead_zone_deg = map_cfg.get("dead_zone_deg", 3.0)
    mapping_config = MappingConfig(
        mirror_mode=map_cfg.get("mirror_mode", True),
        dead_zone=np.deg2rad(dead_zone_deg),
    )
    motion_mapper = MotionMapper(config=mapping_config)

    # Robot setup
    robot = None
    if args.sim:
        sim_cfg = cfg.get("simulation", {})
        urdf_path = sim_cfg.get("urdf_path", "urdf/real_steel.urdf")
        robot = SimulatedRobot(urdf_path=urdf_path, gui=True)
        if not robot.connect():
            print("Failed to connect to simulation")
            sys.exit(1)
        print("Simulation connected")

    # Open camera
    if not camera.open():
        print("Failed to open camera")
        if robot:
            robot.disconnect()
        sys.exit(1)
    print("Camera opened")

    # PyBullet camera state for keyboard control
    if args.sim:
        cam_info = pb.getDebugVisualizerCamera()
        cam_dist = cam_info[10]
        cam_yaw = cam_info[8]
        cam_pitch = cam_info[9]
        cam_target = list(cam_info[11])

    # FPS tracking
    fps_counter = 0
    fps_timer = time.time()
    current_fps = 0.0

    debug_timer = time.time()
    frame_num = 0

    print("Running pipeline. Press 'q' or ESC to quit.")
    print("PyBullet camera: arrow keys to rotate, +/- to zoom")
    print()
    print("Joint order: [L_roll, L_tilt, L_pan, L_elbow, R_roll, R_tilt, R_pan, R_elbow]")

    try:
        while True:
            frame = camera.read()
            if frame is None:
                continue

            # Pose estimation
            pose = pose_estimator.process(frame.image, frame.timestamp)

            # Angle calculation
            joint_angles = angle_calculator.calculate(pose)

            # Motion mapping and robot command
            servo_angles = None
            if joint_angles is not None and robot is not None:
                servo_angles = motion_mapper.map(joint_angles)
                robot.set_joint_positions(servo_angles.angles)

            # Step simulation + keyboard camera control
            if args.sim and robot is not None:
                robot.step()

                keys = pb.getKeyboardEvents()
                if pb.B3G_LEFT_ARROW in keys and keys[pb.B3G_LEFT_ARROW] & pb.KEY_IS_DOWN:
                    cam_yaw -= 2
                if pb.B3G_RIGHT_ARROW in keys and keys[pb.B3G_RIGHT_ARROW] & pb.KEY_IS_DOWN:
                    cam_yaw += 2
                if pb.B3G_UP_ARROW in keys and keys[pb.B3G_UP_ARROW] & pb.KEY_IS_DOWN:
                    cam_pitch -= 2
                if pb.B3G_DOWN_ARROW in keys and keys[pb.B3G_DOWN_ARROW] & pb.KEY_IS_DOWN:
                    cam_pitch += 2
                if ord("=") in keys and keys[ord("=")] & pb.KEY_IS_DOWN:
                    cam_dist = max(0.2, cam_dist - 0.05)
                if ord("-") in keys and keys[ord("-")] & pb.KEY_IS_DOWN:
                    cam_dist += 0.05

                pb.resetDebugVisualizerCamera(cam_dist, cam_yaw, cam_pitch, cam_target)

            # Debug print every second
            frame_num += 1
            now = time.time()
            if now - debug_timer >= 1.0:
                debug_timer = now
                if joint_angles is not None and servo_angles is not None and robot is not None:
                    human = np.rad2deg(joint_angles.to_array())
                    mapped = np.rad2deg(servo_angles.angles)
                    actual = np.rad2deg(robot.get_joint_state().positions)
                    fmt = lambda a: ", ".join(f"{v:+6.1f}" for v in a)
                    print(f"--- Frame {frame_num} ---")
                    print(f"  Human:  [{fmt(human)}]")
                    print(f"  Robot:  [{fmt(mapped)}]")
                    print(f"  Actual: [{fmt(actual)}]")
                elif not pose.is_valid:
                    print(f"--- Frame {frame_num} --- (no pose detected)")

            # Visualization
            if not args.no_viz:
                display = pose_estimator.draw(frame.image, pose)

                # FPS counter
                fps_counter += 1
                elapsed = time.time() - fps_timer
                if elapsed >= 5.0:
                    current_fps = fps_counter / elapsed
                    print(f"FPS: {current_fps:.1f}")
                    fps_counter = 0
                    fps_timer = time.time()

                # Draw FPS on frame
                cv2.putText(
                    display,
                    f"FPS: {current_fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                # Draw current angles
                if joint_angles is not None:
                    angles_deg = np.rad2deg(joint_angles.to_array())
                    labels = ["LR", "LT", "LP", "LE", "RR", "RT", "RP", "RE"]
                    y_start = 60
                    for i, (label, deg) in enumerate(zip(labels, angles_deg)):
                        cv2.putText(
                            display,
                            f"{label}: {deg:+6.1f}°",
                            (10, y_start + i * 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            1,
                        )

                cv2.imshow("Real Steel - Camera", display)

                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\nInterrupted")

    # Cleanup
    print("Shutting down...")
    if robot is not None:
        robot.home()
        robot.disconnect()
    camera.release()
    pose_estimator.close()
    cv2.destroyAllWindows()
    print("Done")


if __name__ == "__main__":
    main()
