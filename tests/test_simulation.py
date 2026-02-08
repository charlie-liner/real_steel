"""Simulation verification script.

Standalone: python tests/test_simulation.py  (opens GUI, runs 5-pose sequence)
Pytest:     pytest tests/test_simulation.py -v
"""

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.simulated_robot import SimulatedRobot

URDF_PATH = "urdf/real_steel.urdf"

# Joint limits from URDF (radians)
# [l_tilt, l_pan, l_elbow, r_tilt, r_pan, r_elbow, torso_yaw]
JOINT_LOWER = np.array([-1.5708, -1.5708, 0.0, -1.5708, -1.5708, 0.0, -1.5708])
JOINT_UPPER = np.array([1.5708, 1.5708, 2.3562, 1.5708, 1.5708, 2.3562, 1.5708])


def _make_robot():
    """Create and connect a headless SimulatedRobot."""
    robot = SimulatedRobot(urdf_path=URDF_PATH, gui=False)
    assert robot.connect(), "Failed to connect to simulation"
    return robot


def test_urdf_loads():
    """Verify URDF loads and all 7 joints are mapped."""
    robot = _make_robot()
    assert robot.is_connected()
    assert len(robot.joint_indices) == 7
    robot.disconnect()


def test_joint_control():
    """Verify joints reach commanded positions."""
    robot = _make_robot()

    target = np.array([0.5, 0.3, 1.0, -0.5, 0.3, 1.0, 0.2])
    robot.set_joint_positions(target)
    for _ in range(200):
        robot.step()

    state = robot.get_joint_state()
    for i in range(7):
        assert abs(state.positions[i] - target[i]) < 0.1, (
            f"Joint {i}: expected {target[i]:.3f}, got {state.positions[i]:.3f}"
        )

    robot.disconnect()


def test_joint_limits():
    """Verify joints cannot exceed URDF limits."""
    robot = _make_robot()

    # Command positions well beyond limits
    robot.set_joint_positions(np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]))
    for _ in range(200):
        robot.step()

    state = robot.get_joint_state()
    for i in range(7):
        assert state.positions[i] <= JOINT_UPPER[i] + 0.01, (
            f"Joint {i} exceeded upper limit: {state.positions[i]:.3f} > {JOINT_UPPER[i]:.3f}"
        )
        assert state.positions[i] >= JOINT_LOWER[i] - 0.01, (
            f"Joint {i} exceeded lower limit: {state.positions[i]:.3f} < {JOINT_LOWER[i]:.3f}"
        )

    robot.disconnect()


def test_home():
    """Verify home() returns all joints to zero."""
    robot = _make_robot()

    # Move to non-zero positions first
    robot.set_joint_positions(np.array([0.5, 0.3, 1.0, -0.5, 0.3, 1.0, 0.2]))
    for _ in range(200):
        robot.step()

    robot.home()

    state = robot.get_joint_state()
    for i in range(7):
        assert abs(state.positions[i]) < 0.1, (
            f"Joint {i} not at home: {state.positions[i]:.3f}"
        )

    robot.disconnect()


def main():
    import pybullet as p

    robot = SimulatedRobot(urdf_path=URDF_PATH, gui=True)

    if not robot.connect():
        print("Failed to connect!")
        return

    print("Testing joint control...")
    print("Camera: arrow keys to rotate, +/- to zoom, Ctrl+C to exit")

    # Camera state
    cam = p.getDebugVisualizerCamera()
    cam_dist = cam[10]
    cam_yaw = cam[8]
    cam_pitch = cam[9]
    cam_target = list(cam[11])

    try:
        # [l_tilt, l_pan, l_elbow, r_tilt, r_pan, r_elbow, torso_yaw] in radians
        sequences = [
            ([1.0, 0.0, 1.5, 1.0, 0.0, 1.5, 0.0], "Guard"),
            ([1.5, 0.0, 0.0, 1.0, 0.0, 1.5, 0.0], "Left jab"),
            ([1.0, 0.0, 1.5, 1.0, 0.0, 1.5, 0.0], "Guard"),
            ([1.0, 0.0, 1.5, 1.5, 0.0, 0.0, 0.3], "Right cross"),
            ([1.0, 0.0, 1.5, 1.0, 0.0, 1.5, 0.0], "Guard"),
            ([1.5, 0.0, 0.0, 1.0, 0.0, 1.5, 0.0], "One-two: jab"),
            ([1.0, 0.0, 1.5, 1.5, 0.0, 0.0, 0.3], "One-two: cross"),
            ([1.0, 0.0, 1.5, 1.0, 0.0, 1.5, 0.0], "Guard"),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "Home"),
        ]

        for target, label in sequences:
            print(f"\n{label}: {target}")
            robot.set_joint_positions(np.array(target))

            for _ in range(80):
                robot.step()
                time.sleep(1 / 240)

                # Handle keyboard camera control
                keys = p.getKeyboardEvents()
                if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                    cam_yaw -= 2
                if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                    cam_yaw += 2
                if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                    cam_pitch -= 2
                if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                    cam_pitch += 2
                if ord("=") in keys and keys[ord("=")] & p.KEY_IS_DOWN:
                    cam_dist = max(0.2, cam_dist - 0.05)
                if ord("-") in keys and keys[ord("-")] & p.KEY_IS_DOWN:
                    cam_dist += 0.05

                p.resetDebugVisualizerCamera(cam_dist, cam_yaw, cam_pitch, cam_target)

            state = robot.get_joint_state()
            print(f"  Reached: [{', '.join(f'{v:.3f}' for v in state.positions)}]")

        print("\nTest complete! Closing in 3 seconds...")
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
