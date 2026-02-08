"""Interactive joint diagnostic. Sliders in PyBullet GUI to control each joint."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import time

import numpy as np
import pybullet as pb

from src.simulated_robot import SimulatedRobot

JOINT_NAMES = [
    "L tilt (fwd/back)",
    "L pan (rotate)",
    "L elbow",
    "R tilt (fwd/back)",
    "R pan (rotate)",
    "R elbow",
    "Torso yaw",
]

JOINT_LIMITS = [
    (-90, 90),    # l_tilt
    (-90, 90),    # l_pan
    (0, 135),     # l_elbow
    (-90, 90),    # r_tilt
    (-90, 90),    # r_pan
    (0, 135),     # r_elbow
    (-90, 90),    # torso_yaw
]


def main():
    robot = SimulatedRobot(urdf_path="urdf/real_steel.urdf", gui=True)

    if not robot.connect():
        print("Failed to connect")
        sys.exit(1)

    pb.resetDebugVisualizerCamera(
        cameraDistance=0.6,
        cameraYaw=90,
        cameraPitch=-15,
        cameraTargetPosition=[0, 0, 0.55],
    )

    # Create sliders for each joint
    sliders = []
    for name, (lo, hi) in zip(JOINT_NAMES, JOINT_LIMITS):
        slider = pb.addUserDebugParameter(name, lo, hi, 0)
        sliders.append(slider)

    print("=" * 50)
    print("INTERACTIVE JOINT DIAGNOSTIC (3 DOF per arm + torso)")
    print("Drag the sliders in the PyBullet window.")
    print("Arrow keys to rotate camera, +/- to zoom.")
    print("Close PyBullet window to quit.")
    print("=" * 50)

    cam_info = pb.getDebugVisualizerCamera()
    cam_dist, cam_yaw, cam_pitch = cam_info[10], cam_info[8], cam_info[9]
    cam_target = list(cam_info[11])

    try:
        while pb.isConnected():
            positions = np.zeros(7)
            for i, slider in enumerate(sliders):
                deg = pb.readUserDebugParameter(slider)
                positions[i] = np.deg2rad(deg)

            robot.set_joint_positions(positions)
            pb.stepSimulation()
            time.sleep(1 / 60)

            # Arrow key camera control
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
    except Exception:
        pass

    robot.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
