"""Boxing move demo. Plays pre-programmed punch sequences to evaluate 4-DOF joint structure."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import time

import numpy as np
import pybullet as pb

from src.simulated_robot import SimulatedRobot

# Joint order: [l_roll, l_tilt, l_pan, l_elbow, r_roll, r_tilt, r_pan, r_elbow] in DEGREES
#
# roll:  0=arm at side, +90=arm spread horizontal (T-pose)
# tilt:  0=neutral, +90=swing forward, -90=swing backward
# pan:   0=center, ±90=rotate around local vertical
# elbow: 0=straight, +90=bent 90deg

GUARD = [30, 40, 0, 100,  30, 40, 0, 100]
REST  = [0, 0, 0, 0,  0, 0, 0, 0]

# Each move: list of (keyframe_deg, transition_seconds)
MOVES = [
    ("Rest → Guard", [
        (REST, 0.1),
        (GUARD, 0.2),
    ]),
    ("Left Jab", [
        (GUARD, 0.1),
        ([30, 70, 0, 5,  30, 40, 0, 100], 0.06),    # punch out
        (GUARD, 0.1),
        ([30, 70, 0, 5,  30, 40, 0, 100], 0.06),    # double jab
        (GUARD, 0.1),
    ]),
    ("Right Cross", [
        (GUARD, 0.1),
        ([30, 40, 0, 100,  30, 70, 0, 5], 0.07),    # punch out
        (GUARD, 0.1),
    ]),
    ("Left Uppercut", [
        (GUARD, 0.1),
        ([10, -20, 0, 120,  30, 40, 0, 100], 0.06),  # drop low, arm tucked
        ([50, 90, 0, 40,  30, 40, 0, 100], 0.05),     # explode upward
        (GUARD, 0.1),
    ]),
    ("Right Uppercut", [
        (GUARD, 0.1),
        ([30, 40, 0, 100,  10, -20, 0, 120], 0.06),  # drop low, arm tucked
        ([30, 40, 0, 100,  50, 90, 0, 40], 0.05),     # explode upward
        (GUARD, 0.1),
    ]),
    ("Left Hook", [
        (GUARD, 0.1),
        ([100, 10, -20, 90,  30, 40, 0, 100], 0.07),  # arm wide outside
        ([15, 60, 50, 90,  30, 40, 0, 100], 0.06),     # sweep hard across torso
        (GUARD, 0.1),
    ]),
    ("Right Hook", [
        (GUARD, 0.1),
        ([30, 40, 0, 100,  100, 10, 20, 90], 0.07),   # arm wide outside
        ([30, 40, 0, 100,  15, 60, -50, 90], 0.06),    # sweep hard across torso
        (GUARD, 0.1),
    ]),
    ("Combo: Jab → Cross → Left Hook", [
        (GUARD, 0.1),
        ([30, 70, 0, 5,  30, 40, 0, 100], 0.05),    # jab
        (GUARD, 0.07),
        ([30, 40, 0, 100,  30, 70, 0, 5], 0.05),    # cross
        (GUARD, 0.07),
        ([100, 10, -20, 90,  30, 40, 0, 100], 0.07),  # hook wind wide
        ([15, 60, 50, 90,  30, 40, 0, 100], 0.06),    # hook sweep across
        (GUARD, 0.1),
    ]),
    ("T-Pose (arms spread)", [
        (GUARD, 0.1),
        ([90, 0, 0, 0,  90, 0, 0, 0], 0.3),         # T-pose
    ]),
    ("Guard → Rest", [
        ([90, 0, 0, 0,  90, 0, 0, 0], 0.2),
        (REST, 0.1),
    ]),
]


def cam_control(keys, cam_dist, cam_yaw, cam_pitch, cam_target):
    """Handle arrow key camera controls."""
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
    return cam_dist, cam_yaw, cam_pitch


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

    cam_info = pb.getDebugVisualizerCamera()
    cam_dist, cam_yaw, cam_pitch = cam_info[10], cam_info[8], cam_info[9]
    cam_target = list(cam_info[11])

    print("=" * 50)
    print("BOXING DEMO (4 DOF per arm)")
    print("Arrow keys to rotate camera, +/- to zoom")
    print("=" * 50)

    current = np.array(REST, dtype=float)

    for move_name, keyframes in MOVES:
        print(f"\n>>> {move_name}")

        for target_deg, duration in keyframes:
            target = np.array(target_deg, dtype=float)
            steps = max(int(duration * 120), 1)

            for i in range(steps):
                t = (i + 1) / steps
                t = t * t * (3 - 2 * t)  # smoothstep
                pose = current + t * (target - current)
                robot.set_joint_positions(np.deg2rad(pose))
                pb.stepSimulation()
                time.sleep(1 / 120)

                keys = pb.getKeyboardEvents()
                cam_dist, cam_yaw, cam_pitch = cam_control(
                    keys, cam_dist, cam_yaw, cam_pitch, cam_target
                )

            current = target.copy()

        # Pause between moves
        for _ in range(40):
            pb.stepSimulation()
            time.sleep(1 / 120)
            keys = pb.getKeyboardEvents()
            cam_dist, cam_yaw, cam_pitch = cam_control(
                keys, cam_dist, cam_yaw, cam_pitch, cam_target
            )

    print("\nDone. Close PyBullet window to exit.")
    try:
        while pb.isConnected():
            pb.stepSimulation()
            time.sleep(1 / 60)
            keys = pb.getKeyboardEvents()
            cam_dist, cam_yaw, cam_pitch = cam_control(
                keys, cam_dist, cam_yaw, cam_pitch, cam_target
            )
    except Exception:
        pass

    robot.disconnect()


if __name__ == "__main__":
    main()
