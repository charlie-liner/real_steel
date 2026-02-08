"""Boxing move demo. Plays pre-programmed punch sequences to evaluate 3-DOF per arm + torso joint structure."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import time

import numpy as np
import pybullet as pb

from src.simulated_robot import SimulatedRobot

# Joint order: [l_tilt, l_pan, l_elbow, r_tilt, r_pan, r_elbow, torso_yaw] in DEGREES
#
# tilt:  0=neutral, +90=swing forward, -90=swing backward
# pan:   0=center, ±90=rotate around local vertical
# elbow: 0=straight, +90=bent 90deg
# torso_yaw: 0=facing forward, +ve=left shoulder forward

GUARD = [40, 0, 100,  40, 0, 100,  15]
REST  = [0, 0, 0,  0, 0, 0,  0]

# Each move: list of (keyframe_deg, transition_seconds)
MOVES = [
    ("Rest → Guard", [
        (REST, 0.1),
        (GUARD, 0.2),
    ]),
    ("Left Jab", [
        (GUARD, 0.1),
        ([70, 0, 5,  40, 0, 100,  20], 0.06),    # punch out, torso rotates into jab
        (GUARD, 0.1),
        ([70, 0, 5,  40, 0, 100,  20], 0.06),    # double jab
        (GUARD, 0.1),
    ]),
    ("Right Cross", [
        (GUARD, 0.1),
        ([40, 0, 100,  70, 0, 5,  -25], 0.07),   # punch out, torso rotates into cross
        (GUARD, 0.1),
    ]),
    ("Left Uppercut", [
        (GUARD, 0.1),
        ([-20, 0, 120,  40, 0, 100,  -10], 0.06),  # drop low, arm tucked, torso coils
        ([90, 0, 40,  40, 0, 100,  25], 0.05),      # explode upward, torso drives
        (GUARD, 0.1),
    ]),
    ("Right Uppercut", [
        (GUARD, 0.1),
        ([40, 0, 100,  -20, 0, 120,  10], 0.06),   # drop low, arm tucked, torso coils
        ([40, 0, 100,  90, 0, 40,  -25], 0.05),     # explode upward, torso drives
        (GUARD, 0.1),
    ]),
    ("Left Hook", [
        (GUARD, 0.1),
        ([10, -20, 90,  40, 0, 100,  -15], 0.07),   # wind up, torso coils back
        ([60, 50, 90,  40, 0, 100,  35], 0.06),      # sweep across, torso drives
        (GUARD, 0.1),
    ]),
    ("Right Hook", [
        (GUARD, 0.1),
        ([40, 0, 100,  10, 20, 90,  15], 0.07),     # wind up, torso coils back
        ([40, 0, 100,  60, -50, 90,  -35], 0.06),    # sweep across, torso drives
        (GUARD, 0.1),
    ]),
    ("Combo: Jab → Cross → Left Hook", [
        (GUARD, 0.1),
        ([70, 0, 5,  40, 0, 100,  20], 0.05),       # jab
        (GUARD, 0.07),
        ([40, 0, 100,  70, 0, 5,  -25], 0.05),      # cross
        (GUARD, 0.07),
        ([10, -20, 90,  40, 0, 100,  -15], 0.07),   # hook wind
        ([60, 50, 90,  40, 0, 100,  35], 0.06),      # hook sweep across
        (GUARD, 0.1),
    ]),
    ("Arms Forward (both punch)", [
        (GUARD, 0.1),
        ([70, 0, 5,  70, 0, 5,  0], 0.3),           # both arms forward
    ]),
    ("Guard → Rest", [
        ([70, 0, 5,  70, 0, 5,  0], 0.2),
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
    print("BOXING DEMO (3 DOF per arm + torso)")
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
