"""PyBullet verification script.

Standalone: python tests/test_pybullet.py  (opens GUI window, runs 5s simulation)
Pytest:     pytest tests/test_pybullet.py::test_pybullet_loads
"""

import pybullet as p
import pybullet_data
import time


def test_pybullet_loads():
    """Headless test: verify PyBullet can load a URDF in DIRECT mode."""
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
    num_joints = p.getNumJoints(robot)
    assert num_joints > 0, f"Expected joints > 0, got {num_joints}"

    # Step a few times to verify simulation runs
    for _ in range(100):
        p.stepSimulation()

    p.disconnect()


def main():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
    num_joints = p.getNumJoints(robot)
    print(f"Loaded robot with {num_joints} joints")

    print("Running simulation for 5 seconds...")
    for _ in range(240 * 5):
        p.stepSimulation()
        time.sleep(1 / 240)

    p.disconnect()
    print("PyBullet test complete.")


if __name__ == "__main__":
    main()
