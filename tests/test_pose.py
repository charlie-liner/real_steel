"""MediaPipe pose verification script.

Standalone: python tests/test_pose.py  (opens GUI window with pose overlay, press 'q' to quit)
Pytest:     pytest tests/test_pose.py::test_mediapipe_imports
"""

import os

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

MODEL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "data",
    "pose_landmarker_lite.task",
)


def test_mediapipe_imports():
    """Headless test: verify MediaPipe pose landmarker can be instantiated."""
    assert os.path.exists(MODEL_PATH), f"Model not found at {MODEL_PATH}"

    options = vision.PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        num_poses=1,
    )
    landmarker = vision.PoseLandmarker.create_from_options(options)
    assert landmarker is not None
    landmarker.close()


def main():
    if not os.path.exists(MODEL_PATH):
        print(f"ERROR: Pose model not found at {MODEL_PATH}")
        print("Download it with:")
        print(
            '  curl -L -o data/pose_landmarker_lite.task "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"'
        )
        return

    options = vision.PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        num_poses=1,
    )
    landmarker = vision.PoseLandmarker.create_from_options(options)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    print("Pose estimation running. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Failed to read frame")
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        result = landmarker.detect(mp_image)

        if result.pose_landmarks:
            for landmarks in result.pose_landmarks:
                # Draw landmarks manually since drawing API changed
                h, w = frame.shape[:2]
                for lm in landmarks:
                    x, y = int(lm.x * w), int(lm.y * h)
                    cv2.circle(frame, (x, y), 4, (0, 255, 0), -1)

        cv2.imshow("Pose Test", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    landmarker.close()
    cap.release()
    cv2.destroyAllWindows()
    print("Pose test complete.")


if __name__ == "__main__":
    main()
