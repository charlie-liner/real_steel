"""Camera verification script.

Standalone: python tests/test_camera.py  (opens GUI window, press 'q' to quit)
Pytest:     pytest tests/test_camera.py::test_camera_opens
"""

import cv2


def test_camera_opens():
    """Headless test: verify camera can be opened and one frame read."""
    cap = cv2.VideoCapture(0)
    assert cap.isOpened(), "Failed to open camera"
    ret, frame = cap.read()
    assert ret, "Failed to read frame"
    assert frame.shape[2] == 3, f"Expected 3 channels, got {frame.shape[2]}"
    assert frame.shape[0] > 0 and frame.shape[1] > 0, "Frame has zero dimensions"
    cap.release()


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    print("Camera opened. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Failed to read frame")
            break
        cv2.imshow("Camera Test", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Camera test complete.")


if __name__ == "__main__":
    main()
