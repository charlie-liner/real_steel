"""Camera input module. Captures frames from webcam via OpenCV."""

import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Frame:
    image: np.ndarray  # BGR, shape (H, W, 3)
    timestamp: float  # time.time()
    frame_number: int


class Camera:
    def __init__(
        self, device_id: int = 0, width: int = 640, height: int = 480, fps: int = 30
    ):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.cap: cv2.VideoCapture | None = None
        self.frame_count = 0

    def open(self) -> bool:
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        return True

    def read(self) -> Frame | None:
        if self.cap is None:
            return None

        ret, image = self.cap.read()
        if not ret:
            return None

        self.frame_count += 1
        return Frame(
            image=image,
            timestamp=time.time(),
            frame_number=self.frame_count,
        )

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def is_opened(self) -> bool:
        return self.cap is not None and self.cap.isOpened()
