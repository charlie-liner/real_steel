#!/usr/bin/env python3
"""Simplest possible test: left elbow angle → single servo."""

import math
import sys
import time

import cv2
import mediapipe as mp
import numpy as np
import serial

# ---- CONFIG ----
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-120"
BAUD = 115200

# ---- SERIAL ----
print(f"Connecting to {PORT} ...")
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(3)

# Drain boot messages
while ser.in_waiting:
    print(f"  ESP32: {ser.readline().decode(errors='ignore').strip()}")

# Test: move servo to 45 then back
print("Testing servo...")
ser.write(b"A:45.0\n")
print(f"  A:45 -> {ser.readline().decode(errors='ignore').strip()}")
time.sleep(1)
ser.write(b"A:0.0\n")
print(f"  A:0  -> {ser.readline().decode(errors='ignore').strip()}")
time.sleep(0.5)

# ---- CAMERA ----
print("Opening camera...")
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("ERROR: cannot open camera")
    ser.close()
    sys.exit(1)

# ---- MEDIAPIPE ----
print("Loading MediaPipe...")
from mediapipe.tasks.python import BaseOptions, vision

options = vision.PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="data/pose_landmarker_lite.task"),
    num_poses=1,
)
landmarker = vision.PoseLandmarker.create_from_options(options)

print(f"Ready! Bend your LEFT elbow. ESC to quit.")
print(f"{'—'*50}")

prev_angle = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Detect pose
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    result = landmarker.detect(mp_image)

    elbow_deg = None

    if result.pose_world_landmarks:
        wl = result.pose_world_landmarks[0]
        # 11=left_shoulder, 13=left_elbow, 15=left_wrist
        shoulder = np.array([wl[11].x, wl[11].y, wl[11].z])
        elbow = np.array([wl[13].x, wl[13].y, wl[13].z])
        wrist = np.array([wl[15].x, wl[15].y, wl[15].z])

        upper_arm = shoulder - elbow
        forearm = wrist - elbow

        ua_n = np.linalg.norm(upper_arm)
        fa_n = np.linalg.norm(forearm)

        if ua_n > 1e-6 and fa_n > 1e-6:
            dot = np.clip(np.dot(upper_arm / ua_n, forearm / fa_n), -1, 1)
            flexion = math.pi - math.acos(dot)
            elbow_deg = math.degrees(flexion)

            # Smooth
            elbow_deg = 0.3 * prev_angle + 0.7 * elbow_deg
            prev_angle = elbow_deg

            # Clamp to servo range (0-180)
            elbow_deg = max(0.0, min(180.0, elbow_deg))

            # Send to servo
            cmd = f"A:{elbow_deg:.1f}\n"
            ser.write(cmd.encode())
            if ser.in_waiting:
                ser.readline()

            print(f"  Elbow: {elbow_deg:6.1f}°  → servo", end="\r")

    # Show preview
    text = f"Elbow: {elbow_deg:.1f} deg" if elbow_deg is not None else "No pose"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.imshow("Elbow Tracking", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# Cleanup
print("\nDone.")
ser.write(b"E:0\n")
ser.close()
cap.release()
cv2.destroyAllWindows()
landmarker.close()
