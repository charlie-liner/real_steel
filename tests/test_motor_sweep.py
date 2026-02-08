"""Sweep each motor one at a time to verify hardware wiring.

Usage:
    python tests/test_motor_sweep.py --port /dev/tty.usbserial-XXXX
    python tests/test_motor_sweep.py --port /dev/tty.usbserial-XXXX --channels 1,2,3,5,6,7

Each motor sweeps: home → +30° → home → -30° → home (where limits allow).
Press Enter to advance to the next motor, or 'q' to quit.
"""

import argparse
import sys
import time

import serial


JOINT_LABELS = [
    "0: L_shoulder_roll",
    "1: L_shoulder_tilt",
    "2: L_shoulder_pan",
    "3: L_elbow",
    "4: R_shoulder_roll",
    "5: R_shoulder_tilt",
    "6: R_shoulder_pan",
    "7: R_elbow",
]

# Angle limits (degrees) — must match config.h
ANGLE_MIN = [-20, -90, -90, 0, -20, -90, -90, 0]
ANGLE_MAX = [135, 90, 90, 135, 135, 90, 90, 135]


def connect(port: str, baud: int = 115200) -> serial.Serial:
    print(f"Connecting to {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)

    # Toggle DTR to reset ESP32 (same as Arduino Serial Monitor does)
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(0.1)
    ser.reset_input_buffer()

    deadline = time.time() + 15
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"  ESP32: {line}")
        if "READY" in line:
            print("Connected.\n")
            return ser

    raise RuntimeError("Did not receive READY from ESP32 within 15s")


def send(ser: serial.Serial, cmd: str) -> str:
    ser.write(f"{cmd}\n".encode())
    resp = ser.readline().decode(errors="replace").strip()
    return resp


def sweep_motor(ser: serial.Serial, channel: int):
    """Sweep one motor: home → +30 → home → -30 → home."""
    label = JOINT_LABELS[channel] if channel < len(JOINT_LABELS) else f"{channel}: ???"
    lo, hi = ANGLE_MIN[channel], ANGLE_MAX[channel]

    # Clamp sweep range to joint limits
    pos_target = min(30.0, hi)
    neg_target = max(-30.0, lo)

    print(f"  [{label}]  range [{lo}, {hi}]")
    print(f"  Sweeping: 0 → {pos_target:+.0f} → 0 → {neg_target:+.0f} → 0")

    steps = [0.0, pos_target, 0.0, neg_target, 0.0]
    for angle in steps:
        resp = send(ser, f"S:{channel}:{angle:.1f}")
        print(f"    S:{channel}:{angle:.1f} -> {resp}")
        if "ERR" in resp:
            return False
        time.sleep(0.5)

    print(f"  OK")
    return True


def sweep_all(ser: serial.Serial):
    """Sweep all 8 motors simultaneously using J: command."""
    pos_targets = [min(30.0, hi) for hi in ANGLE_MAX]
    neg_targets = [max(-30.0, lo) for lo in ANGLE_MIN]
    zeros = [0.0] * 8

    steps = [
        ("Home", zeros),
        ("+30", pos_targets),
        ("Home", zeros),
        ("-30", neg_targets),
        ("Home", zeros),
    ]

    for label, angles in steps:
        angles_str = ",".join(f"{a:.1f}" for a in angles)
        resp = send(ser, f"J:{angles_str}")
        print(f"  {label:>5}: J:{angles_str} -> {resp}")
        time.sleep(1.0)

    print("  Done.")


def main():
    parser = argparse.ArgumentParser(description="Motor sweep test")
    parser.add_argument("--port", required=True, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--channels",
        default=None,
        help="Comma-separated channel indices to test (default: all 0-7)",
    )
    parser.add_argument(
        "--all-at-once",
        action="store_true",
        help="Sweep all motors simultaneously using J: command",
    )
    args = parser.parse_args()

    if args.channels:
        channels = [int(c) for c in args.channels.split(",")]
    else:
        channels = list(range(8))

    ser = connect(args.port, args.baud)

    try:
        # Safe startup
        print("Enabling servos...")
        print(f"  E:0 -> {send(ser, 'E:0')}")
        print(f"  H   -> {send(ser, 'H')}")
        print(f"  E:1 -> {send(ser, 'E:1')}")
        time.sleep(0.5)

        if args.all_at_once:
            print("\nSweeping ALL motors simultaneously...\n")
            sweep_all(ser)
            print("\nAll done.")
        else:
            print(f"\nTesting {len(channels)} motors. Press Enter after each, 'q' to quit.\n")

        for ch in ([] if args.all_at_once else channels):
            # Re-enable servos (watchdog may have disabled them during user input)
            send(ser, "E:1")
            time.sleep(0.1)

            print(f"--- Motor {ch} ---")
            sweep_motor(ser, ch)

            try:
                resp = input("  Press Enter for next motor (q to quit): ")
            except (EOFError, KeyboardInterrupt):
                break
            if resp.strip().lower() == "q":
                break

        print("\nAll done.")

    finally:
        print("Homing and disabling...")
        send(ser, "H")
        time.sleep(1)
        send(ser, "E:0")
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
