"""Interactive joint mapping verification.

Moves one logical joint at a time between two positions.
User checks which physical servo responds and whether the direction is correct.

Usage:
    python tests/test_joint_mapping.py --port /dev/cu.usbserial-120
"""

import argparse
import sys
import time

import serial

JOINTS = [
    {"idx": 0, "name": "L_shoulder_roll", "desc": "Left arm spread outward",    "a": 0, "b": 45},
    {"idx": 1, "name": "L_shoulder_tilt", "desc": "Left arm swing forward",     "a": 0, "b": 45},
    {"idx": 2, "name": "L_shoulder_pan",  "desc": "Left arm rotate horizontal", "a": 0, "b": 45},
    {"idx": 3, "name": "L_elbow",         "desc": "Left elbow bend",            "a": 0, "b": 45},
    {"idx": 4, "name": "R_shoulder_roll", "desc": "Right arm spread outward",   "a": 0, "b": 45},
    {"idx": 5, "name": "R_shoulder_tilt", "desc": "Right arm swing forward",    "a": 0, "b": 45},
    {"idx": 6, "name": "R_shoulder_pan",  "desc": "Right arm rotate horizontal","a": 0, "b": 45},
    {"idx": 7, "name": "R_elbow",         "desc": "Right elbow bend",           "a": 0, "b": 45},
]


def connect(port, baud=115200):
    ser = serial.Serial(port, baud, timeout=1)
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(0.1)
    ser.reset_input_buffer()

    deadline = time.time() + 15
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if "READY" in line:
            print("Connected.\n")
            return ser
    raise RuntimeError("No READY from ESP32")


def send(ser, cmd):
    ser.write(f"{cmd}\n".encode())
    return ser.readline().decode(errors="replace").strip()


def main():
    parser = argparse.ArgumentParser(description="Joint mapping verification")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = connect(args.port, args.baud)

    send(ser, "E:0")
    send(ser, "H")
    send(ser, "E:1")
    time.sleep(0.5)

    print("=" * 50)
    print("JOINT MAPPING TEST")
    print("=" * 50)
    print()
    print("Each joint will toggle between 0° and 45° repeatedly.")
    print("Watch which servo moves and check if it matches.")
    print()
    print("Controls:")
    print("  Enter = next joint")
    print("  r     = repeat (toggle again)")
    print("  q     = quit")
    print()

    for joint in JOINTS:
        idx = joint["idx"]
        send(ser, "E:1")  # re-enable in case watchdog fired
        send(ser, "H")
        time.sleep(0.3)

        print(f"--- Joint {idx}: {joint['name']} ---")
        print(f"    Expected: {joint['desc']}")

        while True:
            # Toggle: 0 → 45 → 0 → 45 → 0
            for angle in [joint["b"], joint["a"], joint["b"], joint["a"]]:
                send(ser, f"S:{idx}:{angle:.1f}")
                time.sleep(0.4)

            resp = input("    [Enter=next, r=repeat, q=quit] ").strip().lower()
            if resp == "q":
                print("Quitting.")
                send(ser, "H")
                send(ser, "E:0")
                ser.close()
                return
            elif resp == "r":
                send(ser, "E:1")  # re-enable
                continue
            else:
                break

    print("\nAll joints tested.")
    send(ser, "H")
    time.sleep(1)
    send(ser, "E:0")
    ser.close()


if __name__ == "__main__":
    main()
