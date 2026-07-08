#!/usr/bin/env python3
import argparse
import re
import socket
import struct
import time
from pathlib import Path

CAN_FRAME_FMT = "=IB3x8s"

def load_home_yaml(path: Path):
    text = path.read_text()
    vals = {}
    for name in [
        "home_can10_0x141",
        "home_can10_0x142",
        "home_can10_0x143",
    ]:
        m = re.search(rf"{name}\s*:\s*([-+0-9.eE]+)", text)
        if not m:
            raise RuntimeError(f"missing {name} in {path}")
        vals[name] = float(m.group(1))
    return vals

def send_position(sock, motor_id: int, raw_deg: float, gear_ratio: float, max_speed: int):
    counts = int(round(raw_deg * gear_ratio / 0.01))

    data = bytearray(8)
    data[0] = 0xA4
    data[1] = 0x00
    data[2] = max_speed & 0xFF
    data[3] = (max_speed >> 8) & 0xFF
    data[4] = counts & 0xFF
    data[5] = (counts >> 8) & 0xFF
    data[6] = (counts >> 16) & 0xFF
    data[7] = (counts >> 24) & 0xFF

    frame = struct.pack(CAN_FRAME_FMT, motor_id, 8, bytes(data))
    sock.send(frame)
    print(f"send can10 0x{motor_id:X}: raw={raw_deg:.6f} deg counts={counts} speed={max_speed}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default="src/arm_control/config/real_home.yaml")
    ap.add_argument("--speed", type=int, default=10)
    ap.add_argument("--preload-j1-deg", type=float, default=0.8)
    ap.add_argument("--preload-j2-deg", type=float, default=0.8)
    ap.add_argument("--wait", type=float, default=1.2)
    args = ap.parse_args()

    yaml_path = Path(args.yaml)
    h = load_home_yaml(yaml_path)

    h_j1 = h["home_can10_0x141"]
    h_j2_main = h["home_can10_0x142"]
    h_j2_mirror = h["home_can10_0x143"]

    print("===== loaded home =====")
    print(f"JOINT1 can10 0x141 = {h_j1:.6f}")
    print(f"JOINT2 main   0x142 = {h_j2_main:.6f}")
    print(f"JOINT2 mirror 0x143 = {h_j2_mirror:.6f}")
    print("")

    print("===== preload pose =====")
    print("JOINT1 approaches home from negative side")
    print("JOINT2 approaches home with 0x142 increasing and 0x143 decreasing")
    pre_j1 = h_j1 - args.preload_j1_deg
    pre_j2_main = h_j2_main - args.preload_j2_deg
    pre_j2_mirror = h_j2_mirror + args.preload_j2_deg

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind(("can10",))

    try:
        # can10 feedback scale is 0.01/36 deg/count, so command gear_ratio is 36.
        gear = 36.0

        print("")
        print("STEP 1/2: move to preload near-home")
        send_position(sock, 0x141, pre_j1, gear, args.speed)
        time.sleep(0.03)
        send_position(sock, 0x142, pre_j2_main, gear, args.speed)
        time.sleep(0.03)
        send_position(sock, 0x143, pre_j2_mirror, gear, args.speed)
        time.sleep(args.wait)

        print("")
        print("STEP 2/2: move to exact YAML home")
        send_position(sock, 0x141, h_j1, gear, args.speed)
        time.sleep(0.03)
        send_position(sock, 0x142, h_j2_main, gear, args.speed)
        time.sleep(0.03)
        send_position(sock, 0x143, h_j2_mirror, gear, args.speed)
        time.sleep(args.wait)

    finally:
        sock.close()

    print("")
    print("done. Now check /joint_states and physical alignment.")

if __name__ == "__main__":
    main()
