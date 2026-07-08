#!/usr/bin/env python3
import argparse
import re
import statistics
import subprocess
import time
from pathlib import Path

TOPICS = {
    "home_can10_0x141": "/motor_angles/can10_motor_0x141",  # JOINT1
    "home_can10_0x142": "/motor_angles/can10_motor_0x142",  # JOINT2 main
    "home_can10_0x143": "/motor_angles/can10_motor_0x143",  # JOINT2 mirror
    "home_can10_0x144": "/motor_angles/can10_motor_0x144",  # JOINT3
    "home_can11_0x141": "/motor_angles/can11_motor_0x141",  # JOINT4
    "home_can11_0x142": "/motor_angles/can11_motor_0x142",  # JOINT5
    "home_can11_0x143": "/motor_angles/can11_motor_0x143",  # JOINT6
}

def read_once(topic: str) -> float:
    out = subprocess.check_output(
        ["ros2", "topic", "echo", topic, "--once"],
        text=True,
        timeout=5.0,
    )
    m = re.search(r"data:\s*([-+0-9.eE]+)", out)
    if not m:
        raise RuntimeError(f"failed to parse {topic}:\n{out}")
    return float(m.group(1))

def read_median(topic: str, samples: int, delay: float):
    vals = []
    for _ in range(samples):
        vals.append(read_once(topic))
        time.sleep(delay)
    med = statistics.median(vals)
    spread = max(vals) - min(vals)
    return med, spread, vals

def patch_yaml(text: str, key: str, value: float) -> str:
    pat = rf"{key}\s*:\s*[-+0-9.eE]+"
    rep = f"{key}: {value:.15f}"
    text2, n = re.subn(pat, rep, text)
    if n != 1:
        raise RuntimeError(f"failed to patch {key}, replacements={n}")
    return text2

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="src/arm_control/config/real_home.yaml")
    ap.add_argument("--out", default="/tmp/drok_session_home.yaml")
    ap.add_argument("--samples", type=int, default=5)
    ap.add_argument("--delay", type=float, default=0.08)
    ap.add_argument("--warn-spread-deg", type=float, default=0.05)
    args = ap.parse_args()

    base = Path(args.base)
    out = Path(args.out)

    if not base.exists():
        raise SystemExit(f"base yaml not found: {base}")

    text = base.read_text()

    print("===== CAPTURE ARM SESSION HOME =====")
    print("전제: 실제 로봇이 물리적으로 home 자세에 있어야 함.")
    print("이 값은 이번 부팅 세션에서만 사용: /tmp/drok_session_home.yaml")
    print("")

    captured = {}

    for key, topic in TOPICS.items():
        med, spread, vals = read_median(topic, args.samples, args.delay)
        captured[key] = med
        text = patch_yaml(text, key, med)

        status = "OK"
        if spread > args.warn_spread_deg:
            status = "WARN_UNSTABLE"

        print(f"{key:18s} {med: .15f} deg  spread={spread:.6f}  {status}")
        print(f"  topic: {topic}")
        print(f"  vals : " + ", ".join(f"{v:.6f}" for v in vals))

    out.write_text(text)

    print("")
    print(f"saved: {out}")
    print("")
    print("이제 joint reader와 bridge는 반드시 이 파일로 실행:")
    print(f"  --params-file {out}")

if __name__ == "__main__":
    main()
