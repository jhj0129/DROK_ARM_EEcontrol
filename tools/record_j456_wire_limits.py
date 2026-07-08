#!/usr/bin/env python3
import argparse
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


TOPICS = {
    "JOINT4": "/motor_angles/can11_motor_0x141",
    "JOINT5": "/motor_angles/can11_motor_0x142",
    "JOINT6": "/motor_angles/can11_motor_0x143",
}


class Recorder(Node):
    def __init__(self, joint_name, out_dir):
        super().__init__("j456_wire_limit_recorder")
        self.joint_name = joint_name
        self.topic = TOPICS[joint_name]
        self.current = None
        self.min_v = None
        self.max_v = None
        self.samples = []

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(out_dir, f"{joint_name}_wire_limit_{ts}.csv")
        self.summary_path = os.path.join(out_dir, f"{joint_name}_wire_limit_{ts}.txt")

        self.sub = self.create_subscription(Float64, self.topic, self.cb, 10)
        self.timer = self.create_timer(0.25, self.print_status)

        print("")
        print("===== WIRE LIMIT RECORDING START =====")
        print(f"joint: {self.joint_name}")
        print(f"topic: {self.topic}")
        print(f"csv:   {self.csv_path}")
        print("")
        print("손으로 해당 조인트만 천천히 움직여.")
        print("배선이 당겨지기 직전/꼬이기 직전에서 멈추고, 반대쪽도 동일하게 측정.")
        print("끝나면 Ctrl+C.")
        print("======================================")
        print("")

    def cb(self, msg):
        v = float(msg.data)
        now = time.time()
        self.current = v
        self.min_v = v if self.min_v is None else min(self.min_v, v)
        self.max_v = v if self.max_v is None else max(self.max_v, v)
        self.samples.append((now, v))

    def print_status(self):
        if self.current is None:
            print(f"[{self.joint_name}] waiting for {self.topic} ...")
            return
        rng = self.max_v - self.min_v
        print(
            f"[{self.joint_name}] current={self.current:+.6f} deg | "
            f"min={self.min_v:+.6f} | max={self.max_v:+.6f} | range={rng:.6f}",
            flush=True,
        )

    def save(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        with open(self.csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_sec", "raw_deg"])
            w.writerows(self.samples)

        with open(self.summary_path, "w") as f:
            f.write("===== WIRE LIMIT SUMMARY =====\n")
            f.write(f"joint: {self.joint_name}\n")
            f.write(f"topic: {self.topic}\n")
            f.write(f"samples: {len(self.samples)}\n")
            if self.current is not None:
                f.write(f"min_raw_deg: {self.min_v:.12f}\n")
                f.write(f"max_raw_deg: {self.max_v:.12f}\n")
                f.write(f"range_deg: {(self.max_v - self.min_v):.12f}\n")
                f.write("\n")
                f.write("Recommended conservative limit:\n")
                f.write(f"lower_raw_deg = min_raw_deg + 3.0 deg margin\n")
                f.write(f"upper_raw_deg = max_raw_deg - 3.0 deg margin\n")
            else:
                f.write("No data received.\n")

        print("")
        print("===== SAVED =====")
        print(self.csv_path)
        print(self.summary_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint", required=True, choices=TOPICS.keys())
    parser.add_argument("--out-dir", default=os.path.expanduser("~/DROK_ARM_EEcontrol/safety_measurements"))
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args.joint, args.out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
