#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

# 손으로 맞춘 실제 Home raw angle
RAW_HOME_DEG = {
    "can10_0x141": -4.503333,     # JOINT1
    "can10_0x142": 33.330833,     # JOINT2 main
    "can10_0x143": -0.030000,     # JOINT2 mirror
    "can10_0x144": 21.615833,     # JOINT3
    "can11_0x141": 30.480000,     # JOINT4
    "can11_0x142": 0.380000,      # JOINT5
    "can11_0x143": 35.136667,     # JOINT6
}

# 현재 MoveIt Home 기준
Q_HOME_RAD = {
    "JOINT1": 0.0,
    "JOINT2": 0.58173277828,
    "JOINT3": -0.37726745641,
    "JOINT4": 0.0,
    "JOINT5": 0.0,
    "JOINT6": -0.78539816339,
}

def nearest_equivalent_angle(q, ref):
    """continuous joint용: ref에 가장 가까운 등가각 선택"""
    while q - ref > math.pi:
        q -= 2.0 * math.pi
    while q - ref < -math.pi:
        q += 2.0 * math.pi
    return q

def motor_targets_from_q(q_map, prev_q6_unwrapped=None):
    q1 = q_map.get("JOINT1", Q_HOME_RAD["JOINT1"])
    q2 = q_map.get("JOINT2", Q_HOME_RAD["JOINT2"])
    q3 = q_map.get("JOINT3", Q_HOME_RAD["JOINT3"])
    q4 = q_map.get("JOINT4", Q_HOME_RAD["JOINT4"])
    q5 = q_map.get("JOINT5", Q_HOME_RAD["JOINT5"])
    q6 = q_map.get("JOINT6", Q_HOME_RAD["JOINT6"])

    if prev_q6_unwrapped is None:
        q6_unwrapped = q6
    else:
        q6_unwrapped = nearest_equivalent_angle(q6, prev_q6_unwrapped)

    raw = {}

    # JOINT1
    raw["can10 0x141"] = RAW_HOME_DEG["can10_0x141"] + (q1 - Q_HOME_RAD["JOINT1"]) * RAD2DEG

    # JOINT2: 요요 구조
    # 0x142는 정방향, 0x143은 반대방향
    delta2 = (q2 - Q_HOME_RAD["JOINT2"]) * RAD2DEG
    raw["can10 0x142"] = RAW_HOME_DEG["can10_0x142"] + delta2
    raw["can10 0x143"] = RAW_HOME_DEG["can10_0x143"] - delta2

    # JOINT3
    raw["can10 0x144"] = RAW_HOME_DEG["can10_0x144"] - (q3 - Q_HOME_RAD["JOINT3"]) * RAD2DEG

    # JOINT4
    raw["can11 0x141"] = RAW_HOME_DEG["can11_0x141"] + (q4 - Q_HOME_RAD["JOINT4"]) * RAD2DEG

    # JOINT5
    raw["can11 0x142"] = RAW_HOME_DEG["can11_0x142"] + (q5 - Q_HOME_RAD["JOINT5"]) * RAD2DEG

    # JOINT6 continuous roll
    raw["can11 0x143"] = RAW_HOME_DEG["can11_0x143"] + (q6_unwrapped - Q_HOME_RAD["JOINT6"]) * RAD2DEG

    return raw, q6_unwrapped

class PlanDryRun(Node):
    def __init__(self):
        super().__init__("moveit_plan_to_rmd_dry_run")
        self.sub = self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self.cb,
            10
        )
        self.get_logger().info("Dry-run ready. RViz에서 Plan만 누르면 motor raw 변환값을 출력합니다.")

    def cb(self, msg):
        if not msg.trajectory:
            self.get_logger().warn("DisplayTrajectory has no trajectory.")
            return

        traj = msg.trajectory[0].joint_trajectory
        names = list(traj.joint_names)
        points = traj.points

        if not points:
            self.get_logger().warn("Trajectory has no points.")
            return

        print("\n\n================ MOVEIT PLAN -> RMD DRY RUN ================")
        print(f"joint_names: {names}")
        print(f"points: {len(points)}")
        print("실제 CAN 명령은 보내지 않음. 출력만 함.")

        raw_list = []
        prev_q6 = None

        for p in points:
            q_map = {name: p.positions[i] for i, name in enumerate(names)}
            raw, prev_q6 = motor_targets_from_q(q_map, prev_q6)
            raw_list.append(raw)

        first = raw_list[0]
        last = raw_list[-1]

        print("\n--- FIRST waypoint raw deg ---")
        for k, v in first.items():
            print(f"{k}: {v:.6f}")

        print("\n--- LAST waypoint raw deg ---")
        for k, v in last.items():
            print(f"{k}: {v:.6f}")

        print("\n--- TOTAL delta deg ---")
        for k in first:
            print(f"{k}: {last[k] - first[k]:+.6f}")

        print("\n--- JOINT2 mirror check ---")
        d142 = last["can10 0x142"] - first["can10 0x142"]
        d143 = last["can10 0x143"] - first["can10 0x143"]
        print(f"can10 0x142 delta: {d142:+.6f} deg")
        print(f"can10 0x143 delta: {d143:+.6f} deg")
        print("정상이라면 두 값은 부호가 반대여야 함.")

        max_step = 0.0
        max_motor = None
        for a, b in zip(raw_list[:-1], raw_list[1:]):
            for k in a:
                d = abs(b[k] - a[k])
                if d > max_step:
                    max_step = d
                    max_motor = k

        print("\n--- max single waypoint step ---")
        print(f"{max_motor}: {max_step:.6f} deg")

        if max_step > 10.0:
            print("주의: waypoint 사이 raw 변화가 10도보다 큼. 속도/보간 확인 필요.")
        else:
            print("OK: waypoint 간 변화가 비교적 작음.")

        print("============================================================\n")

def main():
    rclpy.init()
    node = PlanDryRun()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
