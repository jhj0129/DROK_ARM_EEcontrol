#!/usr/bin/env python3
import math
import socket
import struct
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from control_msgs.action import FollowJointTrajectory, GripperCommand


CAN_FRAME_FMT = "=IB3x8s"
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


class MoveItToRMDBridge(Node):
    """
    MoveIt FollowJointTrajectory -> RMD motor target bridge.

    Default mode is dry_run=True:
    - no CAN socket open
    - no CAN send
    - RViz Execute only prints converted motor targets
    """

    def __init__(self):
        super().__init__("moveit_to_rmd_bridge")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("default_max_speed", 5)
        self.declare_parameter("gripper_max_speed", 5)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.default_max_speed = int(self.get_parameter("default_max_speed").value)
        self.gripper_max_speed = int(self.get_parameter("gripper_max_speed").value)

        # MoveIt arm joints
        self.joint_order = ["JOINT1", "JOINT2", "JOINT3", "JOINT4", "JOINT5", "JOINT6"]

        # Shared real robot home parameters.
        # Pass src/arm_control/config/real_home.yaml with --ros-args --params-file.
        home = lambda name, default: float(self.declare_parameter(name, default).value)

        self.raw_home_deg = {
            ("can10", 0x141): home("home_can10_0x141", 4.6525),    # JOINT1
            ("can10", 0x142): home("home_can10_0x142", 33.330833), # JOINT2 main
            ("can10", 0x143): home("home_can10_0x143", -0.030000), # JOINT2 mirror
            ("can10", 0x144): home("home_can10_0x144", 21.615833), # JOINT3
            ("can11", 0x141): home("home_can11_0x141", 30.480000), # JOINT4
            ("can11", 0x142): home("home_can11_0x142", 0.380000),  # JOINT5
            ("can11", 0x143): home("home_can11_0x143", -20.668333333333333), # JOINT6
            ("can11", 0x144): home("home_can11_0x144", 12.150000), # JOINT7 feedback reference
        }

        # 현재 MoveIt Home 기준 [rad]
        self.q_home_rad = {
            "JOINT1": 0.0,
            "JOINT2": 0.385378389294,
            "JOINT3": -0.37726745641,
            "JOINT4": 0.0,
            "JOINT5": 0.0,
            "JOINT6": -0.78289622440,
            "JOINT7": -1.70000,
        }

        # RMD absolute position command counts = motor_deg * gear_ratio / 0.01
        # 실제 전송은 dry_run=false에서만 사용.
        self.gear_ratio = {
            ("can10", 0x141): 36.0,
            ("can10", 0x142): 36.0,
            ("can10", 0x143): 36.0,
            ("can10", 0x144): 36.0,
            ("can11", 0x141): 1.0,
            ("can11", 0x142): 1.0,
            ("can11", 0x143): 6.0,  # JOINT6: feedback uses 0.01/6 deg/count, so command must multiply by 6
            ("can11", 0x144): 1.0,
        }

        self.sockets = {}

        if self.dry_run:
            self.get_logger().warn("DRY RUN MODE: CAN sockets are NOT opened. No motor command will be sent.")
        else:
            self.get_logger().error("REAL SEND MODE ENABLED. Motors may move.")
            self._open_can_sockets(["can10", "can11"])

        self.arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.gripper_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
            execute_callback=self.execute_gripper_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("Action server ready: /arm_controller/follow_joint_trajectory")
        self.get_logger().info("Action server ready: /gripper_controller/gripper_cmd")

    def goal_cb(self, goal_request):
        if self.dry_run:
            self.get_logger().warn("Accepted goal in DRY RUN mode. This will not move the robot.")
        else:
            self.get_logger().error("Accepted goal in REAL SEND mode. Motors may move.")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested.")
        return CancelResponse.ACCEPT

    def _open_can_sockets(self, ifaces: List[str]):
        for iface in ifaces:
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((iface,))
            self.sockets[iface] = s
            self.get_logger().info(f"Opened CAN socket: {iface}")

    @staticmethod
    def _nearest_equivalent_angle(q: float, ref: float) -> float:
        while q - ref > math.pi:
            q -= 2.0 * math.pi
        while q - ref < -math.pi:
            q += 2.0 * math.pi
        return q

    def _q_to_raw_deg(self, q_by_joint: Dict[str, float], prev_q6_unwrapped=None):
        q1 = q_by_joint.get("JOINT1", self.q_home_rad["JOINT1"])
        q2 = q_by_joint.get("JOINT2", self.q_home_rad["JOINT2"])
        q3 = q_by_joint.get("JOINT3", self.q_home_rad["JOINT3"])
        q4 = q_by_joint.get("JOINT4", self.q_home_rad["JOINT4"])
        q5 = q_by_joint.get("JOINT5", self.q_home_rad["JOINT5"])
        q6 = q_by_joint.get("JOINT6", self.q_home_rad["JOINT6"])

        if prev_q6_unwrapped is None:
            q6_unwrapped = q6
        else:
            q6_unwrapped = self._nearest_equivalent_angle(q6, prev_q6_unwrapped)

        raw = {}

        # JOINT1
        raw[("can10", 0x141)] = self.raw_home_deg[("can10", 0x141)] + (
            q1 - self.q_home_rad["JOINT1"]
        ) * RAD2DEG

        # JOINT2 yo-yo dual motor structure
        delta2 = (q2 - self.q_home_rad["JOINT2"]) * RAD2DEG
        raw[("can10", 0x142)] = self.raw_home_deg[("can10", 0x142)] + delta2
        raw[("can10", 0x143)] = self.raw_home_deg[("can10", 0x143)] - delta2

        # JOINT3
        raw[("can10", 0x144)] = self.raw_home_deg[("can10", 0x144)] - (
            q3 - self.q_home_rad["JOINT3"]
        ) * RAD2DEG

        # JOINT4
        raw[("can11", 0x141)] = self.raw_home_deg[("can11", 0x141)] + (
            q4 - self.q_home_rad["JOINT4"]
        ) * RAD2DEG

        # JOINT5
        raw[("can11", 0x142)] = self.raw_home_deg[("can11", 0x142)] + (
            q5 - self.q_home_rad["JOINT5"]
        ) * RAD2DEG

        # JOINT6 continuous roll
        raw[("can11", 0x143)] = self.raw_home_deg[("can11", 0x143)] + (
            q6_unwrapped - self.q_home_rad["JOINT6"]
        ) * RAD2DEG

        return raw, q6_unwrapped

    def _raw_deg_to_counts(self, iface: str, motor_id: int, raw_deg: float) -> int:
        gear = self.gear_ratio[(iface, motor_id)]
        return int(round(raw_deg * gear / 0.01))

    def _send_rmd_position(self, iface: str, motor_id: int, raw_deg: float, max_speed: int):
        counts = self._raw_deg_to_counts(iface, motor_id, raw_deg)

        if self.dry_run:
            self.get_logger().info(
                f"[DRY] {iface} 0x{motor_id:X}: raw={raw_deg:.3f} deg, counts={counts}, speed={max_speed}"
            )
            return

        data = bytearray(8)
        data[0] = 0xA4
        data[1] = 0x00
        data[2] = max_speed & 0xFF
        data[3] = (max_speed >> 8) & 0xFF
        data[4] = counts & 0xFF
        data[5] = (counts >> 8) & 0xFF
        data[6] = (counts >> 16) & 0xFF
        data[7] = (counts >> 24) & 0xFF

        frame = struct.pack(CAN_FRAME_FMT, int(motor_id), 8, bytes(data))
        self.sockets[iface].send(frame)

    @staticmethod
    def _point_to_qdict(point, traj_joint_names: List[str], required_joints: List[str]):
        idx = {name: i for i, name in enumerate(traj_joint_names)}
        return {j: float(point.positions[idx[j]]) for j in required_joints}

    def _summarize_raw_trajectory(self, raw_list: List[Dict[Tuple[str, int], float]]):
        first = raw_list[0]
        last = raw_list[-1]

        print("\n================ BRIDGE TRAJECTORY SUMMARY ================")
        print(f"dry_run: {self.dry_run}")
        print(f"points: {len(raw_list)}")

        print("\n--- FIRST waypoint raw deg ---")
        for k, v in first.items():
            print(f"{k[0]} 0x{k[1]:X}: {v:.6f}")

        print("\n--- LAST waypoint raw deg ---")
        for k, v in last.items():
            print(f"{k[0]} 0x{k[1]:X}: {v:.6f}")

        print("\n--- TOTAL delta deg ---")
        for k in first:
            print(f"{k[0]} 0x{k[1]:X}: {last[k] - first[k]:+.6f}")

        d142 = last[("can10", 0x142)] - first[("can10", 0x142)]
        d143 = last[("can10", 0x143)] - first[("can10", 0x143)]
        print("\n--- JOINT2 mirror check ---")
        print(f"can10 0x142 delta: {d142:+.6f} deg")
        print(f"can10 0x143 delta: {d143:+.6f} deg")

        max_step = 0.0
        max_motor = None
        for a, b in zip(raw_list[:-1], raw_list[1:]):
            for k in a:
                d = abs(b[k] - a[k])
                if d > max_step:
                    max_step = d
                    max_motor = k

        print("\n--- max single waypoint step ---")
        if max_motor is not None:
            print(f"{max_motor[0]} 0x{max_motor[1]:X}: {max_step:.6f} deg")
        print("===========================================================\n")

    def execute_cb(self, goal_handle):
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory

        if not traj.joint_names or not traj.points:
            self.get_logger().error("Rejected empty trajectory.")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        missing = [j for j in self.joint_order if j not in traj.joint_names]
        if missing:
            self.get_logger().error(f"Rejected trajectory. Missing joints: {missing}, received={traj.joint_names}")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        q_points = [
            self._point_to_qdict(p, traj.joint_names, self.joint_order)
            for p in traj.points
        ]

        raw_points = []
        prev_q6 = None
        for q in q_points:
            raw, prev_q6 = self._q_to_raw_deg(q, prev_q6)
            raw_points.append(raw)

        self._summarize_raw_trajectory(raw_points)

        if self.dry_run:
            self.get_logger().warn("DRY RUN: trajectory accepted and summarized; no CAN command sent.")
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "dry_run success"
            return result

        # Real send mode. Not used yet.
        start_time = time.monotonic()
        for point, raw in zip(traj.points, raw_points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "cancelled"
                return result

            target_t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            wait = start_time + target_t - time.monotonic()
            if wait > 0:
                time.sleep(wait)

            for (iface, mid), deg in raw.items():
                self._send_rmd_position(iface, mid, deg, self.default_max_speed)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "success"
        return result

    def execute_gripper_cb(self, goal_handle):
        result = GripperCommand.Result()
        q = float(goal_handle.request.command.position)

        q = max(-1.70000, min(45.00000, q))
        delta = (q - self.q_home_rad["JOINT7"]) * RAD2DEG

        # JOINT7: measured close direction is raw increase.
        raw_deg = self.raw_home_deg[("can11", 0x144)] + delta

        print("\n================ GRIPPER BRIDGE SUMMARY ================")
        print(f"dry_run: {self.dry_run}")
        print(f"JOINT7 q: {q:.6f} rad")
        print(f"can11 0x144 raw target: {raw_deg:.6f} deg")
        print("========================================================\n")

        self._send_rmd_position("can11", 0x144, raw_deg, self.gripper_max_speed)

        goal_handle.succeed()
        result.position = q
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result


def main():
    rclpy.init()
    node = MoveItToRMDBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
