#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperPlateMimic(Node):
    def __init__(self):
        super().__init__("gripper_plate_mimic_node")

        self.joint7_name = "JOINT7"
        self.left_name = "LEFT_PLATE_JOINT"
        self.right_name = "RIGHT_PLATE_JOINT"

        self.multiplier = 0.0166667
        self.max_plate = 0.07
        self.last_cmd = None

        self.pub = self.create_publisher(
            JointTrajectory,
            "/plate_controller/joint_trajectory",
            10,
        )

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_states,
            10,
        )

        self.get_logger().info("gripper_plate_mimic_node started")

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def on_joint_states(self, msg):
        if self.joint7_name not in msg.name:
            return

        idx = msg.name.index(self.joint7_name)
        q7 = msg.position[idx]

        plate = self.clamp(q7 * self.multiplier, 0.0, self.max_plate)

        # 양쪽 plate 축 방향이 반대로 잡혀 있으면 같은 +값으로 닫히는 경우가 많다.
        # 만약 한쪽만 반대로 움직이면 아래 right 값을 -plate로 바꾸면 된다.
        left = plate
        right = plate

        # 너무 자주 같은 명령 보내지 않기
        key = (round(left, 5), round(right, 5))
        if key == self.last_cmd:
            return
        self.last_cmd = key

        traj = JointTrajectory()
        traj.joint_names = [self.left_name, self.right_name]

        pt = JointTrajectoryPoint()
        pt.positions = [left, right]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 150_000_000

        traj.points.append(pt)
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = GripperPlateMimic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
