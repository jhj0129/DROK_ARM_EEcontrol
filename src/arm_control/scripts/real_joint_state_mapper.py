#!/usr/bin/env python3
import math
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RealJointStateMapper(Node):
    """Map old /motor_angles/* degree topics to MoveIt-compatible /joint_states.

    Run joystick_node_92 with /joint_states remapped away, then run this node as the
    only publisher of MoveIt joint states.
    """

    def __init__(self):
        super().__init__("real_joint_state_mapper")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("joint_names", ["JOINT1", "JOINT2", "JOINT3", "JOINT4", "JOINT5", "JOINT6", "JOINT7"])
        self.declare_parameter("motor_topics", [
            "/motor_angles/can10_motor_0x141",
            "/motor_angles/can10_motor_0x142",
            "/motor_angles/can10_motor_0x144",
            "/motor_angles/can11_motor_0x141",
            "/motor_angles/can11_motor_0x142",
            "/motor_angles/can11_motor_0x143",
            "",  # JOINT7 gripper: leave 0 until real gripper CAN motor is assigned
        ])
        self.declare_parameter("joint_signs", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("joint_offsets_rad", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.motor_topics = list(self.get_parameter("motor_topics").value)
        self.joint_signs = [float(x) for x in self.get_parameter("joint_signs").value]
        self.joint_offsets_rad = [float(x) for x in self.get_parameter("joint_offsets_rad").value]
        hz = float(self.get_parameter("publish_hz").value)

        self.q: Dict[str, float] = {j: 0.0 for j in self.joint_names}
        for i, topic in enumerate(self.motor_topics):
            if not topic:
                continue
            joint_name = self.joint_names[i]
            sign = self.joint_signs[i]
            offset = self.joint_offsets_rad[i]
            self.create_subscription(
                Float64,
                topic,
                lambda msg, j=joint_name, s=sign, o=offset: self._angle_cb(msg, j, s, o),
                10,
            )
            self.get_logger().info(f"Mapping {topic} -> {joint_name}")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / max(hz, 1.0), self._publish)

    def _angle_cb(self, msg: Float64, joint_name: str, sign: float, offset: float):
        self.q[joint_name] = sign * (msg.data * math.pi / 180.0) + offset

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [self.q[j] for j in self.joint_names]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RealJointStateMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
