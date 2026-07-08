from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        # demo.launch.py / RViz FakeSystem 기준은 false가 안전함
        # Gazebo와 같이 쓸 때만 true로 바꿔도 됨
        SetParameter(name="use_sim_time", value=False),

        Node(
            package="arm_only_ee_joystick_control",
            executable="ee_joy_plan_execute_node",
            name="ee_joy_plan_execute_node",
            output="screen",
            parameters=[{
                "use_sim_time": False,

                # 새 gripper 포함 MoveIt config 기준
                "planning_group": "arm",
                "ee_link": "gripper_tcp",

                # null-space에서 유지하고 싶은 팔 모양.
                # home은 자연스러운 초기자세로 두고,
                # 작업 중 EE 제어는 forward_ready 자세를 q_ref로 사용.
                "preferred_joints": [
                    -0.0318,
                    -1.20,
                    -0.95,
                     0.0106,
                     0.0404,
                     3.2295
                ],

                # 자세 비용함수 가중치 W
                # JOINT2, JOINT3를 비교적 강하게 잡아서 팔꿈치가 뒤집히지 않게 함
                "posture_weights": [
                     0.1,
                     0.5,
                     0.5,
                     0.3,
                     0.1,
                     0.1
                ],

                # 빠른 null-space IK 설정
                "ik_position_tolerance": 0.02,
                "ik_max_iterations": 30,
                "ik_damping": 0.10,
                "ik_step_scale": 0.60,
                "nullspace_gain": 0.01,
                "max_joint_step": 0.04,

                # 안정화 설정
                "joint_limit_margin": 0.08,
                "max_goal_joint_delta": 0.35,
                "enable_position_fallback": True,

                # 조이스틱 marker 이동 속도
                "linear_speed": 0.05,
                "linear_speed_min": 0.01,
                "linear_speed_max": 0.20,
                "linear_speed_step": 0.01,
                "deadzone": 0.15,

                # MoveIt planning 설정
                "planning_time": 5.0,
                "planning_attempts": 3,
                "velocity_scaling": 0.50,
                "accel_scaling": 0.50,

                # gripper_tcp 기준 작업공간
                "workspace_x_min": -0.70,
                "workspace_x_max": 0.90,
                "workspace_y_min": -0.70,
                "workspace_y_max": 0.70,
                "workspace_z_min": -0.20,
                "workspace_z_max": 0.90,
            }],
        ),
    ])
