from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),

        Node(
            package="arm_only_ee_joystick_control",
            executable="ee_joy_plan_execute_node",
            name="ee_joy_plan_execute_node",
            output="screen",
            parameters=[{
                "use_sim_time": True,

                "planning_group": "arm",
                "ee_link": "LINK6",

                # 현재 preferred posture
                "preferred_joints": [
                    -0.031826,
                    -0.200239,
                    -0.295410,
                     0.010633,
                     0.040392,
                     3.329572
                ],

                # 자세 비용함수 가중치 W
                # 성공률 우선으로 약하게 시작
                "posture_weights": [
                     0.1,
                     0.5,
                     0.5,
                     0.3,
                     0.1,
                     0.1
                ],

                # Null-space IK 설정
                "ik_position_tolerance": 0.04,
                "ik_max_iterations": 30,
                "ik_damping": 0.10,
                "ik_step_scale": 0.60,
                "nullspace_gain": 0.01,
                "max_joint_step": 0.04,

                # 안정화 설정
                "joint_limit_margin": 0.08,
                "max_goal_joint_delta": 0.35,
                "enable_position_fallback": True,

                # 조이스틱 marker 이동
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

                "workspace_x_min": -0.60,
                "workspace_x_max": 0.60,
                "workspace_y_min": -0.60,
                "workspace_y_max": 0.60,
                "workspace_z_min": -0.20,
                "workspace_z_max": 0.80,
            }],
        ),
    ])
