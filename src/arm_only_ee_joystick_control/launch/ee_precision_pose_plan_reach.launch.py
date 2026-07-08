from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),

        Node(
            package="arm_only_ee_joystick_control",
            executable="ee_precision_pose_plan_node",
            name="ee_precision_pose_plan_node",
            output="screen",
            parameters=[{
                "use_sim_time": False,

                "planning_group": "arm",
                "ee_link": "gripper_tcp",
                "planner_id": "RRTConnectkConfigDefault",

                # reach mode에서는 preferred posture를 거의 무력화
                "preferred_joints": [
                    -0.0318,
                    -1.20,
                    -0.95,
                     0.0106,
                     0.0404,
                     3.2295
                ],

                "posture_weights": [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                ],

                "continuity_weights": [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
                ],

                # reach mode: 거리 우선
                "ik_position_tolerance": 0.030,
                "ik_max_iterations": 800,
                "ik_damping": 0.04,
                "ik_step_scale": 0.80,

                # 자세 유지/연속성 제거
                "nullspace_gain": 0.0,
                "continuity_gain": 0.0,

                # joint limit 회피는 아주 약하게만
                "joint_limit_gain": 0.003,
                "joint_limit_margin": 0.01,

                # 큰 관절 이동 허용
                "max_joint_step": 0.120,
                "max_goal_joint_delta": 3.14,

                # 큰 trajectory 허용
                "max_waypoint_joint_jump": 1.50,
                "max_total_joint_delta": 18.00,

                "planning_time": 10.0,
                "planning_attempts": 15,
                "velocity_scaling": 0.15,
                "accel_scaling": 0.15,

                # 테스트용 workspace 크게 개방
                "workspace_x_min": -1.50,
                "workspace_x_max":  1.50,
                "workspace_y_min": -1.50,
                "workspace_y_max":  1.50,
                "workspace_z_min": -1.00,
                "workspace_z_max":  1.50,

                # 안전 핵심값. 실제 실행 안 함.
                "execute_plan": False,

                # reach 테스트에서는 이전 plan-only 목표를 다음 기준으로 강하게 쓰지 않음
                "save_last_goal_on_plan_only": False,
            }],
        ),
    ])
