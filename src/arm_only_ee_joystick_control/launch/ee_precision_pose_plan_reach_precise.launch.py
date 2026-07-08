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

                # reach_precise:
                # reach mode의 넓은 작업영역은 유지하되,
                # IK 수렴 정확도만 올리는 모드
                "preferred_joints": [
                    -0.0318,
                    -1.20,
                    -0.95,
                     0.0106,
                     0.0404,
                     3.2295
                ],

                # 자세 유지 항목은 거의 제거
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

                # reach mode보다 정확도 강화
                # 기존 reach: 0.030
                # 이번: 0.010
                "ik_position_tolerance": 0.010,
                "ik_max_iterations": 1600,

                # damping을 낮추면 더 정확히 붙을 수 있지만 불안정할 수 있음
                # 0.035 정도가 중간값
                "ik_damping": 0.035,

                # step을 줄여서 마지막 수렴을 세밀하게
                "ik_step_scale": 0.35,

                # 자세/연속성 제한은 꺼둠
                "nullspace_gain": 0.0,
                "continuity_gain": 0.0,

                # joint limit 회피는 아주 약하게만 유지
                "joint_limit_gain": 0.002,
                "joint_limit_margin": 0.005,

                # 관절 이동량은 크게 허용해서 작업영역을 줄이지 않음
                "max_joint_step": 0.060,
                "max_goal_joint_delta": 3.14,

                # trajectory 검증은 reach보다 약간만 강화
                "max_waypoint_joint_jump": 1.20,
                "max_total_joint_delta": 18.00,

                "planning_time": 12.0,
                "planning_attempts": 20,
                "velocity_scaling": 0.12,
                "accel_scaling": 0.12,

                # reach mode처럼 넓게 유지
                "workspace_x_min": -1.50,
                "workspace_x_max":  1.50,
                "workspace_y_min": -1.50,
                "workspace_y_max":  1.50,
                "workspace_z_min": -1.00,
                "workspace_z_max":  1.50,

                # 안전 핵심값. 실제 실행 안 함.
                "execute_plan": False,

                # 넓은 workspace에서는 이전 q_goal에 끌려가지 않게 false
                "save_last_goal_on_plan_only": False,
            }],
        ),
    ])
