from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),

        Node(
            package="arm_only_ee_joystick_control",
            executable="ee_obstacle_waypoint_plan_node",
            name="ee_obstacle_waypoint_plan_node",
            output="screen",
            parameters=[{
                "use_sim_time": False,

                "planning_group": "arm",
                "ee_link": "gripper_tcp",
                "planner_id": "RRTConnectkConfigDefault",

                # 안전 기본값
                "execute_plan": False,

                # 기존 reach_precise 계열 위치 IK 설정
                "ik_position_tolerance": 0.010,
                "q6_plan_error_tolerance": 0.025,
                "ik_max_iterations": 1600,
                "ik_damping": 0.035,
                "ik_step_scale": 0.35,
                "max_joint_step": 0.060,

                "planning_time": 12.0,
                "planning_attempts": 20,
                "velocity_scaling": 0.12,
                "accel_scaling": 0.12,

                "max_waypoint_joint_jump": 1.20,

                "workspace_x_min": -1.50,
                "workspace_x_max":  1.50,
                "workspace_y_min": -1.50,
                "workspace_y_max":  1.50,
                "workspace_z_min": -1.00,
                "workspace_z_max":  1.50,

                # 직사각형 장애물 box
                # frame = ARM_BASE_LINK 기준
                #
                # obstacle_center:
                #   장애물 중심 좌표
                #
                # obstacle_size:
                #   실제 장애물 크기 [x길이, y폭, z높이]
                #
                # 예시:
                # x=0.55 앞쪽에 있고,
                # y방향 폭 0.60m,
                # z높이 0.50m인 직사각형 장애물
                "obstacle_enabled": False,
                "obstacle_id": "front_box_obstacle",
                "obstacle_center": [0.55, 0.0, 0.20],
                "obstacle_size": [0.22, 0.60, 0.40],

                # 장애물 collision box를 실제보다 크게 잡는 안전 마진
                "obstacle_margin": 0.04,

                # 장애물 상단보다 얼마나 더 높게 지나갈지
                "obstacle_clearance": 0.06,

                # 일단 항상 장애물 회피 waypoint를 쓰도록 설정
                # 나중에는 직선 경로가 장애물과 겹칠 때만 쓰게 바꿀 수 있음
                # q6 roll candidate search:
                # gripper_base가 긴 막대기라서 박스에 걸릴 때,
                # JOINT6를 여러 각도로 돌려 collision-free plan을 찾는다.
                "enable_q6_candidates": True,
                "q6_joint_name": "JOINT6",
                "q6_neutral": -0.78289622440,
                "q6_cable_limit": 1.0472,
                "q6_offset_candidates": [
                    0.0,
                    0.3491, -0.3491,
                    0.6981, -0.6981,
                    1.0472, -1.0472,
                ],
                "q6_candidates": [
                    -1.5708, 1.5708,
                    0.0,
                    -2.0944, 2.0944,
                    -1.0472, 1.0472
                ],

                "always_use_waypoints": True,
            }],
        ),
    ])
