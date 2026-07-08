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

                "preferred_joints": [
                    -0.0318,
                    -1.20,
                    -0.95,
                     0.0106,
                     0.0404,
                     3.2295
                ],

                "posture_weights": [
                    0.2,
                    0.8,
                    0.8,
                    0.5,
                    0.3,
                    0.3
                ],

                "continuity_weights": [
                    0.2,
                    0.8,
                    0.8,
                    0.5,
                    0.3,
                    0.3
                ],

                # extreme 테스트:
                # 한계거리 확인 목적이라 tolerance를 약간 풀고 iteration을 크게 둠
                "ik_position_tolerance": 0.020,
                "ik_max_iterations": 400,
                "ik_damping": 0.08,
                "ik_step_scale": 0.55,

                # 자세 보정은 완전히 끄지는 않되, 한계거리 도달을 방해하지 않도록 약하게
                "nullspace_gain": 0.002,
                "continuity_gain": 0.010,
                "joint_limit_gain": 0.02,

                # 한 번에 큰 관절 변화 허용
                "max_joint_step": 0.080,
                "joint_limit_margin": 0.06,
                "max_goal_joint_delta": 1.20,

                # 큰 궤적 허용
                "max_waypoint_joint_jump": 0.80,
                "max_total_joint_delta": 8.00,

                "planning_time": 8.0,
                "planning_attempts": 10,
                "velocity_scaling": 0.20,
                "accel_scaling": 0.20,

                # workspace clamp도 크게 열어둠
                "workspace_x_min": -1.20,
                "workspace_x_max":  1.20,
                "workspace_y_min": -1.20,
                "workspace_y_max":  1.20,
                "workspace_z_min": -0.60,
                "workspace_z_max":  1.20,

                # 안전 핵심값. 실제 execute 안 함.
                "execute_plan": False,

                # plan-only 상태에서도 다음 목표에서 직전 목표 자세를 continuity 기준으로 사용
                "save_last_goal_on_plan_only": True,
            }],
        ),
    ])
