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

                # Accurate75:
                # reach mode 작업영역의 약 75%만 사용하면서 정확도 우선
                "preferred_joints": [
                    -0.0318,
                    -1.20,
                    -0.95,
                     0.0106,
                     0.0404,
                     3.2295
                ],

                # 자세 유지 약하게 복구
                "posture_weights": [
                    0.10,
                    0.35,
                    0.35,
                    0.20,
                    0.10,
                    0.10
                ],

                # 이전 자세 연속성 약하게 복구
                "continuity_weights": [
                    0.10,
                    0.50,
                    0.50,
                    0.25,
                    0.15,
                    0.15
                ],

                # 정확도 강화
                "ik_position_tolerance": 0.008,
                "ik_max_iterations": 1000,
                "ik_damping": 0.045,
                "ik_step_scale": 0.35,

                # reach mode보다는 자세 안정성 복구, stable보다는 약하게
                "nullspace_gain": 0.0015,
                "continuity_gain": 0.006,
                "joint_limit_gain": 0.008,
                "joint_limit_margin": 0.035,

                # 관절 이동은 허용하되, 너무 튀는 움직임은 줄임
                "max_joint_step": 0.035,
                "max_goal_joint_delta": 2.20,

                # trajectory 검증 강화
                "max_waypoint_joint_jump": 0.45,
                "max_total_joint_delta": 9.00,

                "planning_time": 10.0,
                "planning_attempts": 20,
                "velocity_scaling": 0.12,
                "accel_scaling": 0.12,

                # reach 성공 작업영역의 75% 자동 계산값
                "workspace_x_min": 0.3000,
                "workspace_x_max": 0.5000,
                "workspace_y_min": -0.4875,
                "workspace_y_max": 0.5625,
                "workspace_z_min": 0.0000,
                "workspace_z_max": 0.5000,

                # 안전 핵심값. 실제 실행 안 함.
                "execute_plan": False,

                # 정확도/반복성을 위해 plan-only 성공 목표를 다음 기준으로 사용
                "save_last_goal_on_plan_only": True,
            }],
        ),
    ])
