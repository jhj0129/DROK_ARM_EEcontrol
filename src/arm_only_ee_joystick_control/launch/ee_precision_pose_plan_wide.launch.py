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
                    0.3,
                    1.2,
                    1.2,
                    0.8,
                    0.4,
                    0.4
                ],

                "continuity_weights": [
                    0.3,
                    1.5,
                    1.5,
                    0.8,
                    0.5,
                    0.5
                ],

                # wide 테스트용: 기존보다 더 멀리 움직이게 제한 완화
                "ik_position_tolerance": 0.008,
                "ik_max_iterations": 260,
                "ik_damping": 0.06,
                "ik_step_scale": 0.40,

                "nullspace_gain": 0.005,
                "continuity_gain": 0.04,
                "joint_limit_gain": 0.01,

                "max_joint_step": 0.040,
                "joint_limit_margin": 0.12,
                "max_goal_joint_delta": 0.45,

                "max_waypoint_joint_jump": 0.35,
                "max_total_joint_delta": 3.50,

                "planning_time": 6.0,
                "planning_attempts": 8,
                "velocity_scaling": 0.20,
                "accel_scaling": 0.20,

                "workspace_x_min": -0.70,
                "workspace_x_max": 0.90,
                "workspace_y_min": -0.70,
                "workspace_y_max": 0.70,
                "workspace_z_min": -0.20,
                "workspace_z_max": 0.90,

                # 안전 기본값. 이 상태에서는 실제 execute 안 함.
                "execute_plan": False,

                # 실행하지 않아도 다음 좌표에서 직전 목표 자세를 continuity 기준으로 사용
                "save_last_goal_on_plan_only": True,
            }],
        ),
    ])
