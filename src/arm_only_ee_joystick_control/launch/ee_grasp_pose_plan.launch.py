from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    execute_plan = LaunchConfiguration("execute_plan")

    return LaunchDescription([
        DeclareLaunchArgument(
            "execute_plan",
            default_value="false",
            description="If true, execute the planned trajectory on the controller."
        ),

        SetParameter(name="use_sim_time", value=False),

        Node(
            package="arm_only_ee_joystick_control",
            executable="ee_grasp_pose_plan_node",
            name="ee_grasp_pose_plan_node",
            output="screen",
            parameters=[{
                "use_sim_time": False,

                "planning_group": "arm",
                "ee_link": "gripper_tcp",
                "planner_id": "RRTConnectkConfigDefault",

                "execute_plan": execute_plan,

                "ik_position_tolerance": 0.010,
                "ik_orientation_tolerance": 0.350,
                "ik_max_iterations": 1400,
                "ik_damping": 0.035,
                "ik_step_scale": 0.35,
                "max_joint_step": 0.060,

                "position_weight": 1.0,
                "orientation_weight": 0.25,

                "stage2_base_joint_scale": 0.70,
                "stage2_wrist_joint_scale": 1.00,

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

                "tool_axis": "x",
                "approach_direction_world": [0.50, 0.0, -0.866],
            }],
        ),
    ])
