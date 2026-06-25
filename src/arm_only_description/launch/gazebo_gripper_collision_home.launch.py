from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("arm_only_description"),
        "urdf",
        "arm_only_gripper_moveit.urdf.xacro",
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare("arm_only_description"),
        "worlds",
        "arm_only_ros2_control.world",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]),
            value_type=str
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": world_file,
            "pause": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "arm_only_gripper_collision_check",

            # robot base spawn pose
            "-x", "0",
            "-y", "0",
            "-z", "0.0",

            # home pose joint values
            "-J", "JOINT1", "-0.0318",
            "-J", "JOINT2", "-0.2002",
            "-J", "JOINT3", "-0.2955",
            "-J", "JOINT4", "0.0106",
            "-J", "JOINT5", "0.0404",
            "-J", "JOINT6", "3.3295",

            # gripper open pose
            "-J", "JOINT7", "0.0",
            "-J", "LEFT_PLATE_JOINT", "0.0",
            "-J", "RIGHT_PLATE_JOINT", "0.0",
        ],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
    ])
