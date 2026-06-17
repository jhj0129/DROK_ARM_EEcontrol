import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    moveit_config_share = get_package_share_directory("arm_only_moveit_config")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("arm_only_description"),
        "urdf",
        "arm_only_clean_moveit.urdf.xacro",
    ])

    # GitHub에서 받은 다른 컴퓨터에서도 동작하도록
    # Use the world file installed inside arm_only_description package.
    world_file = PathJoinSubstitution([
        FindPackageShare("arm_only_description"),
        "worlds",
        "arm_only_ros2_control.world",
    ])

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
        }.items(),
    )

    # robot_state_publisher는 Gazebo /clock 기준을 써야 함
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "arm_only_clean",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0",
        ],
    )

    static_virtual_joint_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "static_virtual_joint_tfs.launch.py")
        )
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "move_group.launch.py")
        )
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "moveit_rviz.launch.py")
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription([
        # Gazebo는 먼저 실행. 여기에 전역 use_sim_time을 먹이면 /clock이 꼬일 수 있음.
        gazebo,
        robot_state_publisher,
        spawn_robot,

        TimerAction(period=2.0, actions=[
            static_virtual_joint_tf
        ]),

        TimerAction(period=5.0, actions=[
            joint_state_broadcaster_spawner
        ]),

        TimerAction(period=7.0, actions=[
            arm_controller_spawner
        ]),

        # 여기부터 MoveIt 계열 노드에만 use_sim_time 적용
        TimerAction(period=9.0, actions=[
            SetParameter(name="use_sim_time", value=True),
            move_group
        ]),

        TimerAction(period=11.0, actions=[
            SetParameter(name="use_sim_time", value=True),
            moveit_rviz
        ]),
    ])
