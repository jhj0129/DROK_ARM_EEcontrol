import os
import re
from xml.dom import Node as XMLNode

import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def remove_xml_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == XMLNode.COMMENT_NODE:
            node.removeChild(child)
        else:
            remove_xml_comments(child)


def make_clean_robot_description(xacro_file):
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "use_embedded_ros2_control": "true",
        },
    )

    # 핵심:
    # gazebo_ros2_control이 robot_description 전체 XML을 ROS parameter override로 넘길 때
    # <?xml ...?> / <!-- ... --> 주석이 있으면 parser error가 날 수 있어서 제거한다.
    remove_xml_comments(doc)

    xml = doc.toxml()

    # XML declaration 제거
    xml = re.sub(r'<\?xml[^>]*\?>', '', xml).strip()

    # 혹시 남은 comment 제거
    xml = re.sub(r'<!--.*?-->', '', xml, flags=re.S).strip()

    return xml


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    moveit_config_share = get_package_share_directory("arm_only_moveit_config")
    desc_share = get_package_share_directory("arm_only_description")

    xacro_file = os.path.join(
        desc_share,
        "urdf",
        "arm_only_gripper_moveit.urdf.xacro",
    )

    world_file = os.path.join(
        desc_share,
        "worlds",
        "arm_only_ros2_control.world",
    )

    robot_description_xml = make_clean_robot_description(xacro_file)

    robot_description = {
        "robot_description": robot_description_xml
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
            "-entity", "drok_arm_gripper",
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

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )


    plate_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "plate_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    gripper_plate_mimic_node = Node(
        package="arm_only_moveit_config",
        executable="gripper_plate_mimic_node.py",
        output="screen",
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

    return LaunchDescription([
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

        TimerAction(period=8.0, actions=[
            gripper_controller_spawner
        ]),

        TimerAction(period=8.5, actions=[
            plate_controller_spawner
        ]),

        TimerAction(period=10.0, actions=[
            gripper_plate_mimic_node
        ]),

        TimerAction(period=9.0, actions=[
            SetParameter(name="use_sim_time", value=True),
            move_group
        ]),

        TimerAction(period=11.0, actions=[
            SetParameter(name="use_sim_time", value=True),
            moveit_rviz
        ]),
    ])
