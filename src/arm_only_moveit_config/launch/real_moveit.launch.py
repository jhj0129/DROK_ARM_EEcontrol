from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_share = get_package_share_directory("arm_only_moveit_config")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "rsp.launch.py"))
    )
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "static_virtual_joint_tfs.launch.py"))
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "move_group.launch.py"))
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "moveit_rviz.launch.py"))
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),
        rsp,
        static_tf,
        move_group,
        rviz,
    ])
