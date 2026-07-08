from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=False),

        Node(
            package="arm_only_ee_joystick_control",
            executable="vlm_mask_position_collector_node",
            name="vlm_mask_position_collector_node",
            output="screen",
            parameters=[{
                "mode": "manual_bbox",
                "target_color": "blue",
                "manual_bbox": [250, 180, 390, 330],
                "command": "",
                "publish_enabled": False,
                "publish_once": True,
                "save_enabled": True,
                "save_once": True,
                "target_topic": "/ee_obstacle/target_point",
                "target_frame": "ARM_BASE_LINK",
                "rgb_topic": "/camera/camera/color/image_raw",
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "info_topic": "/camera/camera/color/camera_info",
            }],
        ),
    ])
