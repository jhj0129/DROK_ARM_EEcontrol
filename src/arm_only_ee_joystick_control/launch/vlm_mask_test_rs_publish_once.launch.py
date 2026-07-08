from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_arm_base_to_camera",
            output="screen",
            arguments=[
                "--x", "-0.3915",
                "--y", "0.0",
                "--z", "0.5050",
                "--qx", "0.5",
                "--qy", "-0.5",
                "--qz", "0.5",
                "--qw", "-0.5",
                "--frame-id", "ARM_BASE_LINK",
                "--child-frame-id", "camera_color_optical_frame",
            ],
        ),

        Node(
            package="arm_only_ee_joystick_control",
            executable="vlm_mask_position_collector_node",
            name="vlm_mask_position_collector_node",
            output="screen",
            parameters=[
                {
                    "rgb_topic": "/test_rs/color/image_raw",
                    "depth_topic": "/test_rs/depth/image_raw",
                    "info_topic": "/test_rs/color/camera_info",

                    "target_frame": "ARM_BASE_LINK",
                    "source_frame_fallback": "camera_color_optical_frame",

                    "mode": "color",
                    "target_color": "blue",

                    "target_topic": "/ee_obstacle/target_point",

                    # 이번 테스트 핵심
                    "publish_enabled": True,
                    "publish_once": True,

                    "process_period": 0.2,

                    "raw_depth_min": 0.10,
                    "raw_depth_max": 1.50,

                    "min_mask_pixels": 200,
                    "min_valid_depth_pixels": 100,
                    "max_points_for_stats": 300,

                    "save_enabled": False,
                    "save_once": True,

                    "arm_x_min": 0.10,
                    "arm_x_max": 1.08,
                    "arm_y_abs_max": 0.60,
                    "arm_z_min": -0.05,
                    "arm_z_max": 0.90,
                }
            ],
        ),
    ])
