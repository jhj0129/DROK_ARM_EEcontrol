from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Camera fixed pose relative to robot base
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

        # VLM / mask / 3D position collector
        Node(
            package="arm_only_ee_joystick_control",
            executable="vlm_mask_position_collector_node",
            name="vlm_mask_position_collector_node",
            output="screen",
            parameters=[
                {
                    # Stable pyrealsense2 RGB-D topics
                    "rgb_topic": "/test_rs/color/image_raw",
                    "depth_topic": "/test_rs/depth/image_raw",
                    "info_topic": "/test_rs/color/camera_info",

                    # TF
                    "target_frame": "ARM_BASE_LINK",
                    "source_frame_fallback": "camera_color_optical_frame",

                    # Detection mode
                    "mode": "color",
                    "target_color": "blue",

                    # First test: do not move robot yet
                    "target_topic": "/ee_obstacle/target_point",
                    "publish_enabled": False,
                    "publish_once": True,

                    # 0.2 sec = about 5Hz requested processing
                    "process_period": 0.2,

                    # Depth filter
                    "raw_depth_min": 0.10,
                    "raw_depth_max": 3.00,

                    # Mask/depth validity
                    "min_mask_pixels": 50,
                    "min_valid_depth_pixels": 30,

                    # Lower CPU load
                    "max_points_for_stats": 300,

                    # Dataset saving off during test
                    "save_enabled": False,
                    "save_once": True,

                    # Workspace safety range
                    "arm_x_min": 0.10,
                    "arm_x_max": 1.08,
                    "arm_y_abs_max": 0.60,
                    "arm_z_min": -0.05,
                    "arm_z_max": 0.90,
                }
            ],
        ),
    ])
