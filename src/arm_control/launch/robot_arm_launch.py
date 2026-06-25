import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    """
    RMD 모터 제어 및 로봇 팔 시뮬레이션을 위한 ROS 2 런치 파일입니다.
    """

    # --- URDF 및 RViz 시각화 설정 ---
    pkg_path = get_package_share_directory('arm_control')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_arm.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = {'robot_description': doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_config]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')]
    )
    
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )

    # --- 시스템 노드 설정 ---
    motor_reader_node = Node(
        package='arm_control',
        executable='joystick_node_92',
        name='motor_angle_publisher',
        output='log'
    )

    motor_control_node = Node(
        package='arm_control',
        executable='joystick_to_rmd_control_node',
        name='joystick_to_rmd_control',
        output='screen',
        parameters=[{
            # --- 조이스틱 축 설정 ---
            'axis.can10_motor1': 3, 'axis.can10_motor4': 1, 'axis.can10_motor2': 4,
            'axis.can11_motor1': 0, 'axis.can11_motor2': 7, 'axis.can11_motor3': 6,
            'invert.can10_motor1': False, 'invert.can10_motor4': False, 'invert.can10_motor2': False,
            'invert.can11_motor1': False, 'invert.can11_motor2': False, 'invert.can11_motor3': False,
            'invert.can11_motor4': False,
            
            # --- 튜닝 파라미터 ---
            # 그룹 1: can10 모터들
            'group_can10.speed_scales': [0.0, 100000.0, 250000.0],
            'group_can10.curve_exponent': 2.0,
            'group_can10.cos_curve_factor': 1.0,
            'group_can10.max_speed_scale': 65534,

            # 그룹 2: can11 id1, id2
            'group_can11_id1_id2.speed_scales': [10000.0, 20000.0, 50000.0],
            'group_can11_id1_id2.curve_exponent': 2.0,
            'group_can11_id1_id2.cos_curve_factor': 1.0,
            'group_can11_id1_id2.max_speed_scale': 7000,

            # 그룹 3: can11 id3, id4
            'group_can11_id3_id4.speed_scales': [70000.0, 140000.0, 200000.0],
            'group_can11_id3_id4.curve_exponent': 2.0,
            'group_can11_id3_id4.cos_curve_factor': 1.0,
            'group_can11_id3_id4.max_speed_scale': 65534,
            
            # --- 모터 안전 각도 제한 ---
            'limits.can10_motor1.min': -45.0, 'limits.can10_motor1.max': 120.0,
            'limits.can10_motor2.min': 0.0, 'limits.can10_motor2.max': 180.0,
            'limits.can10_motor4.min': 0.0, 'limits.can10_motor4.max': 180.0,
            'limits.can11_motor1.min': -90.0, 'limits.can11_motor1.max': 90.0,
            'limits.can11_motor2.min': -90.0, 'limits.can11_motor2.max': 90.0,
            'limits.can11_motor3.min': -60.0, 'limits.can11_motor3.max': 60.0,
            'limits.can11_motor4.min': -555.0, 'limits.can11_motor4.max': 0.0,
        }]

    )

    inverse_kinematics_node = Node(
        package='arm_control',
        executable='inverse_kinematics_node',
        name='inverse_kinematics_node',
        output='screen'
    )

    # --- 실행할 노드 목록 반환 ---
    return LaunchDescription([
        static_tf_pub_node,
        robot_state_publisher_node,
        rviz_node,
        motor_reader_node,
        motor_control_node,
        # joy_node, # IK 시스템과 함께 사용 시 주석 해제
        inverse_kinematics_node
    ])
