# DROK_ARM_EEcontrol

DROK arm-only end-effector control with ROS2 Humble, MoveIt2, Gazebo, and joystick input.

This package launches the DROK arm in Gazebo and controls the end-effector target marker using a joystick.  
The control node uses a null-space based IK approach to move the end-effector while trying to maintain a preferred arm posture.

## Packages

- `arm_only_description`: robot URDF/Xacro, mesh files, and Gazebo world
- `arm_only_moveit_config`: MoveIt2, ros2_control, Gazebo launch/config files
- `arm_only_ee_joystick_control`: joystick-based EE target marker control with null-space IK

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- Gazebo Classic
- ros2_control
- joy package

## Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-controller-manager \
  ros-humble-joy \
  ros-humble-xacro
```

## Build

```bash
cd ~/DROK_ARM_EEcontrol
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Use three terminals.

### Terminal 1: Launch Gazebo and MoveIt

```bash
cd ~/DROK_ARM_EEcontrol
source install/setup.bash
ros2 launch arm_only_moveit_config arm_only_gazebo_ros2_control.launch.py
```

This launches the DROK arm model in Gazebo and starts the MoveIt control environment.

### Terminal 2: Run joystick node

```bash
cd ~/DROK_ARM_EEcontrol
source install/setup.bash
ros2 run joy joy_node
```

This reads the connected joystick or gamepad input.

### Terminal 3: Run EE joystick control node

```bash
cd ~/DROK_ARM_EEcontrol
source install/setup.bash
ros2 launch arm_only_ee_joystick_control ee_joy_plan_execute.launch.py
```

This starts the end-effector target marker control node.

## Joystick Control

- Left stick: move the EE target marker in the XY direction
- Right stick / trigger input: move the EE target marker in the Z direction
- A button: plan and execute motion to the target marker
- X button: reset target marker to the current end-effector position
- Y button: move the robot arm to the home pose
- LB / RB: decrease or increase marker movement speed

## Basic Workflow

1. Launch Gazebo and MoveIt.
2. Run the joystick node.
3. Run the EE joystick control node.
4. Move the target marker with the joystick.
5. Press the A button to plan and execute the robot motion.
6. Use X to reset the marker if the target becomes too far from the current end-effector position.

## Notes

If planning fails, reduce the target marker movement distance and try again.  
For stable operation, move the marker in small steps rather than sending a large target displacement at once.
