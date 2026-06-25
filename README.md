# DROK Real Arm MoveIt + RMD CAN Control

ROS 2 Humble + MoveIt 2 + RViz + RMD CAN 모터 기반 DROK 실제 로봇팔 제어 workspace입니다.

이 repository는 GitHub에서 clone한 뒤 빌드하면 MoveIt planning, RViz Execute, dry-run trajectory 확인, 실제 RMD CAN 모터 제어까지 이어서 사용할 수 있도록 구성되어 있습니다.

> ⚠️ 실제 하드웨어를 연결한 상태에서는 `dry_run:=false` 실행 시 로봇팔이 실제로 움직입니다. 반드시 dry-run으로 trajectory 변환값을 먼저 확인한 뒤 real-run을 실행하십시오.

---

## Current Status

- MoveIt RViz planning 성공
- RViz Execute로 실제 로봇팔 구동 성공
- JOINT2 이중 모터 요요 구조 반영
- JOINT6 continuous roll / 과회전 문제 해결
- 그리퍼 visual 정렬 완료
- dry-run bridge / real-run bridge 분리
- 실제 구동 전 trajectory 변환값 확인 가능
- clean clone 환경에서 의존 패키지 포함 빌드 성공
- clean clone 환경에서 dry-run bridge 동작 확인

---

## Repository

```bash
git clone https://github.com/jhj0129/DROK_ARM_EEcontrol.git
cd DROK_ARM_EEcontrol
```

사용자 PC에서 원하는 위치에 clone해도 됩니다.  
아래 예시는 기본적으로 `~/DROK_ARM_EEcontrol` 경로를 기준으로 설명합니다.

```bash
cd ~
git clone https://github.com/jhj0129/DROK_ARM_EEcontrol.git ~/DROK_ARM_EEcontrol
cd ~/DROK_ARM_EEcontrol
```

---

## Main Packages

필수 패키지:

```text
custom_msgs
gcode_interpreter
arm_control
arm_only_description
arm_only_moveit_config
```

주요 제어/설정 파일:

```text
src/arm_control/src/joystick_node_92.cpp
src/arm_control/scripts/moveit_to_rmd_bridge.py
src/arm_only_description/urdf/arm_only_gripper_moveit.urdf.xacro
src/arm_only_moveit_config/config/arm_only_gripper.srdf
src/arm_only_moveit_config/config/joint_limits.yaml
src/arm_only_moveit_config/config/initial_positions.yaml
src/arm_only_moveit_config/launch/real_moveit.launch.py
```

---

## Requirements

권장 환경:

```text
Ubuntu 22.04
ROS 2 Humble
MoveIt 2
colcon
Python 3
RMD CAN motor driver environment
CAN interfaces: can10, can11
```

ROS 2 Humble과 MoveIt 2가 설치되어 있어야 합니다.

예시 설치 패키지:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  python3-colcon-common-extensions
```

하드웨어 실제 구동을 위해서는 RMD 모터와 CAN 통신 환경이 별도로 준비되어 있어야 합니다.  
이 프로젝트의 bridge는 `can10`, `can11` 인터페이스를 기준으로 작성되어 있습니다.

---

## Build

처음 clone한 뒤 아래 명령으로 빌드합니다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to arm_control arm_only_description arm_only_moveit_config

source install/setup.bash
```

> 중요: `--packages-select`가 아니라 `--packages-up-to` 사용을 권장합니다.  
> `arm_control`이 `custom_msgs`, `gcode_interpreter`에 의존하므로 `--packages-up-to`를 사용해야 의존 패키지까지 함께 빌드됩니다.

빌드 성공 예시:

```text
Starting >>> custom_msgs
Starting >>> arm_only_description
Finished <<< arm_only_description
Starting >>> arm_only_moveit_config
Finished <<< arm_only_moveit_config
Finished <<< custom_msgs
Starting >>> gcode_interpreter
Finished <<< gcode_interpreter
Starting >>> arm_control
Finished <<< arm_control

Summary: 5 packages finished
```

---

## Motor Mapping

```text
JOINT1 = can10 0x141
JOINT2 = can10 0x142 + can10 0x143 mirror
JOINT3 = can10 0x144
JOINT4 = can11 0x141
JOINT5 = can11 0x142
JOINT6 = can11 0x143
JOINT7 = can11 0x144 gripper
```

JOINT2는 두 개의 모터가 반대 방향으로 움직이는 요요 구조입니다.

```text
MoveIt JOINT2 증가:
can10 0x142 증가
can10 0x143 감소
```

dry-run 로그에서 JOINT2 mirror check가 아래처럼 반대 부호로 나오면 정상입니다.

```text
can10 0x142 delta: +
can10 0x143 delta: -
```

---

## Home Raw Angles

```text
can10 0x141: -4.503333
can10 0x142: 33.330833
can10 0x143: -0.030000
can10 0x144: 21.615833

can11 0x141: 30.480000
can11 0x142: 0.380000
can11 0x143: 35.136667
can11 0x144: -452.570000
```

---

## Execution Overview

기본적으로 터미널 3개를 사용합니다.

```text
Terminal 1: 실제 모터 각도 읽기 + /joint_states 발행
Terminal 2: MoveIt trajectory를 RMD raw angle로 변환하는 bridge
Terminal 3: MoveIt / RViz 실행
```

새 터미널을 열 때마다 반드시 workspace setup을 다시 적용해야 합니다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Terminal 1: Joint State Publisher

실제 RMD 모터 각도를 읽고 MoveIt용 `/joint_states`를 발행합니다.  
이 노드는 실제 팔을 움직이지 않습니다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arm_control joystick_node_92
```

정상적으로 실행되면 MoveIt이 현재 실제 로봇팔의 joint state를 기준으로 planning할 수 있습니다.

---

## Terminal 2-A: Dry-run Bridge

실제 CAN 명령을 보내지 않고, MoveIt trajectory가 RMD raw angle로 어떻게 변환되는지만 출력합니다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arm_control moveit_to_rmd_bridge.py \
  --ros-args \
  -p dry_run:=true \
  -p default_max_speed:=10 \
  -p gripper_max_speed:=5
```

정상 로그:

```text
DRY RUN MODE: CAN sockets are NOT opened. No motor command will be sent.
Action server ready: /arm_controller/follow_joint_trajectory
Action server ready: /gripper_controller/gripper_cmd
```

MoveIt에서 Execute를 누르면 dry-run bridge에 trajectory summary가 출력됩니다.

정상 dry-run 예시:

```text
================ BRIDGE TRAJECTORY SUMMARY ================
dry_run: True
points: 82

--- TOTAL delta deg ---
can10 0x141: -0.409611
can10 0x142: +33.578677
can10 0x143: -33.578677
can10 0x144: +21.392491
can11 0x141: +0.126040
can11 0x142: -4.931790
can11 0x143: +54.093930

--- JOINT2 mirror check ---
can10 0x142 delta: +33.578677 deg
can10 0x143 delta: -33.578677 deg

--- max single waypoint step ---
can11 0x143: 0.687549 deg
===========================================================
```

dry-run에서 확인할 것:

```text
1. dry_run: True
2. JOINT2 mirror check에서 0x142와 0x143 부호가 반대인지 확인
3. max single waypoint step이 너무 크지 않은지 확인
4. JOINT6가 비정상적으로 큰 jump를 만들지 않는지 확인
5. 이상 없을 때만 real-run bridge로 전환
```

---

## Terminal 2-B: Real-run Bridge

⚠️ 실제 모터가 움직입니다.

dry-run bridge를 종료한 뒤 real-run bridge를 실행합니다.  
dry-run bridge와 real-run bridge를 동시에 실행하지 마십시오.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arm_control moveit_to_rmd_bridge.py \
  --ros-args \
  -p dry_run:=false \
  -p default_max_speed:=10 \
  -p gripper_max_speed:=5
```

정상 로그:

```text
REAL SEND MODE ENABLED. Motors may move.
Opened CAN socket: can10
Opened CAN socket: can11
Action server ready
```

속도 기준:

```text
기본 실제 구동: default_max_speed:=10
느리면 중단 후 현재 자세 기준 재계획, default_max_speed:=20까지 증가 가능
처음부터 30 이상은 비추천
```

주의:

```text
dry-run bridge와 real-run bridge를 동시에 실행하지 말 것
실제 구동 전 dry-run 먼저 확인할 것
기존 plan 그대로 재사용하지 말고, bridge 전환 후 다시 Plan 할 것
비정상 동작 시 bridge 터미널에서 Ctrl+C
```

---

## Terminal 3: MoveIt / RViz

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch arm_only_moveit_config real_moveit.launch.py
```

RViz 사용 순서:

```text
Planning Group: arm
Start State: Current
목표 지정
Plan
경로 확인
Execute
```

금지:

```text
Plan & Execute 사용 금지
연속 Execute 금지
경로 확인 없이 Execute 금지
```

---

## Recommended Real-run Procedure

실제 로봇팔 구동 시 권장 절차:

```text
1. Terminal 1에서 joystick_node_92 실행
2. Terminal 2에서 dry-run bridge 실행
3. Terminal 3에서 MoveIt / RViz 실행
4. RViz에서 목표 지정
5. Plan
6. Execute
7. dry-run 로그 확인
8. 이상 없으면 dry-run bridge 종료
9. real-run bridge 실행
10. RViz에서 Start State를 Current로 다시 설정
11. 다시 Plan
12. 경로 확인
13. Execute 1회
14. 이상하면 bridge 터미널 Ctrl+C
```

---

## MoveIt Start Tolerance

Execute 직전에 current state와 trajectory start state 차이 때문에 실패하면 다음을 실행합니다.

```bash
ros2 param set /move_group trajectory_execution.allowed_start_tolerance 0.05
```

확인:

```bash
ros2 param get /move_group trajectory_execution.allowed_start_tolerance
```

예상 문제 로그:

```text
Invalid Trajectory: start point deviates from current robot state more than 0.01
```

---

## JOINT6 Notes

JOINT6는 손목 roll 축입니다.

적용된 처리:

```text
URDF/MoveIt: continuous joint
joint_states: 실제 roll 기준 오프셋 적용
bridge: nearest equivalent angle로 unwrap 처리
RMD gear ratio: 1.0
```

주의:

```text
JOINT6 상태 발행에서 wrap_to_pi를 강제로 적용하지 말 것
continuous joint이므로 등가각 처리는 bridge에서 처리
JOINT6 gear ratio는 1.0으로 유지
```

JOINT6 과회전 문제 해결 핵심:

```text
continuous joint + nearest equivalent angle unwrap + gear_ratio 1.0
```

---

## Gripper Visual Alignment

그리퍼 글씨 방향과 MoveIt visual 방향을 맞추기 위해 gripper visual만 보정했습니다.

```text
LINK6 -> gripper_base fixed joint는 건드리지 않음
gripper_base visual origin만 회전
TCP, joint axis, child frame은 유지
```

fixed joint를 돌리면 gripper frame, TCP, child joint까지 모두 뒤집히므로 visual만 수정해야 합니다.

---

## Safety Rules

```text
1. 실제 구동 전 dry-run 먼저 실행
2. dry-run 로그에서 TOTAL delta deg 확인
3. JOINT2 mirror 부호 확인
4. max single waypoint step 확인
5. 이상 없으면 real-run bridge 실행
6. RViz에서 다시 Plan 후 Execute
7. 이상하면 bridge 터미널 Ctrl+C
```

권장 기준:

```text
max single waypoint step <= 약 1 deg
JOINT2 0x142 / 0x143 delta 부호 반대
JOINT6 jump 없음
```

---

## Useful Commands

### Check package visibility

`Package 'arm_control' not found`가 나오면 workspace setup이 적용되지 않은 상태입니다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 pkg list | grep arm_control
```

정상 출력:

```text
arm_control
```

---

### Check `/joint_states`

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /joint_states --once
```

---

### Check duplicate `/joint_states` publishers

```bash
ros2 topic info /joint_states -v
```

정상:

```text
Publisher count: 1
```

---

### Check JOINT6 only

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
import rclpy
from sensor_msgs.msg import JointState

def cb(msg):
    i = msg.name.index("JOINT6")
    q = msg.position[i]
    print(f"JOINT6 = {q:.6f} rad = {q*180.0/3.1415926535:.3f} deg")
    rclpy.shutdown()

rclpy.init()
node = rclpy.create_node("check_joint6_once")
node.create_subscription(JointState, "/joint_states", cb, 10)
rclpy.spin(node)
PY
```

---

## Troubleshooting

### 1. `Package 'arm_control' not found`

원인:

```text
workspace build가 안 됐거나,
현재 터미널에서 install/setup.bash를 source하지 않은 상태
```

해결:

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep arm_control
```

---

### 2. `custom_msgs` 또는 `gcode_interpreter`를 찾을 수 없음

`--packages-select`만 사용하면 의존 패키지가 같이 빌드되지 않을 수 있습니다.

해결:

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to arm_control arm_only_description arm_only_moveit_config

source install/setup.bash
```

---

### 3. MoveIt Execute 실패: start state 차이

해결:

```bash
ros2 param set /move_group trajectory_execution.allowed_start_tolerance 0.05
```

그리고 RViz에서:

```text
Start State: Current
다시 Plan
다시 Execute
```

---

### 4. dry-run은 되는데 실제 모터가 안 움직임

확인할 것:

```text
dry_run:=false로 bridge 실행했는지 확인
can10, can11 인터페이스가 준비되어 있는지 확인
RMD 모터 전원이 들어와 있는지 확인
bridge 로그에 Opened CAN socket: can10 / can11이 뜨는지 확인
MoveIt에서 Execute를 눌렀는지 확인
```

---

## Current Confirmed Status

```text
GitHub clean clone 성공
의존 패키지 포함 빌드 성공
MoveIt Plan 성공
RViz Execute 실제 구동 성공
Home 이동 dry-run 성공
Home 방향 실제 구동 가능
JOINT2 mirror 동작 정상
JOINT6 과회전 해결
그리퍼 visual 방향 정렬 완료
dry-run bridge 정상 동작 확인
```

---

## TODO

```text
JOINT7 gripper 실제 open/close 검증
Home 복귀 속도 튜닝
작업공간 경계 테스트
충돌 모델 정리
launch 자동화
```
