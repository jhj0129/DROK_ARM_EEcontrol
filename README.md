# DROK Real Arm MoveIt + RMD CAN Control

ROS 2 Humble + MoveIt 2 + RViz + RMD CAN 모터 기반 DROK 실제 로봇팔 제어 workspace입니다.

현재 상태:

- MoveIt RViz planning 성공
- RViz Execute로 실제 로봇팔 구동 성공
- JOINT2 이중 모터 요요 구조 반영
- JOINT6 continuous roll / 과회전 문제 해결
- 그리퍼 visual 정렬 완료
- dry-run bridge / real-run bridge 분리
- 실제 구동 전 trajectory 변환값 확인 가능

---

## Workspace

```bash
~/DROK_real_ws
```

주요 패키지:

```text
arm_control
arm_only_description
arm_only_moveit_config
```

주요 파일:

```text
src/arm_control/src/joystick_node_92.cpp
src/arm_control/scripts/moveit_to_rmd_bridge.py
src/arm_only_description/urdf/arm_only_gripper_moveit.urdf.xacro
src/arm_only_moveit_config/config/arm_only_gripper.srdf
src/arm_only_moveit_config/config/joint_limits.yaml
src/arm_only_moveit_config/config/initial_positions.yaml
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

## Build

```bash
cd ~/DROK_real_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select arm_control arm_only_description arm_only_moveit_config
source install/setup.bash
```

---

## 실행 순서

항상 터미널 3개를 기본으로 사용합니다.

---

### Terminal 1: Joint State Publisher

실제 RMD 모터 각도를 읽고 MoveIt용 `/joint_states`를 발행합니다.  
이 노드는 실제 팔을 움직이지 않습니다.

```bash
cd ~/DROK_real_ws
source install/setup.bash

ros2 run arm_control joystick_node_92
```

---

### Terminal 2-A: Dry-run Bridge

실제 CAN 명령을 보내지 않고, MoveIt trajectory가 RMD raw angle로 어떻게 변환되는지만 출력합니다.

```bash
cd ~/DROK_real_ws
source install/setup.bash

ros2 run arm_control moveit_to_rmd_bridge.py \
  --ros-args \
  -p dry_run:=true \
  -p default_max_speed:=10 \
  -p gripper_max_speed:=5
```

정상 로그:

```text
DRY RUN MODE: CAN sockets are NOT opened.
No motor command will be sent.
```

---

### Terminal 2-B: Real-run Bridge

실제 모터가 움직입니다.

```bash
cd ~/DROK_real_ws
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
```

---

### Terminal 3: MoveIt / RViz

```bash
cd ~/DROK_real_ws
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

## MoveIt Start Tolerance

Execute 직전에 current state와 trajectory start state 차이 때문에 실패하면 다음을 실행합니다.

```bash
ros2 param set /move_group trajectory_execution.allowed_start_tolerance 0.05
```

확인:

```bash
ros2 param get /move_group trajectory_execution.allowed_start_tolerance
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

### Check `/joint_states`

```bash
cd ~/DROK_real_ws
source install/setup.bash

ros2 topic echo /joint_states --once
```

### Check duplicate `/joint_states` publishers

```bash
ros2 topic info /joint_states -v
```

정상:

```text
Publisher count: 1
```

### Check JOINT6 only

```bash
cd ~/DROK_real_ws
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

## Current Confirmed Status

```text
MoveIt Plan 성공
RViz Execute 실제 구동 성공
Home 이동 dry-run 성공
Home 방향 실제 구동 가능
JOINT2 mirror 동작 정상
JOINT6 과회전 해결
그리퍼 visual 방향 정렬 완료
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
