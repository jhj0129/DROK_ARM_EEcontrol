# DROK_ARM_EEcontrol

DROK 로봇팔과 P 그리퍼를 ROS 2 Humble, MoveIt 2, Gazebo Classic, ros2_control, 조이스틱 입력으로 제어하기 위한 프로젝트입니다.

이 프로젝트는 Gazebo 환경에서 DROK 로봇팔과 그리퍼 모델을 실행하고, MoveIt 2를 이용해 End-Effector 목표 지점까지 로봇팔을 이동시킵니다.
조이스틱을 이용해 RViz 상의 목표 마커를 움직이고, 버튼 입력으로 로봇팔이 해당 목표 지점까지 계획 및 실행하도록 구성되어 있습니다.

또한 단순한 위치 제어가 아니라, Null-space 기반 IK 방식을 적용하여 End-Effector를 목표 위치로 이동시키는 동시에 로봇팔이 가능한 한 원하는 팔 형태를 유지하도록 설계했습니다.

---

## 주요 기능

* Gazebo에서 DROK 로봇팔 + P 그리퍼 모델 실행
* MoveIt 2 기반 로봇팔 경로 계획 및 실행
* 조이스틱을 이용한 End-Effector 목표 마커 이동
* Null-space 기반 IK를 이용한 팔 자세 유지 제어
* RViz에서 목표 마커 시각화
* `gripper_tcp` 기준 End-Effector 제어
* Gazebo에서 실제 그리퍼 plate 동작 확인 가능
* `JOINT7` 기반 그리퍼 열기/닫기 제어
* `LEFT_PLATE_JOINT`, `RIGHT_PLATE_JOINT`를 Gazebo에서 실제로 움직이기 위한 plate mimic 제어 노드 포함

---

## 패키지 구성

이 프로젝트는 크게 세 개의 ROS 2 패키지로 구성되어 있습니다.

### `arm_only_description`

로봇 모델 관련 파일을 포함합니다.

* DROK 로봇팔 URDF/Xacro
* P 그리퍼가 포함된 로봇 모델
* STL mesh 파일
* Gazebo world 파일
* 로봇 visual/collision 설정

### `arm_only_moveit_config`

MoveIt 2와 ros2_control, Gazebo 실행 설정을 포함합니다.

* MoveIt planning group 설정
* SRDF 설정
* kinematics 설정
* joint limits 설정
* ros2_control controller 설정
* Gazebo + MoveIt 통합 launch 파일
* 그리퍼 plate 동기화를 위한 mimic node

### `arm_only_ee_joystick_control`

조이스틱 기반 End-Effector 제어 노드를 포함합니다.

* 조이스틱 입력 처리
* RViz 목표 마커 제어
* Null-space 기반 IK 계산
* 목표 위치까지 plan/execute 실행
* home pose 이동
* 목표 마커 reset 기능

---

## 시스템 구조

최종 로봇 구조는 다음과 같습니다.

```text
ARM_BASE_LINK
 └─ LINK1
    └─ LINK2
       └─ LINK3
          └─ LINK4
             └─ LINK5
                └─ LINK6
                   └─ gripper_base
                      ├─ drive_gear       ← JOINT7
                      ├─ LEFT_PLATE       ← LEFT_PLATE_JOINT
                      ├─ RIGHT_PLATE      ← RIGHT_PLATE_JOINT
                      └─ gripper_tcp      ← End-Effector 기준 TCP
```

MoveIt에서는 `gripper_tcp`를 End-Effector 기준 링크로 사용합니다.

```text
arm group:
  JOINT1 ~ JOINT6

gripper group:
  JOINT7

End-Effector:
  gripper_tcp
```

`JOINT7`은 팔 IK에 포함하지 않고, 그리퍼 열기/닫기 전용 조인트로 사용합니다.

---

## 그리퍼 제어 구조

MoveIt/RViz에서는 URDF의 mimic joint를 통해 그리퍼가 움직이는 것처럼 표시할 수 있습니다.
하지만 Gazebo에서는 기어 STL이나 mimic 태그만으로 실제 plate 조인트가 자동으로 움직이지 않습니다.

그래서 Gazebo에서는 다음과 같은 구조로 그리퍼를 제어합니다.

```text
gripper_controller:
  JOINT7 제어

plate_controller:
  LEFT_PLATE_JOINT
  RIGHT_PLATE_JOINT 제어

gripper_plate_mimic_node.py:
  JOINT7 값을 읽어서
  LEFT_PLATE_JOINT / RIGHT_PLATE_JOINT가 함께 움직이도록 명령 발행
```

즉, 사용자는 `JOINT7`에 열기/닫기 명령을 보내면 되고, plate 조인트는 내부 mimic node가 자동으로 따라가게 됩니다.

---

## 요구 환경

* Ubuntu 22.04
* ROS 2 Humble
* MoveIt 2
* Gazebo Classic
* ros2_control
* ros2_controllers
* joy package
* xacro

---

## 의존성 설치

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

---

## 빌드 방법

저장소를 clone합니다.

```bash
cd ~
git clone https://github.com/jhj0129/DROK_ARM_EEcontrol.git
cd DROK_ARM_EEcontrol
```

의존성을 확인하고 빌드합니다.

```bash
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --base-paths src \
  --packages-select arm_only_description arm_only_moveit_config arm_only_ee_joystick_control \
  --allow-overriding arm_only_description arm_only_moveit_config arm_only_ee_joystick_control

source install/setup.bash
```

---

## 실행 방법

총 세 개 또는 네 개의 터미널을 사용합니다.

---

## 터미널 1: Gazebo + MoveIt 실행

```bash
cd ~/DROK_ARM_EEcontrol

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MODEL_PATH=$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
export QT_QPA_PLATFORM=xcb

ros2 launch arm_only_moveit_config arm_only_gazebo_ros2_control.launch.py
```

이 명령은 다음 기능을 실행합니다.

* Gazebo Classic 실행
* DROK 로봇팔 + P 그리퍼 모델 spawn
* robot_state_publisher 실행
* ros2_control controller_manager 실행
* joint_state_broadcaster 실행
* arm_controller 실행
* gripper_controller 실행
* plate_controller 실행
* MoveIt move_group 실행
* RViz 실행
* gripper plate mimic node 실행

정상적으로 실행되면 다음 컨트롤러들이 active 상태가 되어야 합니다.

```text
joint_state_broadcaster active
arm_controller active
plate_controller active
gripper_controller active
```

---

## 터미널 2: 조이스틱 노드 실행

```bash
cd ~/DROK_ARM_EEcontrol

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1

ros2 run joy joy_node
```

이 노드는 연결된 조이스틱 또는 게임패드 입력을 읽습니다.

---

## 터미널 3: End-Effector 조이스틱 제어 노드 실행

```bash
cd ~/DROK_ARM_EEcontrol

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1

ros2 launch arm_only_ee_joystick_control ee_joy_plan_execute.launch.py
```

이 노드는 조이스틱 입력을 이용해 RViz 상의 End-Effector 목표 마커를 움직이고, 버튼 입력에 따라 로봇팔의 경로 계획 및 실행을 수행합니다.

---

## 컨트롤러 상태 확인

새 터미널에서 다음 명령을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_EEcontrol/install/setup.bash

export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1

ros2 control list_controllers
```

정상 상태는 다음과 같습니다.

```text
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
plate_controller        joint_trajectory_controller/JointTrajectoryController  active
gripper_controller      position_controllers/GripperActionController           active
```

---

## 그리퍼 단독 테스트

그리퍼를 닫으려면 다음 명령을 실행합니다.

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
"{command: {position: 4.2, max_effort: 50.0}}"
```

그리퍼를 열려면 다음 명령을 실행합니다.

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
"{command: {position: 0.0, max_effort: 50.0}}"
```

조인트 상태를 확인하려면 다음 명령을 실행합니다.

```bash
ros2 topic echo /joint_states --once
```

닫힘 상태에서는 대략 다음과 같은 값이 확인됩니다.

```text
JOINT7 ≈ 4.2
LEFT_PLATE_JOINT ≈ 0.07
RIGHT_PLATE_JOINT ≈ 0.07
```

열림 상태에서는 대략 다음과 같은 값이 확인됩니다.

```text
JOINT7 ≈ 0.0
LEFT_PLATE_JOINT ≈ 0.0
RIGHT_PLATE_JOINT ≈ 0.0
```

---

## 조이스틱 조작 방법

조이스틱 입력은 설정에 따라 약간 다를 수 있지만, 기본적인 사용 방식은 다음과 같습니다.

* 왼쪽 스틱: End-Effector 목표 마커를 XY 방향으로 이동
* 오른쪽 스틱 또는 트리거 입력: End-Effector 목표 마커를 Z 방향으로 이동
* A 버튼: 현재 목표 마커 위치로 경로 계획 및 실행
* X 버튼: 목표 마커를 현재 End-Effector 위치로 초기화
* Y 버튼: 로봇팔을 home pose로 이동
* LB / RB: 목표 마커 이동 속도 감소 / 증가

---

## 기본 사용 순서

1. Gazebo + MoveIt을 실행합니다.
2. 조이스틱 노드를 실행합니다.
3. End-Effector 조이스틱 제어 노드를 실행합니다.
4. RViz에서 목표 마커가 표시되는지 확인합니다.
5. 조이스틱으로 목표 마커를 조금씩 이동합니다.
6. A 버튼을 눌러 로봇팔의 경로 계획 및 실행을 수행합니다.
7. 목표 마커가 너무 멀어졌거나 planning이 실패하면 X 버튼으로 목표 마커를 현재 End-Effector 위치로 초기화합니다.
8. 필요하면 Y 버튼으로 home pose로 복귀합니다.
9. 그리퍼는 `gripper_controller`를 통해 열기/닫기 명령을 보낼 수 있습니다.

---

## Null-space 기반 IK 제어

이 프로젝트의 End-Effector 제어 노드는 단순히 목표 위치만 따라가는 것이 아니라, 로봇팔이 가능한 한 원하는 자세를 유지하도록 Null-space 기반 IK를 사용합니다.

기본 개념은 다음과 같습니다.

```text
1. End-Effector를 목표 방향으로 이동시키는 관절 변화량 계산
2. End-Effector 움직임에 영향을 덜 주는 null-space 방향 계산
3. null-space 방향으로 preferred posture에 가까워지도록 관절 변화량 추가
```

이를 통해 다음과 같은 효과를 기대할 수 있습니다.

* 팔이 과하게 꺾이는 현상 감소
* 특정 관절이 불필요하게 크게 움직이는 현상 감소
* 비슷한 End-Effector 위치에서도 더 안정적인 팔 모양 유지
* 조이스틱으로 EE 목표를 움직일 때 팔 자세가 덜 튐
* planning 성공률 향상

---

## 주의사항

planning이 실패하면 목표 마커 이동 거리를 줄이고 다시 시도하는 것이 좋습니다.

큰 목표 이동을 한 번에 보내기보다, 작은 거리로 여러 번 나누어 이동하는 것이 안정적입니다.

Gazebo와 MoveIt을 함께 사용할 때는 항상 다음 환경 변수를 맞춰주는 것이 좋습니다.

```bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
```

Gazebo 실행 중 controller가 제대로 올라오지 않으면 다음 명령으로 상태를 확인합니다.

```bash
ros2 control list_controllers
```

---

## 문제 해결

### controller_manager가 보이지 않는 경우

```bash
ros2 node list | grep controller
```

`/controller_manager`가 보이지 않으면 Gazebo의 ros2_control plugin이 제대로 올라오지 않은 것입니다.
이 경우 터미널 1의 Gazebo 실행 로그를 확인해야 합니다.

---

### 그리퍼 action은 성공하는데 Gazebo에서 plate가 안 움직이는 경우

다음 명령으로 조인트 상태를 확인합니다.

```bash
ros2 topic echo /joint_states --once
```

`JOINT7`은 움직이는데 `LEFT_PLATE_JOINT`, `RIGHT_PLATE_JOINT`가 움직이지 않으면 `gripper_plate_mimic_node.py` 또는 `plate_controller`가 실행되지 않은 것입니다.

다음 컨트롤러가 active인지 확인합니다.

```bash
ros2 control list_controllers
```

정상 상태에는 반드시 다음 항목이 있어야 합니다.

```text
plate_controller active
gripper_controller active
```

---

### RViz에서 planning이 실패하는 경우

다음 방법을 시도합니다.

* 목표 마커를 더 가까운 위치로 이동
* X 버튼으로 목표 마커를 현재 End-Effector 위치로 초기화
* Y 버튼으로 home pose 복귀
* 다시 작은 이동부터 시도
* 충돌이 발생할 수 있는 위치를 피해서 이동

---

## 현재 검증된 상태

현재 버전에서는 다음 항목이 확인되었습니다.

* GitHub main 브랜치 기준 최신 코드 반영
* STL mesh 파일 포함
* Gazebo에서 로봇팔 + P 그리퍼 spawn 확인
* ros2_control에서 `JOINT1~JOINT7`, `LEFT_PLATE_JOINT`, `RIGHT_PLATE_JOINT` 로딩 확인
* `arm_controller` active 확인
* `gripper_controller` active 확인
* `plate_controller` active 확인
* Gazebo에서 그리퍼 열기/닫기 동작 확인
* 새 clone 후 build 및 실행 확인

---

## 저장소

```text
https://github.com/jhj0129/DROK_ARM_EEcontrol
```
