# DROK ARM EE Control - Real Gripper Calibration & MoveIt Integration

이 문서는 DROK ARM EE Control 프로젝트에 추가된 **실제 그리퍼 제어**, **MoveIt 그리퍼 시각화 보정**, **JOINT7 피드백 보정**, **cm 단위 그리퍼 명령 테스트 방법**을 정리한 README 삽입용 문서입니다.

---

## 1-1. 개발/검증 완료 내용 및 추가 개선점

현재 프로젝트는 기존 6축 DROK ARM 제어에 실제 그리퍼 구동축 `JOINT7`을 추가하여, MoveIt/RViz와 실제 RMD 모터 기반 그리퍼가 함께 동작하도록 구성되어 있습니다.

검증 완료된 항목은 다음과 같습니다.

- 실제 팔 본체 JOINT1~JOINT6 MoveIt real-run 동작 확인
- 실제 그리퍼 모터 `can11 0x144`를 `JOINT7`로 매핑
- `JOINT7` 명령 범위 설정
  - open: `-1.7`
  - mid: `21.65`
  - close: `45.0`
- 실제 그리퍼 완전 열림/닫힘 동작 확인
- `/joint_states`에서 실제 그리퍼 상태가 MoveIt에 반영되는 것 확인
- MoveIt/RViz에서 그리퍼 plate mimic motion 시각화 보정
- cm 단위 그리퍼 벌림 명령을 action command로 변환하여 테스트 완료
- GitHub push 완료 commit:
  - `cf0bee1 Calibrate real gripper feedback and MoveIt mimic motion`

## 1-2. 추가 개선점 
개선점 및 추가 기능 예정안

-


---

## 2. 실제 그리퍼 기준값

현재 실제 그리퍼는 다음 기준으로 사용합니다.

| 상태 | 실제 플리퍼 사이 거리 | JOINT7 |
|---|---:|---:|
| 완전 열림 | 약 `12 cm` | `-1.7` |
| 중간 | 약 `6 cm` | `21.65` |
| 완전 닫힘 | `0 cm` | `45.0` |

> 참고: 초기에는 14.3cm 기준으로 측정했으나, 최종 실제 동작 검증 결과 현재 운용 기준은 **완전 열림 12cm / 완전 닫힘 0cm**입니다.

---

## 3. JOINT7 명령 변환식

원하는 그리퍼 폭을 cm 단위로 줄 때, `JOINT7` 명령값은 아래 식으로 변환합니다.

```text
q7 = q_close - (q_close - q_open) * (width_cm / width_open_cm)
```

현재 값 대입:

```text
q_open = -1.7
q_close = 45.0
width_open_cm = 12.0

q7 = 45.0 - 46.7 * (width_cm / 12.0)
q7 = 45.0 - 3.8916667 * width_cm
```

예시:

| 원하는 그리퍼 폭 | JOINT7 명령값 |
|---:|---:|
| `12 cm` | `-1.700` |
| `10 cm` | `6.083` |
| `8 cm` | `13.867` |
| `6 cm` | `21.650` |
| `4 cm` | `29.433` |
| `2 cm` | `37.217` |
| `0 cm` | `45.000` |

---

## 4. 실행 순서

### Terminal 1 - 실제 Joint State Publisher 실행

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arm_control joystick_node_92
```

이 노드는 실제 모터 각도를 읽어 `/joint_states`로 발행합니다.  
MoveIt은 이 `/joint_states`를 보고 현재 팔과 그리퍼 상태를 인식합니다.

---

### Terminal 2 - Real-run Bridge 실행

> 주의: 아래 명령은 실제 팔/그리퍼 모터를 움직일 수 있습니다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run arm_control moveit_to_rmd_bridge.py \
  --ros-args \
  -p dry_run:=false \
  -p default_max_speed:=5 \
  -p gripper_max_speed:=300
```

권장 속도:

| 상황 | gripper_max_speed |
|---|---:|
| 초기 테스트 | `300` |
| 일반 검증 | `300 ~ 600` |
| 빠른 동작 테스트 | `900` |

---

### Terminal 3 - MoveIt/RViz 실행

> 주의: RViz에서 Execute를 누르면 실제 팔/그리퍼가 움직입니다.

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch arm_only_moveit_config real_moveit.launch.py
```

RViz에서는 반드시 아래 순서로 실행합니다.

```text
Plan → 경로 확인 → Execute 1회
```

`Plan & Execute`는 사용하지 않습니다.

---

## 5. 그리퍼 직접 action 명령

### 완전 열림

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: -1.7, max_effort: 0.0}}"
```

### 중간

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 21.65, max_effort: 0.0}}"
```

### 완전 닫힘

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 45.0, max_effort: 0.0}}"
```

---

## 6. cm 단위 그리퍼 테스트 명령

터미널에서 아래 함수를 붙여넣으면, 원하는 cm 단위로 그리퍼 폭을 지정할 수 있습니다.

```bash
gripper_cm () {
  CM="$1"

  Q=$(python3 - <<PY
cm = float("$CM")

if cm < 0.0:
    cm = 0.0
if cm > 12.0:
    cm = 12.0

q_open = -1.7
q_close = 45.0
width_open_cm = 12.0

q = q_close - (q_close - q_open) * (cm / width_open_cm)
print(f"{q:.6f}")
PY
)

  echo "target width = ${CM} cm"
  echo "send JOINT7 q = ${Q}"

  ros2 action send_goal /gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    "{command: {position: ${Q}, max_effort: 0.0}}"
}
```

사용 예시:

```bash
gripper_cm 12
gripper_cm 8
gripper_cm 6
gripper_cm 4
gripper_cm 2
gripper_cm 0
```

추천 검증 순서:

```bash
gripper_cm 12
sleep 2
gripper_cm 8
sleep 2
gripper_cm 6
sleep 2
gripper_cm 4
sleep 2
gripper_cm 0
sleep 2
gripper_cm 12
```

---

## 7. 상태 확인 토픽

### joint_states 확인

```bash
ros2 topic echo /joint_states --once
```

`JOINT7` 값이 다음과 같이 나오면 정상입니다.

| 실제 상태 | JOINT7 |
|---|---:|
| 완전 열림 | 약 `-1.7` |
| 중간 | 약 `21.65` |
| 완전 닫힘 | 약 `45.0` |

---

### 그리퍼 폭 확인

```bash
ros2 topic echo /gripper/width_cm --once
```

### 닫힘 비율 확인

```bash
ros2 topic echo /gripper/close_ratio --once
```

닫힘 비율 기준:

| 상태 | close_ratio |
|---|---:|
| 완전 열림 | `0.0` |
| 중간 | `0.5` |
| 완전 닫힘 | `1.0` |

---

## 8. MoveIt 그리퍼 시각화 보정

실제 그리퍼는 회전 모터 `JOINT7`이 구동하지만, MoveIt/RViz에서는 좌우 plate가 prismatic joint처럼 횡방향으로 이동하는 형태로 표현됩니다.

따라서 `LEFT_PLATE_JOINT`, `RIGHT_PLATE_JOINT`는 `JOINT7`을 mimic하도록 설정했습니다.

최종 시각화 보정 기준:

```text
q_open = -1.7
q_close = 45.0

실제 open width = 0.120 m
시각화 close gap correction = 0.040 m

plate_stroke_each = (0.120 + 0.040) / 2
                  = 0.080 m
```

최종 mimic 계산값:

```text
mimic_multiplier = 0.080 / (45.0 - (-1.7))
                 ≈ 0.0017130621

mimic_offset = -(-1.7) * mimic_multiplier
             ≈ 0.0029122056
```

의도한 MoveIt 표시:

| JOINT7 | 표시 상태 | Plate 이동량 |
|---:|---|---:|
| `-1.7` | open | `0.000 m` |
| `21.65` | mid | `0.040 m each` |
| `45.0` | close | `0.080 m each` |

---

## 9. 빌드 명령

```bash
cd ~/DROK_ARM_EEcontrol
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to arm_control arm_only_description arm_only_moveit_config

source install/setup.bash
```

정상 빌드 예시:

```text
Summary: 5 packages finished
```

---

## 10. 안전 주의사항

- `moveit_to_rmd_bridge.py`를 `dry_run:=false`로 실행하면 실제 모터가 움직일 수 있습니다.
- dry-run bridge와 real-run bridge를 동시에 실행하지 않습니다.
- RViz에서 `Plan & Execute`를 사용하지 않습니다.
- 항상 `Plan → 경로 확인 → Execute 1회` 순서로 실행합니다.
- 이상 동작 시 bridge 터미널에서 즉시 `Ctrl+C`를 누릅니다.
- 실제 그리퍼 테스트 시 손이나 물체를 plate 사이에 넣지 않습니다.

---

## 11. GitHub 반영 상태

최종 변경사항은 GitHub `main` 브랜치에 push 완료되었습니다.

```text
commit cf0bee1
Calibrate real gripper feedback and MoveIt mimic motion
```

최종 확인 상태:

```text
현재 브랜치 main
브랜치가 'origin/main'에 맞게 업데이트된 상태입니다.
커밋할 사항 없음, 작업 폴더 깨끗함
```
