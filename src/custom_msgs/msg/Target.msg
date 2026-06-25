# custom_msgs/msg/Target.msg

# --- 제어 모드 ---
# 0: POSITION_ONLY (자세 제어 안함)
# 1: FULL_POSE (사용자 정의 자세로 제어)
# 2: PLANE_CONSTRAINT (지정된 평면에 평행하게 자세 제어)
int8 mode

# --- 평면 타입 (mode가 2일 때만 사용) ---
# 0: XY_PLANE (수평)
# 1: YZ_PLANE (수직)
# 2: XZ_PLANE (수직)
int8 plane_type

# --- 목표 위치 (미터) ---
geometry_msgs/Point position

# --- 목표 자세 (쿼터니언) ---
# FULL_POSE 모드에서만 사용자가 지정한 값이 의미를 가짐.
# 그 외 모드에서는 기본값(w=1)으로 전송.
geometry_msgs/Quaternion orientation

# --- 동작 시간 (초) ---
float64 duration_s
