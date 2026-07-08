#!/usr/bin/env bash
set -e

TOPIC="/ee_precision/target_point"
MSG_TYPE="geometry_msgs/msg/PointStamped"
FRAME="ARM_BASE_LINK"
OUT_CSV="precision_boundary_targets.csv"

echo "direction,x,y,z" > "$OUT_CSV"

send_point () {
  local direction="$1"
  local x="$2"
  local y="$3"
  local z="$4"

  echo ""
  echo "=================================================="
  echo "TARGET: $direction  x=$x  y=$y  z=$z"
  echo "=================================================="

  echo "$direction,$x,$y,$z" >> "$OUT_CSV"

  ros2 topic pub --once "$TOPIC" "$MSG_TYPE" \
  "{header: {frame_id: '$FRAME'}, point: {x: $x, y: $y, z: $z}}"

  sleep 3
}

echo "Start precision boundary sweep"
echo "This sweep focuses only on the success/failure boundary."
echo ""

# 기준점
send_point "CENTER_SAFE" 0.35 0.00 0.25

# --------------------------------------------------
# 1. 앞쪽 +X 경계: 기존 x=0.50 성공, x=0.65 실패
# --------------------------------------------------
for x in 0.52 0.54 0.56 0.58 0.60 0.62 0.64; do
  send_point "BOUNDARY_FORWARD_X+" "$x" 0.00 0.25
done

# --------------------------------------------------
# 2. 오른쪽 -Y 경계: 기존 y=-0.20 성공, y=-0.40 실패
# --------------------------------------------------
for y in -0.22 -0.24 -0.26 -0.28 -0.30 -0.32 -0.34 -0.36 -0.38; do
  send_point "BOUNDARY_RIGHT_Y-" 0.50 "$y" 0.25
done

# --------------------------------------------------
# 3. 왼쪽 +Y 경계: 기존 y=+0.20 성공, y=+0.40 실패
# --------------------------------------------------
for y in 0.22 0.24 0.26 0.28 0.30 0.32 0.34 0.36 0.38; do
  send_point "BOUNDARY_LEFT_Y+" 0.50 "$y" 0.25
done

# --------------------------------------------------
# 4. 위쪽 +Z 경계: 기존 z=0.65 성공, z=0.80 실패
# --------------------------------------------------
for z in 0.67 0.69 0.71 0.73 0.75 0.77 0.79; do
  send_point "BOUNDARY_UP_Z+" 0.35 0.00 "$z"
done

# --------------------------------------------------
# 5. 아래쪽 -Z 경계: 기존 z=0.05 성공, z=-0.10 실패
# --------------------------------------------------
for z in 0.03 0.01 0.00 -0.02 -0.04 -0.06 -0.08; do
  send_point "BOUNDARY_DOWN_Z-" 0.35 0.00 "$z"
done

# --------------------------------------------------
# 6. 작은 대각선 안전영역 확인
# 큰 대각선은 실패했으니, 실제 작업에 쓸 수 있는 작은 대각선 확인
# --------------------------------------------------
send_point "SMALL_DIAG_FORWARD_RIGHT_UP"   0.50 -0.20 0.50
send_point "SMALL_DIAG_FORWARD_LEFT_UP"    0.50  0.20 0.50
send_point "SMALL_DIAG_FORWARD_RIGHT_DOWN" 0.50 -0.20 0.10
send_point "SMALL_DIAG_FORWARD_LEFT_DOWN"  0.50  0.20 0.10

echo ""
echo "Boundary sweep finished."
echo "Target list saved to: $OUT_CSV"
