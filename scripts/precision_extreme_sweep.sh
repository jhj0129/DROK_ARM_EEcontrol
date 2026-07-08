#!/usr/bin/env bash
set -e

TOPIC="/ee_precision/target_point"
MSG_TYPE="geometry_msgs/msg/PointStamped"
FRAME="ARM_BASE_LINK"
OUT_CSV="precision_extreme_targets.csv"

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

echo "Start precision extreme sweep"
echo "Coordinate convention:"
echo "  +X: forward"
echo "  -Y: screen/right side, based on current RViz observation"
echo "  +Y: screen/left side, based on current RViz observation"
echo "  +Z: up"
echo "  -Z: down"
echo ""

# 기준점
send_point "CENTER_SAFE" 0.35 0.00 0.25

# 앞쪽 +X
for x in 0.50 0.65 0.80 0.95 1.10 1.20; do
  send_point "FORWARD_X+" "$x" 0.00 0.25
done

# 오른쪽: 화면 기준 오른쪽은 -Y일 가능성이 큼
for y in -0.20 -0.40 -0.60 -0.80 -1.00 -1.20; do
  send_point "RIGHT_Y-" 0.50 "$y" 0.25
done

# 왼쪽: 화면 기준 왼쪽은 +Y
for y in 0.20 0.40 0.60 0.80 1.00 1.20; do
  send_point "LEFT_Y+" 0.50 "$y" 0.25
done

# 위쪽 +Z
for z in 0.35 0.50 0.65 0.80 0.95 1.10; do
  send_point "UP_Z+" 0.35 0.00 "$z"
done

# 아래쪽 -Z
for z in 0.20 0.05 -0.10 -0.25 -0.40 -0.55; do
  send_point "DOWN_Z-" 0.35 0.00 "$z"
done

# 대각선
send_point "DIAG_FORWARD_RIGHT_DOWN" 0.80 -0.50 0.05
send_point "DIAG_FORWARD_LEFT_DOWN"  0.80  0.50 0.05
send_point "DIAG_FORWARD_RIGHT_UP"   0.80 -0.50 0.75
send_point "DIAG_FORWARD_LEFT_UP"    0.80  0.50 0.75

echo ""
echo "Sweep finished."
echo "Target list saved to: $OUT_CSV"
