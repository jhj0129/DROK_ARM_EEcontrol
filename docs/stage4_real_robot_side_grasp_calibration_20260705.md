# Stage 4 Real Robot Side Grasp Calibration

Date: 2026-07-05

Goal:
Prepare the real DROK arm for VLM based side grasping.

Side grasp strategy:
- grasp pose is generated from the detected object grasp point.
- pregrasp pose is shifted 10 cm in the minus X direction.
- approach motion is from pregrasp to grasp along plus X.
- gripper tcp orientation is kept the same for pregrasp and grasp.

Real home calibration:
- JOINT1 raw home was updated to 5.109444444444445 degrees.
- JOINT6 raw home was updated to -20.668333333333333 degrees.
- JOINT6 MoveIt home is -0.78289622440 rad.

Wiring safe limits:
- JOINT4: lower -5.89 rad, upper 2.64 rad.
- JOINT5: lower -1.52 rad, upper 1.32 rad.
- JOINT6: lower -2.87729 rad, upper 1.31150 rad.

Model updates:
- JOINT6 was changed from continuous to revolute.
- JOINT6 now has wiring safe position limits.
- Full pose IK is enabled by setting position_only_ik to false.
- The gripper mount direction was tuned to match the real gripper.

Tested side grasp quaternion:
- target_qx: 1.000
- target_qy: 0.000
- target_qz: -0.004
- target_qw: -0.002

Current status:
- Real home alignment is done.
- JOINT1 and JOINT6 home calibration is done.
- JOINT4, JOINT5, and JOINT6 wiring protection limits are applied.
- JOINT6 revolute limit model is working.
- Full pose IK is enabled.
- Side grasp plan-only test is working.
