#!/usr/bin/env bash
# Rydder zombie DDS-participants og stale shared memory fra tidligere SIGKILL.
# Kjøres automatisk (via alias) før ros2 launch på Pi.
set -u
pkill -KILL -f 'controller_server|lifecycle_manager|bt_navigator|planner_server|behavior_server|smoother_server|robot_state_publisher|ekf_node|motor_driver|odometry_node|arm_controller|ldlidar|camera_node|yolo_detector|bear_mission' 2>/dev/null || true
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* 2>/dev/null || true
rm -rf /tmp/fastrtps_* /tmp/fastdds_* 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
echo "DDS clean — klar for launch."
