#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Aki Moto
# SPDX-License-Identifier: BSD-3-Clause

set -e

RES=0

cd /root/ros2_ws
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
source /root/.bashrc

cleanup() {
    echo "[TEST] Cleanup"
    kill $LIMITER_PID >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[TEST] Running status checker..."
ros2 run speed_lim_pkg check_topics &
CHECKER_PID=$!

sleep 5

echo "[TEST] Starting limiter node..."
ros2 run speed_lim_pkg lim_node >/dev/null 2>&1 &
LIMITER_PID=$!

sleep 5

echo "[TEST] Publishing cmd_vel..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.5}, angular: {z: 1.0}}"

wait $CHECKER_PID || RES = 1

if test "${RES}" = 0 ; then
    echo "PASS"
else
    echo "FAILED"
fi

exit $RES

