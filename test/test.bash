#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025 Aki Moto
# SPDX-License-Identifier: BSD-3-Clause

PASS=0
FAIL=1
RES=$PASS

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "[TEST] Running status checker..."
ros2 run speed_lim_pkg check_topics &
CHECKER_PID=$!

echo "[TEST] Starting limiter node..."
ros2 run speed_lim_pkg lim_node.py >/dev/null 2>&1 &
LIMITER_PID=$!

cleanup() {
    echo "[TEST] Cleanup"
    kill $LIMITER_PID >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 5

echo "[TEST] Publishing cmd_vel..."
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.4}, angular: {z: 0.1}}"

sleep 1

wait $CHECKER_PID || RES=$FAIL

if test "${RES}" = 0 ; then
    echo "PASS"
else
    echo "FAILED"
fi

exit $RES

