#!/usr/bin/bash
# SPDX-FileCopyrightText: 2025 Aki Moto aki.robosys2025@gmail.com
# SPDX-License-Indentifier: BSD-3-Clause

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

echo "[TEST] Starting limiter node..."
ros2 run speed_lim_pkg lim_node >/dev/null 2>&1 &
LIMITER_PID=$!

sleep 5

run_case () {
    CASE_NAME=$1

    echo "[TEST] CASE: ${CASE_NAME}"

    echo "[TEST] Starting status checker..."
    ros2 run speed_lim_pkg check_topics &
    CHECKER_PID=$!

    sleep 2

    echo "[TEST] Publishing input..."
    case "${CASE_NAME}" in
      NORMAL)
        ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
        "{linear: {x: 0.1}, angular: {z: 0.1}}"
        ;;
      LIMITED)
        ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
        "{linear: {x: 2.0}, angular: {z: 1.0}}"
        ;;
      EMERGENCY)
        ros2 topic pub --once /emergency_stop std_msgs/Bool \
        "{data: true}"
        ;;
    esac

    wait $CHECKER_PID
    RET=$?

    if [ "${RET}" = 0 ]; then
        echo "[TEST] ${CASE_NAME}: PASS"
    else
        echo "[TEST] ${CASE_NAME}: FAIL"
        RES=1
    fi

    sleep 2
}

run_case NORMAL
run_case LIMITED
run_case EMERGENCY

if [ "${RES}" = 0 ]; then
    echo "PASS"
else
    echo "FAILED"
fi

exit $RES

