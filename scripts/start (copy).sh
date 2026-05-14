#!/bin/bash

CATKIN_WS=~/Documents/catkin_ws
LOG_DIR=~/.drone_logs/$(date +%Y-%m-%d_%H-%M-%S)
mkdir -p "$LOG_DIR"
echo "Logs: $LOG_DIR"

log() { echo "[$(date +%H:%M:%S)] $*"; }

cleanup() {
    trap - SIGINT SIGTERM
    log "Shutting down..."
    kill 0
    wait
}
trap cleanup SIGINT SIGTERM

# Reset camera daemon in case it's in a bad state
log "Restarting nvargus-daemon..."
sudo systemctl restart nvargus-daemon 2>&1 | tee "$LOG_DIR/nvargus.log"
sleep 2

# ROS1 bringup (also starts roscore)
(
    source /opt/ros/noetic/setup.bash
    source "$CATKIN_WS/devel/setup.bash"
    roslaunch drone_bringup bringup.launch
) > >(tee "$LOG_DIR/bringup.log") 2>&1 &

# Wait for roscore to be ready
sleep 5

# ros1_bridge
(
    source /opt/ros/noetic/setup.bash
    unset ROS_DISTRO
    source ~/ros2_humble/install/local_setup.bash
    export ROS_DOMAIN_ID=16
    export ROS_MASTER_URI=http://localhost:11311
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
) > >(tee "$LOG_DIR/ros1_bridge.log") 2>"$LOG_DIR/ros1_bridge_err.log" &

# RTK GPS (str2str NTRIP stream -> ublox ROS2 node)
(
    log "Starting str2str NTRIP correction stream..."
    str2str -in ntrip://miskopod10@gmail.com:none@rtk2go.com:2101/MERLIN \
            -out serial://ttyACM0:115200 &
    STR2STR_PID=$!
    log "Waiting for RTK data stream (PID $STR2STR_PID)..."
    sleep 5
    if ! kill -0 "$STR2STR_PID" 2>/dev/null; then
        log "ERROR: str2str exited early, skipping ublox launch"
        wait
        exit 1
    fi
    log "Launching ublox_gps node..."
    unset ROS_DISTRO
    source ~/ros2_humble/install/local_setup.bash
    export ROS_DOMAIN_ID=16
    ros2 launch ublox_gps ublox_gps_node-launch.py
    wait
) > >(tee "$LOG_DIR/rtk_gps.log") 2>&1 &

# GoPro webcam + ros node
# (
#     source ~/ros2_humble/install/local_setup.bash
#     export ROS_DOMAIN_ID=16
#     sudo gopro webcam -r 1080 2>&1 | tee "$LOG_DIR/gopro_webcam.log" &
#     sleep 10
#     ros2 run gopro_ros gopro_ros
# ) > >(tee "$LOG_DIR/gopro_ros.log") 2>&1 &

wait
