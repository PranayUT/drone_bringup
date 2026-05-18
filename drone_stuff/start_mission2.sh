#!/bin/bash
#
# CSI / nvargus + YOLO (bench_best_v1.py --source camera):
# - Default: restarts nvargus-daemon so the camera stack is clean before ROS. That kills any
#   other app already using nvarguscamerasrc (e.g. a running YOLO bench). Stop those first.
# - Only one consumer should open the same CSI sensor at a time (ROS camera XOR direct GStreamer).
#   If drone_bringup enables jetson_csi_cam, do not run bench in parallel on CSI; use a ROS image topic instead.
# - Suggested order: run this script first, wait until bringup is up, then start extras that need CSI
#   only if bringup does not own the camera (see bringup2.launch).
# - To skip the daemon bounce (e.g. you already reset nvargus and no stale state): SKIP_NVARGUS_RESTART=1 $0 ...
#
# Loiter mission: dji_sdk + waypoint_runner2 (orbit radius locked from height above takeoff at arrival).
#

CATKIN_WS=~/Documents/catkin_ws
log() { echo "[$(date +%H:%M:%S)] $*"; }

STAMP=$(date +%Y-%m-%d_%H-%M-%S)
LOG_DIR="${DRONE_MISSION_LOG_DIR:-/media/drone/extreme/logs/$STAMP}"
if ! mkdir -p "$LOG_DIR" 2>/dev/null; then
    LOG_DIR="$HOME/.drone_logs/$STAMP"
    if ! mkdir -p "$LOG_DIR"; then
        log "ERROR: could not create log directory under /media/drone/extreme/logs or $HOME/.drone_logs"
        exit 1
    fi
    log "External log drive unavailable; using $LOG_DIR"
fi

if [ -z "$1" ]; then
    echo "Usage: $0 <waypoints_file> [speed] [loiter_secs]"
    echo "Optional env: SKIP_NVARGUS_RESTART=1  (do not restart nvargus-daemon before bringup)"
    exit 1
fi

WAYPOINTS_FILE="$(realpath "$1")"
SPEED="${2:-5.0}"
LOITER="${3:-10.0}"

log "Logs: $LOG_DIR"
log "Waypoints: $WAYPOINTS_FILE  speed: ${SPEED} m/s  loiter: ${LOITER} s  runner: waypoint_runner2"

cleanup() {
    trap - SIGINT SIGTERM
    log "Shutting down..."
    kill 0
    wait
}
trap cleanup SIGINT SIGTERM

# ROS1 bringup (camera + dji_sdk + waypoint_runner2)
log "Launching drone_bringup (waypoint_runner2)..."
(
    source /opt/ros/noetic/setup.bash
    source "$CATKIN_WS/devel/setup.bash"
    roslaunch drone_bringup bringup2.launch \
        waypoints_file:="$WAYPOINTS_FILE" \
        speed:="$SPEED" \
        loiter:="$LOITER"
) > >(tee "$LOG_DIR/bringup.log") 2>&1 &

# Wait for roscore to be ready
sleep 5

# # ros1_bridge
# log "Starting ros1_bridge..."
# (
#     source /opt/ros/noetic/setup.bash
#     unset ROS_DISTRO
#     source ~/ros2_humble/install/local_setup.bash
#     export ROS_DOMAIN_ID=16
#     export ROS_MASTER_URI=http://localhost:11311
#     ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
# ) > >(tee "$LOG_DIR/ros1_bridge.log") 2>"$LOG_DIR/ros1_bridge_err.log" &

# # RTK GPS (str2str NTRIP stream -> ublox ROS2 node)
# log "Starting RTK GPS..."
# (
#     log "Starting str2str NTRIP correction stream..."
#     str2str -in ntrip://miskopod10@gmail.com:none@rtk2go.com:2101/MERLIN \
#             -out serial://ttyACM0:115200 &
#     STR2STR_PID=$!
#     log "Waiting for RTK data stream (PID $STR2STR_PID)..."
#     sleep 5
#     if ! kill -0 "$STR2STR_PID" 2>/dev/null; then
#         log "ERROR: str2str exited early, skipping ublox launch"
#         wait
#         exit 1
#     fi
#     log "Launching ublox_gps node..."
#     unset ROS_DISTRO
#     source ~/ros2_humble/install/local_setup.bash
#     export ROS_DOMAIN_ID=16
#     ros2 launch ublox_gps ublox_gps_node-launch.py
#     wait
# ) > >(tee "$LOG_DIR/rtk_gps.log") 2>&1 &

# GoPro webcam + ros node
# (
#     source ~/ros2_humble/install/local_setup.bash
#     export ROS_DOMAIN_ID=16
#     sudo gopro webcam -r 1080 2>&1 | tee "$LOG_DIR/gopro_webcam.log" &
#     sleep 10
#     ros2 run gopro_ros gopro_ros
# ) > >(tee "$LOG_DIR/gopro_ros.log") 2>&1 &

# rqt_image_view — wait for bridge to come up first
# log "Starting rqt_image_view..."
# (
#     sleep 10
#     unset ROS_DISTRO
#     source ~/ros2_humble/install/local_setup.bash
#     export ROS_DOMAIN_ID=16
#     ros2 run rqt_image_view rqt_image_view
# ) > >(tee "$LOG_DIR/rqt_image_view.log") 2>&1 &

# ros2 bag record — wait for bridge to come up first
# log "Starting ros2 bag record (waits 10 s for bridge)..."
# (
#     sleep 30
#     unset ROS_DISTRO
#     source ~/ros2_humble/install/local_setup.bash
#     export ROS_DOMAIN_ID=16
#     log "Recording bag to $LOG_DIR/bag"
#     ros2 bag record -a -o "$LOG_DIR/bag"
# ) > >(tee "$LOG_DIR/bag.log") 2>&1 &

wait
