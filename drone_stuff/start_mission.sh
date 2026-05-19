#!/bin/bash
#
# CSI / nvargus + YOLO (bench_best_v1.py --source camera):
# - Default: restarts nvargus-daemon so the camera stack is clean before ROS. That kills any
#   other app already using nvarguscamerasrc (e.g. a running YOLO bench). Stop those first.
# - Only one consumer should open the same CSI sensor at a time (ROS camera XOR direct GStreamer).
#   If drone_bringup enables jetson_csi_cam, do not run bench in parallel on CSI; use a ROS image topic instead.
# - Suggested order: run this script first, wait until bringup is up, then start extras that need CSI
#   only if bringup does not own the camera (see bringup.launch).
# - To skip the daemon bounce (e.g. you already reset nvargus and no stale state): SKIP_NVARGUS_RESTART=1 $0 ...
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$_PKG_DIR/../.." && pwd)"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
if mkdir -p "/media/drone/extreme1/logs/$TIMESTAMP" 2>/dev/null; then
    LOG_DIR=/media/drone/extreme1/logs/$TIMESTAMP
else
    LOG_DIR=~/logs/$TIMESTAMP
    mkdir -p "$LOG_DIR"
    echo "WARNING: /media/drone/extreme1 not writable — logging to $LOG_DIR"
fi

log() { echo "[$(date +%H:%M:%S)] $*"; }

stop_local_mission_stack() {
    log "Stopping any previous local mission stack..."
    if pgrep -f 'record_rosbag.sh' >/dev/null 2>&1; then
        log "Stopping previous rosbag recording..."
        pkill -INT -f 'rosbag record' 2>/dev/null || true
        pkill -INT -f 'record_rosbag.sh' 2>/dev/null || true
        sleep 2
    fi
    pkill -f 'roslaunch drone_bringup bringup.launch' 2>/dev/null || true
    pkill -f 'waypoint_runner.py' 2>/dev/null || true
    sleep 1
}

clear_fc_missions() {
    log "Clearing waypoint/hotpoint missions on the flight controller..."
    (
        source /opt/ros/noetic/setup.bash
        source "$CATKIN_WS/devel/setup.bash"
        export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
        if ! rostopic list >/dev/null 2>&1; then
            log "ROS not ready; skipping FC mission clear"
            exit 0
        fi
        for _ in 1 2 3 4 5; do
            if rosservice list 2>/dev/null | grep -q '^/dji_sdk/mission_waypoint_action$'; then
                break
            fi
            sleep 1
        done
        rosservice call /dji_sdk/mission_waypoint_action "action: 1" >/dev/null 2>&1 || true
        rosservice call /dji_sdk/mission_hotpoint_action "action: 1" >/dev/null 2>&1 || true
        sleep 2
    )
}

if [ -z "$1" ]; then
    echo "Usage: $0 <waypoints_file> [speed] [arrive_radius]"
    echo "Optional env: SKIP_NVARGUS_RESTART=1  (do not restart nvargus-daemon before bringup)"
    exit 1
fi

WAYPOINTS_FILE="$(realpath "$1")"
SPEED="${2:-5.0}"
ARRIVE_RADIUS="${3:-1.0}"

log "Logs: $LOG_DIR"
log "Waypoints: $WAYPOINTS_FILE  speed: ${SPEED} m/s  arrive_radius: ${ARRIVE_RADIUS} m"

cleanup() {
    trap - SIGINT SIGTERM
    log "Shutting down..."
    kill 0
    wait
}
trap cleanup SIGINT SIGTERM

stop_local_mission_stack

# ROS1 bringup (camera + dji_sdk + waypoint_runner)
# NVARGUS_SETTLE_SEC="${NVARGUS_SETTLE_SEC:-3}"
# if [ "${SKIP_NVARGUS_RESTART:-0}" = "1" ]; then
#     log "SKIP_NVARGUS_RESTART=1 — not restarting nvargus-daemon (ensure no stale CSI state)."
# else
#     log "Restarting nvargus-daemon..."
#     sudo systemctl restart nvargus-daemon 2>&1 | tee "$LOG_DIR/nvargus.log"
#     log "Waiting ${NVARGUS_SETTLE_SEC}s for nvargus to settle..."
#     sleep "$NVARGUS_SETTLE_SEC"
# fi

# ROS1 bringup (camera + dji_sdk + waypoint_runner)
log "Launching drone_bringup..."
(
    source /opt/ros/noetic/setup.bash
    source "$CATKIN_WS/devel/setup.bash"
    roslaunch drone_bringup bringup.launch \
        waypoints_file:="$WAYPOINTS_FILE" \
        speed:="$SPEED" \
        arrive_radius:="$ARRIVE_RADIUS"
) > >(tee "$LOG_DIR/bringup.log") 2>&1 &

# Wait for roscore to be ready
sleep 5

clear_fc_missions

log "Starting rosbag record..."
(
    export LOG_DIR
    "$SCRIPT_DIR/record_rosbag.sh"
) > >(tee "$LOG_DIR/bag.log") 2>&1 &

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
