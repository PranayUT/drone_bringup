#!/bin/bash
#
# Standalone CSI camera + rosbag test (no mission / no dji_sdk).
# Confirms jetson_csi_cam topics exist and land in a bag before enabling camera on
# bringup_synchronized / start_mission_synchronized.sh.
#
# Usage:
#   ./test_camera_record.sh [seconds]
#
# Optional env:
#   RECORD_SECS=30          duration (default 30, or first positional arg)
#   SKIP_NVARGUS_RESTART=1  skip nvargus-daemon restart
#   CAMERA_WIDTH HEIGHT FPS passed to csi_camera_only.launch
#   LOG_DIR=/path/to/logs     output directory (default: extreme1 or ~/logs)
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$_PKG_DIR/../.." && pwd)"
RECORD_SECS="${1:-${RECORD_SECS:-30}}"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)

CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
CAMERA_FPS="${CAMERA_FPS:-30}"
IMAGE_TOPIC="/csi_cam_0/image_raw"

if [ -z "${LOG_DIR:-}" ]; then
    if mkdir -p "/media/drone/extreme1/logs/camera_test_$TIMESTAMP" 2>/dev/null; then
        LOG_DIR="/media/drone/extreme1/logs/camera_test_$TIMESTAMP"
    else
        LOG_DIR="$HOME/logs/camera_test_$TIMESTAMP"
        mkdir -p "$LOG_DIR"
    fi
fi

log() { echo "[$(date +%H:%M:%S)] $*"; }

cleanup() {
    trap - SIGINT SIGTERM
    log "Stopping camera test..."
    pkill -INT -f 'rosbag record' 2>/dev/null || true
    pkill -INT -f 'record_rosbag_camera.sh' 2>/dev/null || true
    pkill -f 'roslaunch drone_bringup csi_camera_only' 2>/dev/null || true
    pkill -f 'gscam' 2>/dev/null || true
    sleep 1
}
trap cleanup SIGINT SIGTERM

set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "$CATKIN_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

log "Log dir: $LOG_DIR"
log "Recording ${RECORD_SECS}s from $IMAGE_TOPIC (${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS})"

if [ "${SKIP_NVARGUS_RESTART:-0}" != "1" ]; then
    log "Restarting nvargus-daemon..."
    sudo systemctl restart nvargus-daemon 2>&1 | tee -a "$LOG_DIR/nvargus.log" || true
    sleep "${NVARGUS_SETTLE_SEC:-3}"
fi

log "Launching csi_camera_only.launch..."
roslaunch drone_bringup csi_camera_only.launch \
    width:="$CAMERA_WIDTH" height:="$CAMERA_HEIGHT" fps:="$CAMERA_FPS" \
    > >(tee -a "$LOG_DIR/camera.log") 2>&1 &
CAM_PID=$!

wait_for_camera() {
    # rostopic hz in a pipe can hang; one message is enough.
    timeout 8 rostopic echo -n 1 "$IMAGE_TOPIC" >/dev/null 2>&1
}

deadline=$(( $(date +%s) + 60 ))
log "Waiting for $IMAGE_TOPIC (up to 60s)..."
while [ "$(date +%s)" -lt "$deadline" ]; do
    if wait_for_camera; then
        log "  camera publishing"
        break
    fi
    if ! kill -0 "$CAM_PID" 2>/dev/null; then
        log "ERROR: camera launch exited — see $LOG_DIR/camera.log"
        exit 1
    fi
    sleep 2
done
if ! wait_for_camera; then
    log "ERROR: $IMAGE_TOPIC not publishing — see $LOG_DIR/camera.log"
    cleanup
    exit 1
fi

export LOG_DIR
export ROSBAG_NAME=camera_test
export ROSBAG_WAIT_SECS=2
# Mission topics + CSI; skip -a so this test bag stays small and easy to inspect.
export ROSBAG_TOPICS="$IMAGE_TOPIC /csi_cam_0/camera_info /csi_cam_0/image_raw/compressed"

log "Starting rosbag ($ROSBAG_TOPICS)..."
"$SCRIPT_DIR/record_rosbag_camera.sh" > >(tee -a "$LOG_DIR/bag.log") 2>&1 &
BAG_PID=$!

log "Recording ${RECORD_SECS}s..."
sleep "$RECORD_SECS"

log "Done. Finalizing bag..."
kill -INT "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true
cleanup

BAG="$LOG_DIR/camera_test.bag"
if [ -f "$BAG" ]; then
    log "Bag written: $BAG"
    rosbag info "$BAG" | sed -n '/topics:/,$p' | head -20
else
    log "WARN: expected $BAG — check $LOG_DIR/bag.log (maybe only .bag.active)"
    ls -la "$LOG_DIR"/*.bag* 2>/dev/null || true
fi
