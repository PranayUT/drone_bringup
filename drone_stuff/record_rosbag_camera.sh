#!/bin/bash
#
# Experimental rosbag helper for CSI camera tests (does not replace record_rosbag.sh).
# Usage:
#   LOG_DIR=/path/to/logs ./record_rosbag_camera.sh [topic ...]
# Optional env:
#   ROSBAG_WAIT_SECS=5
#   ROSBAG_NAME=camera_test
#   ROSBAG_TOPICS="..."    space-separated topic list (overrides default -a)
#   CATKIN_WS=/path/to/catkin_ws
# With no ROSBAG_TOPICS and no CLI topics, records all topics (-a).
#

set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_PKG_DIR="$(cd "$_SCRIPT_DIR/.." && pwd)"
_DEFAULT_CATKIN_WS="$(cd "$_PKG_DIR/../.." && pwd)"
CATKIN_WS="${CATKIN_WS:-$_DEFAULT_CATKIN_WS}"
ROSBAG_WAIT_SECS="${ROSBAG_WAIT_SECS:-5}"
ROSBAG_NAME="${ROSBAG_NAME:-camera_test}"

log() { echo "[$(date +%H:%M:%S)] $*"; }

if [ -z "${LOG_DIR:-}" ]; then
    STAMP=$(date +%Y-%m-%d_%H-%M-%S)
    LOG_DIR="${DRONE_MISSION_LOG_DIR:-/media/drone/extreme/logs/camera_$STAMP}"
    if ! mkdir -p "$LOG_DIR" 2>/dev/null; then
        LOG_DIR="$HOME/.drone_logs/camera_$STAMP"
        mkdir -p "$LOG_DIR"
        log "External log drive unavailable; using $LOG_DIR"
    fi
fi

mkdir -p "$LOG_DIR"

set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "$CATKIN_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

log "Waiting ${ROSBAG_WAIT_SECS}s for ROS topics..."
sleep "$ROSBAG_WAIT_SECS"

if ! rostopic list >/dev/null 2>&1; then
    log "WARNING: roscore not reachable at $ROS_MASTER_URI; starting record anyway"
fi

BAG_PATH="$LOG_DIR/$ROSBAG_NAME"
if [ "$#" -gt 0 ]; then
    log "Recording topics to ${BAG_PATH}.bag: $*"
    exec rosbag record -O "$BAG_PATH" "$@"
fi

if [ -n "${ROSBAG_TOPICS:-}" ]; then
    # shellcheck disable=SC2086
    log "Recording ROSBAG_TOPICS to ${BAG_PATH}.bag: $ROSBAG_TOPICS"
    exec rosbag record -O "$BAG_PATH" $ROSBAG_TOPICS
fi

log "Recording all topics to ${BAG_PATH}.bag"
exec rosbag record -a -O "$BAG_PATH"
