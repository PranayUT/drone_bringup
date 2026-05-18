#!/bin/bash
#
# ROS1 rosbag recording for mission logs.
# Usage:
#   LOG_DIR=/path/to/logs ./record_rosbag.sh [topic ...]
# Optional env:
#   ROSBAG_WAIT_SECS=5   wait before recording (default 5)
#   ROSBAG_NAME=mission    bag basename under LOG_DIR (default mission -> mission.bag)
#   CATKIN_WS=~/Documents/catkin_ws
# With no topics, records all topics (-a).
#

set -euo pipefail

CATKIN_WS="${CATKIN_WS:-$HOME/Documents/catkin_ws}"
ROSBAG_WAIT_SECS="${ROSBAG_WAIT_SECS:-5}"
ROSBAG_NAME="${ROSBAG_NAME:-mission}"

log() { echo "[$(date +%H:%M:%S)] $*"; }

if [ -z "${LOG_DIR:-}" ]; then
    STAMP=$(date +%Y-%m-%d_%H-%M-%S)
    LOG_DIR="${DRONE_MISSION_LOG_DIR:-/media/drone/extreme/logs/$STAMP}"
    if ! mkdir -p "$LOG_DIR" 2>/dev/null; then
        LOG_DIR="$HOME/.drone_logs/$STAMP"
        mkdir -p "$LOG_DIR"
        log "External log drive unavailable; using $LOG_DIR"
    fi
fi

mkdir -p "$LOG_DIR"

# ROS setup scripts reference vars that may be unset; `set -u` would abort.
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

log "Recording all topics to ${BAG_PATH}.bag"
exec rosbag record -a -O "$BAG_PATH"
