#!/bin/bash
#
# start_mission.sh
#
# Brings up CSI cam + dji_sdk + synchronized waypoint runner, gates on
# readiness, starts rosbag, then hands off to roslaunch.
#
# Usage: start_mission.sh <waypoints_file> [speed] [arrive_radius]
#
# Optional env overrides:
#   ROSCORE_WAIT NODE_WAIT TOPIC_WAIT SERVICE_WAIT PROC_DIE_WAIT (seconds)

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CATKIN_WS="$(cd "$_PKG_DIR/../.." && pwd)"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)

if mkdir -p "/media/drone/extreme/logs/$TIMESTAMP" 2>/dev/null; then
    LOG_DIR=/media/drone/extreme/logs/$TIMESTAMP
else
    LOG_DIR=~/logs/$TIMESTAMP
    mkdir -p "$LOG_DIR"
    echo "WARNING: /media/drone/extreme not writable - logging to $LOG_DIR"
fi

MAIN_LOG="$LOG_DIR/start_mission_camera.log"
log() { local s; s="[$(date +%H:%M:%S)] $*"; echo "$s" | tee -a "$MAIN_LOG"; }

ROSCORE_WAIT="${ROSCORE_WAIT:-30}"
NODE_WAIT="${NODE_WAIT:-30}"
TOPIC_WAIT="${TOPIC_WAIT:-30}"
SERVICE_WAIT="${SERVICE_WAIT:-30}"
PROC_DIE_WAIT="${PROC_DIE_WAIT:-10}"

# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------
if [ -z "${1:-}" ]; then
    echo "Usage: $0 <waypoints_file> [speed] [arrive_radius]"
    exit 1
fi

WAYPOINTS_FILE="$(realpath "$1")"
SPEED="${2:-2.0}"
ARRIVE_RADIUS="${3:-1.0}"

if [ ! -f "$WAYPOINTS_FILE" ]; then
    echo "ERROR: waypoints file not found: $WAYPOINTS_FILE"
    exit 1
fi

# ---------------------------------------------------------------------------
# Source ROS environment
# ---------------------------------------------------------------------------
set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "$CATKIN_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

wait_until() {
    local desc="$1"; shift
    local timeout="$1"; shift
    local deadline=$(( $(date +%s) + timeout ))
    log "Waiting for $desc (abort if not ready within ${timeout}s)..."
    while [ "$(date +%s)" -lt "$deadline" ]; do
        if "$@" >/dev/null 2>&1; then
            log "  $desc OK"
            return 0
        fi
        sleep 0.5
    done
    log "ERROR: timed out after ${timeout}s waiting for $desc"
    return 1
}

wait_proc_dies() {
    local pattern="$1"
    local timeout="${2:-$PROC_DIE_WAIT}"
    local deadline=$(( $(date +%s) + timeout ))
    while [ "$(date +%s)" -lt "$deadline" ]; do
        pgrep -f "$pattern" >/dev/null 2>&1 || return 0
        sleep 0.3
    done
    log "WARN: '$pattern' still alive after ${timeout}s; sending SIGKILL"
    pkill -9 -f "$pattern" 2>/dev/null || true
    sleep 1
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        log "WARN: '$pattern' STILL alive after SIGKILL - giving up"
        return 1
    fi
    return 0
}

test_roscore()      { rostopic list; }
test_dji_node()     { rosnode info /dji_sdk; }
test_flight_topic() { rostopic echo -n 1 /dji_sdk/flight_status; }
test_service()      { rosservice list | grep -q "^$1\$"; }
test_waypoint_runner_node() { rosnode list 2>/dev/null | grep -q "waypoint_runner_synchronized"; }

stop_local_mission_stack() {
    log "Stopping any previous local mission stack..."

    if pgrep -f 'rosbag record'           >/dev/null 2>&1 \
    || pgrep -f 'record_rosbag_camera.sh' >/dev/null 2>&1; then
        log "  stopping previous rosbag (SIGINT so it finalises its index)..."
        pkill -INT -f 'rosbag record'           2>/dev/null || true
        pkill -INT -f 'record_rosbag_camera.sh' 2>/dev/null || true
        wait_proc_dies 'rosbag record'           5
        wait_proc_dies 'record_rosbag_camera.sh' 5
    fi

    pkill -f 'waypoint_runner_synchronized.py' 2>/dev/null || true
    pkill -f 'waypoint_runner.py'              2>/dev/null || true
    pkill -f 'roslaunch drone_bringup'         2>/dev/null || true
    pkill -f 'roslaunch dji_sdk'               2>/dev/null || true

    wait_proc_dies 'waypoint_runner_synchronized.py' 5
    wait_proc_dies 'waypoint_runner.py'              5
    wait_proc_dies 'roslaunch drone_bringup'         "$PROC_DIE_WAIT"
    wait_proc_dies 'roslaunch dji_sdk'               "$PROC_DIE_WAIT"

    if pgrep -f 'dji_sdk_node' >/dev/null 2>&1; then
        log "  killing leftover dji_sdk_node (holds /dev/ttyUSB0)..."
        pkill -f 'dji_sdk_node' 2>/dev/null || true
        wait_proc_dies 'dji_sdk_node' "$PROC_DIE_WAIT"
    fi

    log "  teardown complete"
}

cleanup() {
    trap - SIGINT SIGTERM
    log "Shutting down..."
    kill 0 2>/dev/null || true
    wait
}
trap cleanup SIGINT SIGTERM

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
log "Logs: $LOG_DIR"
log "Waypoints: $WAYPOINTS_FILE  speed: ${SPEED} m/s  arrive_radius: ${ARRIVE_RADIUS} m"

stop_local_mission_stack

log "Launching bringup_synchronized_camera.launch..."
export PYTHONUNBUFFERED=1
if command -v stdbuf >/dev/null 2>&1; then
    stdbuf -oL -eL roslaunch drone_bringup bringup_synchronized_camera.launch \
        waypoints_file:="$WAYPOINTS_FILE" \
        speed:="$SPEED" \
        arrive_radius:="$ARRIVE_RADIUS" \
        2>&1 | stdbuf -oL -eL tee -a "$LOG_DIR/bringup.log" &
else
    roslaunch drone_bringup bringup_synchronized_camera.launch \
        waypoints_file:="$WAYPOINTS_FILE" \
        speed:="$SPEED" \
        arrive_radius:="$ARRIVE_RADIUS" \
        2>&1 | tee -a "$LOG_DIR/bringup.log" &
fi
BRINGUP_PID=$!

# Gate 1: roscore alive
if ! wait_until "roscore" "$ROSCORE_WAIT" test_roscore; then
    log "FATAL: roscore never came up - aborting"
    cleanup; exit 1
fi

sleep 1
if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    log "FATAL: roslaunch exited unexpectedly - see $LOG_DIR/bringup.log"
    cleanup; exit 1
fi

# Gate 2: /dji_sdk node registered
if ! wait_until "/dji_sdk node registration" "$NODE_WAIT" test_dji_node; then
    log "FATAL: /dji_sdk never registered with master - is /dev/ttyUSB0 attached?"
    cleanup; exit 1
fi

# Gate 3: telemetry publishing (proves FC handshake)
if ! wait_until "/dji_sdk/flight_status publishing" "$TOPIC_WAIT" test_flight_topic; then
    log "FATAL: no flight_status - FC didn't activate. Check power & cable."
    cleanup; exit 1
fi

# Gate 4: mission services advertised
for svc in /dji_sdk/set_local_pos_ref \
           /dji_sdk/sdk_control_authority \
           /dji_sdk/mission_waypoint_upload \
           /dji_sdk/mission_waypoint_action \
           /dji_sdk/mission_status; do
    if ! wait_until "service $svc" "$SERVICE_WAIT" test_service "$svc"; then
        log "FATAL: service $svc never advertised"
        cleanup; exit 1
    fi
done

# Gate 5: waypoint runner registered
if ! wait_until "waypoint_runner_synchronized node" "$NODE_WAIT" test_waypoint_runner_node; then
    log "FATAL: waypoint runner never registered — see $LOG_DIR/bringup.log"
    cleanup; exit 1
fi

if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    log "FATAL: bringup roslaunch died between gates - aborting"
    cleanup; exit 1
fi

log "Starting rosbag record..."
if command -v stdbuf >/dev/null 2>&1; then
    (
        export LOG_DIR PYTHONUNBUFFERED=1 ROSBAG_NAME=mission
        "$SCRIPT_DIR/record_rosbag_camera.sh"
    ) 2>&1 | stdbuf -oL -eL tee -a "$LOG_DIR/bag.log" &
else
    (
        export LOG_DIR PYTHONUNBUFFERED=1 ROSBAG_NAME=mission
        "$SCRIPT_DIR/record_rosbag_camera.sh"
    ) 2>&1 | tee -a "$LOG_DIR/bag.log" &
fi

log "All gates passed. Waypoint runner is running inside roslaunch."
log "If you see no [INFO] lines here, open: tail -f $LOG_DIR/bringup.log"
log "Press Ctrl+C to tear down CSI cam, dji_sdk, rosbag, and the runner."
wait
