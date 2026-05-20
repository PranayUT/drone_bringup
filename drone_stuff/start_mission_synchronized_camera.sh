#!/bin/bash
#
# start_mission_synchronized_camera.sh  (EXPERIMENTAL — does not replace start_mission_synchronized.sh)
#
# Same gates as start_mission_synchronized.sh, but uses bringup_synchronized_camera.launch
# (dji_sdk + waypoint runner + jetson_csi_cam). Test with test_camera_record.sh first.
#
# Same purpose as start_mission.sh, but every blind `sleep` is replaced by a
# poll-until-ready gate. The "clear old mission" step lives entirely inside
# waypoint_runner_synchronized.py (drain SDK → dummy prime → PAUSE/STOP disarm
# with bounded budget), because dji_sdk_node's local-state guard makes a shell-side
# STOP a no-op against the FC when called from a fresh process.
#
# Launch architecture: a single roslaunch of
# drone_bringup/bringup_synchronized_camera.launch brings up CSI cam + dji_sdk +
# synchronized waypoint runner together. This script's job is to:
#
#   1. Tear down anything left over from a previous run (and wait for it).
#   2. Fire the bringup_synchronized.launch in the background.
#   3. Gate on readiness (roscore alive -> /dji_sdk node -> /dji_sdk/flight_status
#      publishing -> mission services advertised) so we fail fast if the FC
#      didn't connect, rather than letting the runner spin forever.
#   4. Start rosbag recording.
#   5. Wait. The synchronized runner inside the launch performs:
#        CLEAR WAYPOINT STACK (drain → dummy upload → PAUSE/STOP disarm)
#        UPLOAD NEW MISSION (retries on transient FC acks)
#        START MISSION
#      and prints progress to the same terminal via roslaunch output="screen".
#
# Usage: start_mission_synchronized_camera.sh [clear_anchor_waypoints] [speed] [arrive_radius]
#
# Dynamic mission (default USE_DYNAMIC_MISSION=1):
#   Dynamic_Waypoint/dynamic_mission.txt — created before or after takeoff.
#   Runner clears FC, waits until that file exists, then uploads it.
#   Bootstrap waypoints (for clear only) default to Dynamic_Waypoint/clear_anchor_waypoints.txt
#

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

# Tunables (seconds). Each gate polls until success OR this max elapses — then
# the script aborts with FATAL (it never proceeds to the next gate on timeout).
# Override at runtime, e.g. SERVICE_WAIT=120 ./start_mission_synchronized.sh ...
ROSCORE_WAIT="${ROSCORE_WAIT:-30}"
NODE_WAIT="${NODE_WAIT:-30}"
TOPIC_WAIT="${TOPIC_WAIT:-30}"
SERVICE_WAIT="${SERVICE_WAIT:-30}"
PROC_DIE_WAIT="${PROC_DIE_WAIT:-10}"

# ---------------------------------------------------------------------------
# Dynamic waypoint directory (single mission file: dynamic_mission.txt)
# ---------------------------------------------------------------------------
DYNAMIC_WAYPOINT_DIR="$SCRIPT_DIR/Dynamic_Waypoint"
DYNAMIC_MISSION_FILE="$DYNAMIC_WAYPOINT_DIR/dynamic_mission.txt"
CLEAR_ANCHOR_DEFAULT="$DYNAMIC_WAYPOINT_DIR/clear_anchor_waypoints.txt"
USE_DYNAMIC_MISSION="${USE_DYNAMIC_MISSION:-1}"

mkdir -p "$DYNAMIC_WAYPOINT_DIR"

ensure_single_mission_in_dynamic_dir() {
    local dir="$1"
    local mission_count=0
    local extra=""
    shopt -s nullglob
    for f in "$dir"/*.txt; do
        local base
        base="$(basename "$f")"
        case "$base" in
            clear_anchor_waypoints.txt) continue ;;
            dynamic_mission.txt)
                mission_count=$((mission_count + 1))
                ;;
            *)
                extra="${extra} ${base}"
                mission_count=$((mission_count + 1))
                ;;
        esac
    done
    shopt -u nullglob
    if [ -n "$extra" ]; then
        log "WARN: unexpected .txt in Dynamic_Waypoint (only dynamic_mission.txt should be the mission):$extra"
    fi
    if [ "$mission_count" -gt 1 ]; then
        log "ERROR: multiple mission .txt files in $dir — only one mission allowed"
        return 1
    fi
    return 0
}

wait_for_dynamic_mission_file() {
    local path="$1"
    local interval="${DYNAMIC_MISSION_POLL_SEC:-5}"
    log "Waiting for dynamic mission file (post-FC-clear upload source): $path"
    while [ ! -f "$path" ]; do
        log "  ... still waiting for $path to exist (create with Dynamic_Waypoint/create_dynamic_mission.sh)"
        sleep "$interval"
    done
    log "Dynamic mission file ready: $path"
}

# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------
if [ "$USE_DYNAMIC_MISSION" = "1" ]; then
    if [ -n "${1:-}" ] && [ -f "$1" ]; then
        WAYPOINTS_FILE="$(realpath "$1")"
        SPEED="${2:-2.0}"
        ARRIVE_RADIUS="${3:-1.0}"
    else
        WAYPOINTS_FILE="$(realpath "$CLEAR_ANCHOR_DEFAULT")"
        SPEED="${1:-2.0}"
        ARRIVE_RADIUS="${2:-1.0}"
    fi
    DYNAMIC_MISSION_ARG="$DYNAMIC_MISSION_FILE"
else
    if [ -z "${1:-}" ]; then
        echo "Usage: $0 <waypoints_file> [speed] [arrive_radius]"
        echo ""
        echo "Experimental: synchronized mission + CSI camera in rosbag (-a)."
        echo "Does not modify start_mission_synchronized.sh."
        echo ""
        echo "Dynamic mission (default on): set USE_DYNAMIC_MISSION=1 (default)"
        echo "  Bootstrap: Dynamic_Waypoint/clear_anchor_waypoints.txt"
        echo "  Mission:   Dynamic_Waypoint/dynamic_mission.txt (waited for after FC clear)"
        echo ""
        echo "Optional env:"
        echo "  USE_DYNAMIC_MISSION=0   upload only from CLI waypoints file"
        echo "  SKIP_NVARGUS_RESTART=1  skip nvargus-daemon restart before bringup"
        echo "  ROSCORE_WAIT NODE_WAIT TOPIC_WAIT SERVICE_WAIT PROC_DIE_WAIT"
        exit 1
    fi
    WAYPOINTS_FILE="$(realpath "$1")"
    SPEED="${2:-2.0}"
    ARRIVE_RADIUS="${3:-1.0}"
    DYNAMIC_MISSION_ARG="__none__"
fi

if [ ! -f "$WAYPOINTS_FILE" ]; then
    echo "ERROR: waypoints file not found: $WAYPOINTS_FILE"
    exit 1
fi

# ---------------------------------------------------------------------------
# Source ROS environment once for the parent shell
# (ROS profile scripts reference vars like $ROS_DISTRO that aren't yet set, so
# we temporarily relax `set -u` around the sources.)
# ---------------------------------------------------------------------------
set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "$CATKIN_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

# ---------------------------------------------------------------------------
# Generic poll-until helper
#   wait_until <description> <timeout-sec> <shell-test-command...>
# ---------------------------------------------------------------------------
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

# Wrappers usable by wait_until
test_roscore()      { rostopic list; }
test_dji_node()     { rosnode info /dji_sdk; }
test_flight_topic() { rostopic echo -n 1 /dji_sdk/flight_status; }
test_service()      { rosservice list | grep -q "^$1\$"; }
test_waypoint_runner_node() { rosnode list 2>/dev/null | grep -q "waypoint_runner_synchronized"; }

# ---------------------------------------------------------------------------
# Tear down anything from a previous run, and wait for it to actually die
# ---------------------------------------------------------------------------
stop_local_mission_stack() {
    log "Stopping any previous local mission stack..."

    # rosbag: SIGINT first so it finalises its index, then enforce.
    if pgrep -f 'rosbag record'           >/dev/null 2>&1 \
    || pgrep -f 'record_rosbag_camera.sh' >/dev/null 2>&1; then
        log "  stopping previous rosbag..."
        pkill -INT -f 'rosbag record'           2>/dev/null || true
        pkill -INT -f 'record_rosbag_camera.sh' 2>/dev/null || true
        wait_proc_dies 'rosbag record'           5
        wait_proc_dies 'record_rosbag_camera.sh' 5
    fi

    # Runners and bringup
    pkill -f 'waypoint_runner_synchronized.py' 2>/dev/null || true
    pkill -f 'waypoint_runner.py'              2>/dev/null || true
    pkill -f 'roslaunch drone_bringup'         2>/dev/null || true
    pkill -f 'roslaunch dji_sdk'               2>/dev/null || true

    wait_proc_dies 'waypoint_runner_synchronized.py' 5
    wait_proc_dies 'waypoint_runner.py'              5
    wait_proc_dies 'roslaunch drone_bringup'         "$PROC_DIE_WAIT"
    wait_proc_dies 'roslaunch dji_sdk'               "$PROC_DIE_WAIT"

    # dji_sdk_node holds /dev/ttyUSB0 - the new instance will fail to open
    # the serial port if the old one is still around. Wait it out, then kill.
    if pgrep -f 'dji_sdk_node' >/dev/null 2>&1; then
        log "  killing leftover dji_sdk_node (holds /dev/ttyUSB0)..."
        pkill -f 'dji_sdk_node' 2>/dev/null || true
        wait_proc_dies 'dji_sdk_node' "$PROC_DIE_WAIT"
    fi

    # if pgrep -f 'gscam' >/dev/null 2>&1; then
    #     log "  stopping gscam / CSI camera..."
    #     pkill -f 'gscam' 2>/dev/null || true
    #     pkill -f 'csi_cam_' 2>/dev/null || true
    #     wait_proc_dies 'gscam' 5
    # fi

    log "  teardown complete"
}

# ---------------------------------------------------------------------------
# Trap: clean exit on Ctrl+C / SIGTERM
# ---------------------------------------------------------------------------
cleanup() {
    trap - SIGINT SIGTERM
    log "Shutting down..."
    # Send SIGTERM to the whole process group; `wait` below will reap.
    kill 0 2>/dev/null || true
    wait
}
trap cleanup SIGINT SIGTERM

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
log "Logs: $LOG_DIR"
log "Waypoints (bootstrap/clear): $WAYPOINTS_FILE  speed: ${SPEED} m/s  arrive_radius: ${ARRIVE_RADIUS} m"
if [ "$USE_DYNAMIC_MISSION" = "1" ]; then
    if ! ensure_single_mission_in_dynamic_dir "$DYNAMIC_WAYPOINT_DIR"; then
        exit 1
    fi
    log "Dynamic mission enabled: $DYNAMIC_MISSION_FILE"
    log "  After FC clear, runner waits for that file then uploads it (see bringup.log)"
    if [ -f "$DYNAMIC_MISSION_FILE" ]; then
        log "  (file already present — no wait needed after clear)"
    else
        log "  (file not present yet — runner will log 'still waiting' until created)"
    fi
fi

stop_local_mission_stack

if [ "${SKIP_NVARGUS_RESTART:-0}" != "1" ]; then
    log "Restarting nvargus-daemon for CSI camera..."
    sudo systemctl restart nvargus-daemon 2>&1 | tee -a "$LOG_DIR/nvargus.log" || true
    sleep "${NVARGUS_SETTLE_SEC:-3}"
else
    log "SKIP_NVARGUS_RESTART=1 — not restarting nvargus-daemon"
fi

log "Launching bringup_synchronized_camera.launch..."
# roslaunch + Python nodes often fully-buffer stdout when piped; that left
# bringup.log at 0 bytes and made the mission look "stuck" with no [INFO] lines.
# Line-buffer through stdbuf; PYTHONUNBUFFERED helps rospy children.
export PYTHONUNBUFFERED=1
if command -v stdbuf >/dev/null 2>&1; then
    stdbuf -oL -eL roslaunch drone_bringup bringup_synchronized_camera.launch \
        waypoints_file:="$WAYPOINTS_FILE" \
        speed:="$SPEED" \
        arrive_radius:="$ARRIVE_RADIUS" \
        dynamic_mission_file:="$DYNAMIC_MISSION_ARG" \
        2>&1 | stdbuf -oL -eL tee -a "$LOG_DIR/bringup.log" &
else
    roslaunch drone_bringup bringup_synchronized_camera.launch \
        waypoints_file:="$WAYPOINTS_FILE" \
        speed:="$SPEED" \
        arrive_radius:="$ARRIVE_RADIUS" \
        dynamic_mission_file:="$DYNAMIC_MISSION_ARG" \
        2>&1 | tee -a "$LOG_DIR/bringup.log" &
fi
BRINGUP_PID=$!

# Gate 1: roscore is alive
if ! wait_until "roscore" "$ROSCORE_WAIT" test_roscore; then
    log "FATAL: roscore never came up - aborting"
    cleanup
    exit 1
fi

# Liveness check on the backgrounded roslaunch BEFORE we wait for the node:
# if dji_sdk_node crashed during activation, roslaunch may have already exited.
sleep 1
if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    log "FATAL: roslaunch exited unexpectedly - see $LOG_DIR/bringup.log"
    cleanup
    exit 1
fi

# Gate 2: /dji_sdk node registered with master
if ! wait_until "/dji_sdk node registration" "$NODE_WAIT" test_dji_node; then
    log "FATAL: /dji_sdk never registered with master - is /dev/ttyUSB0 attached?"
    cleanup
    exit 1
fi

# Gate 3: telemetry is actually publishing (proves FC handshake worked)
if ! wait_until "/dji_sdk/flight_status publishing" "$TOPIC_WAIT" test_flight_topic; then
    log "FATAL: no flight_status - FC didn't activate. Check power & cable."
    cleanup
    exit 1
fi

# Gate 4: every service the runner needs is advertised
for svc in /dji_sdk/set_local_pos_ref \
           /dji_sdk/sdk_control_authority \
           /dji_sdk/mission_waypoint_upload \
           /dji_sdk/mission_waypoint_action \
           /dji_sdk/mission_status; do
    if ! wait_until "service $svc" "$SERVICE_WAIT" test_service "$svc"; then
        log "FATAL: service $svc never advertised"
        cleanup
        exit 1
    fi
done

# Gate 5: waypoint runner registered (anonymous node name still contains this substring)
if ! wait_until "waypoint_runner_synchronized node" "$NODE_WAIT" test_waypoint_runner_node; then
    log "FATAL: waypoint runner never registered — see $LOG_DIR/bringup.log"
    cleanup
    exit 1
fi

# Optional shell-side wait (runner also waits after FC clear; this logs to start_mission_camera.log)
if [ "$USE_DYNAMIC_MISSION" = "1" ] && [ ! -f "$DYNAMIC_MISSION_FILE" ]; then
    log "Pre-flight: dynamic_mission.txt not on disk yet (runner will wait again after FC clear)"
    if [ "${SHELL_WAIT_FOR_DYNAMIC:-0}" = "1" ]; then
        wait_for_dynamic_mission_file "$DYNAMIC_MISSION_FILE" || exit 1
    fi
fi

# Liveness re-check just before starting rosbag
if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    log "FATAL: bringup roslaunch died between gates - aborting"
    cleanup
    exit 1
fi

# rosbag recorder - safe to start now that the FC is talking
log "Starting rosbag record..."
if command -v stdbuf >/dev/null 2>&1; then
    (
        export LOG_DIR
        export PYTHONUNBUFFERED=1
        export ROSBAG_NAME=mission
        "$SCRIPT_DIR/record_rosbag_camera.sh"
    ) 2>&1 | stdbuf -oL -eL tee -a "$LOG_DIR/bag.log" &
else
    (
        export LOG_DIR
        export PYTHONUNBUFFERED=1
        export ROSBAG_NAME=mission
        "$SCRIPT_DIR/record_rosbag_camera.sh"
    ) 2>&1 | tee -a "$LOG_DIR/bag.log" &
fi

# Hand off entirely to roslaunch: the synchronized runner is one of the nodes
# inside bringup_synchronized_camera.launch, and prints progress to this terminal via
# output="screen". Wait until the user Ctrl+Cs (or roslaunch dies on its own).
log "All gates passed. Waypoint runner is running inside roslaunch."
log "If you see no [INFO] lines here, open: tail -f $LOG_DIR/bringup.log"
log "Press Ctrl+C to tear down CSI cam, dji_sdk, rosbag, and the runner."
wait
