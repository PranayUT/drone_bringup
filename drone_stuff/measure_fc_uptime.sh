#!/bin/bash
#
# measure_fc_uptime.sh
#
# Launches dji_sdk.launch and times every milestone of the FC link bringup,
# in seconds from the moment roslaunch is invoked. Useful for showing the
# difference between:
#   t_svc  - when /dji_sdk/mission_waypoint_action just *exists* on the ROS
#            graph. This is the ONLY thing start_mission.sh's clear_fc_missions
#            checks before firing its STOP call. In simulation this is ~1-2 s;
#            in the real world it lands well before the FC is actually usable.
#   t_fs   - when the FC actually starts streaming /dji_sdk/flight_status.
#            Real proof the serial/USB link to the FC is up and telemetry
#            is flowing. waypoint_runner.py won't proceed until this happens.
#   t_auth - when DJI grants SDK control authority. Required for ANY mission
#            command to actually take effect on the FC.
#   t_clr  - when /dji_sdk/mission_waypoint_action action:1 returns. Same
#            call clear_fc_missions makes; we run it here AFTER everything is
#            up so we can see what dji_sdk says when called too early vs late.
#
# The race-condition window is roughly  (t_fs - t_svc)  or  (t_auth - t_svc).
# In sim that gap is ~0. In the real world it's often 5-20+ seconds.
#
# Usage:
#   ./measure_fc_uptime.sh                  # full bringup + timing
#   ./measure_fc_uptime.sh --no-clear       # skip the test clear call at end
#   TIMEOUT=120 ./measure_fc_uptime.sh      # raise overall timeout (default 60s)
#
# Requirements: no other dji_sdk_node / roscore already running. Script
# refuses to start if it sees one (would skew the measurement).

set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_PKG_DIR="$(cd "$_SCRIPT_DIR/.." && pwd)"
_DEFAULT_CATKIN_WS="$(cd "$_PKG_DIR/../.." && pwd)"
CATKIN_WS="${CATKIN_WS:-$_DEFAULT_CATKIN_WS}"
TIMEOUT="${TIMEOUT:-60}"
DO_CLEAR=1
for arg in "$@"; do
    case "$arg" in
        --no-clear) DO_CLEAR=0 ;;
        -h|--help)
            sed -n '2,30p' "$0"
            exit 0
            ;;
    esac
done

# --- guard against stale state -------------------------------------------
for p in dji_sdk_node 'roslaunch dji_sdk' rosmaster; do
    if pgrep -f "$p" >/dev/null 2>&1; then
        echo "ERROR: '$p' already running. Kill it first so the timing is fair:"
        echo "  pkill -f 'roslaunch dji_sdk'; pkill -f dji_sdk_node; pkill -f rosmaster; pkill -f rosout"
        exit 1
    fi
done

# --- source ROS ----------------------------------------------------------
set +u
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

# --- timing helpers ------------------------------------------------------
START_NS=$(date +%s%N)

elapsed() {
    local now
    now=$(date +%s%N)
    awk -v s="$START_NS" -v n="$now" 'BEGIN{printf "%7.3f", (n-s)/1e9}'
}

# Named-milestone storage so we can print a clean summary at the end.
declare -A T_AT
mark() {
    local name="$1"; shift
    local e
    e=$(elapsed)
    T_AT["$name"]="$e"
    printf "[%s]  t=%ss  %-9s  %s\n" \
        "$(date +%H:%M:%S.%3N)" "$e" "$name" "$*"
}

# --- launch dji_sdk in background ----------------------------------------
LOG_FILE=$(mktemp /tmp/dji_sdk_launch.XXXXXX.log)
mark t0 "Invoking 'roslaunch dji_sdk sdk.launch'  (log -> $LOG_FILE)"

(
    set +u
    source /opt/ros/noetic/setup.bash
    source "$CATKIN_WS/devel/setup.bash"
    set -u
    exec roslaunch dji_sdk sdk.launch
) >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    trap - EXIT INT TERM
    echo
    echo "Stopping dji_sdk and roscore..."
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 1
    pkill -f 'roslaunch dji_sdk' 2>/dev/null || true
    pkill -f 'dji_sdk_node'      2>/dev/null || true
    pkill -f 'rosmaster'         2>/dev/null || true
    pkill -f 'rosout'            2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

wait_for() {
    # wait_for <name> <message> <command...>
    # Polls <command> every 50 ms until it exits 0, or TIMEOUT seconds expire.
    local name="$1" msg="$2"; shift 2
    local deadline=$(( SECONDS + TIMEOUT ))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if "$@" >/dev/null 2>&1; then
            mark "$name" "$msg"
            return 0
        fi
        sleep 0.05
    done
    mark "$name" "TIMEOUT after ${TIMEOUT}s  ($msg)"
    return 1
}

# --- t_proc : dji_sdk_node process visible -------------------------------
wait_for t_proc "dji_sdk_node process is alive (ROS launched it, but FC link may not be up yet)" \
    pgrep -f dji_sdk_node

# --- t_svc  : service appears on the ROS graph ---------------------------
wait_for t_svc "/dji_sdk/mission_waypoint_action is LISTED on the ROS graph  <-- this is the ONLY check start_mission.sh does" \
    bash -c 'rosservice list 2>/dev/null | grep -q "^/dji_sdk/mission_waypoint_action$"'

# --- t_fs   : first flight_status message --------------------------------
echo "                          (blocking on first /dji_sdk/flight_status message...)"
if timeout "$TIMEOUT" rostopic echo -n1 /dji_sdk/flight_status >/dev/null 2>&1; then
    mark t_fs "first /dji_sdk/flight_status received  <-- FC link is actually alive now"
else
    mark t_fs "TIMEOUT waiting for flight_status (FC link never came up)"
fi

# --- t_gps  : first GPS message ------------------------------------------
echo "                          (blocking on first /dji_sdk/gps_position message...)"
if timeout 30 rostopic echo -n1 /dji_sdk/gps_position >/dev/null 2>&1; then
    mark t_gps "first /dji_sdk/gps_position received"
else
    mark t_gps "TIMEOUT waiting for gps_position"
fi

# --- t_lpr  : set_local_pos_ref returns result:true ----------------------
echo "                          (calling /dji_sdk/set_local_pos_ref ...)"
if rosservice call /dji_sdk/set_local_pos_ref 2>/dev/null | grep -q 'result: True'; then
    mark t_lpr "set_local_pos_ref result:true"
else
    mark t_lpr "set_local_pos_ref did NOT return result:true (FC not ready or no GPS)"
fi

# --- t_auth : sdk_control_authority returns result:true ------------------
echo "                          (requesting SDK control authority ...)"
if rosservice call /dji_sdk/sdk_control_authority "control_enable: 1" 2>/dev/null | grep -q 'result: True'; then
    mark t_auth "sdk_control_authority result:true  <-- FC will now accept commands"
else
    mark t_auth "sdk_control_authority did NOT return result:true (FC not ready / RC not in F-mode / app key issue)"
fi

# --- t_clr  : the actual clear call --------------------------------------
if [ "$DO_CLEAR" = "1" ]; then
    echo "                          (calling /dji_sdk/mission_waypoint_action action:1, same as start_mission.sh ...)"
    OUT=$(rosservice call /dji_sdk/mission_waypoint_action "action: 1" 2>&1 || true)
    if echo "$OUT" | grep -q 'result: True'; then
        mark t_clr "mission_waypoint_action action:1 result:true (cleared)"
    else
        # extract ack_data if present
        ACK=$(echo "$OUT" | awk -F: '/ack_data/{gsub(/[^0-9]/,"",$2); print $2; exit}')
        if echo "$OUT" | grep -qi 'no waypoint mission'; then
            mark t_clr "mission_waypoint_action returned 'no waypoint mission uploaded'${ACK:+ (ack_data=$ACK)} -- harmless, nothing to clear"
        else
            mark t_clr "mission_waypoint_action returned: $(echo "$OUT" | tr '\n' ' ')"
        fi
    fi
fi

# --- summary -------------------------------------------------------------
gap_svc_to_fs=$(awk -v a="${T_AT[t_svc]:-0}" -v b="${T_AT[t_fs]:-0}"  'BEGIN{printf "%.3f", b-a}')
gap_svc_to_au=$(awk -v a="${T_AT[t_svc]:-0}" -v b="${T_AT[t_auth]:-0}" 'BEGIN{printf "%.3f", b-a}')

cat <<EOF

==============================================================================
                          FC bringup timing summary
==============================================================================
  Milestone                                                Elapsed (s)
  -------------------------------------------------------  -----------
  t0     roslaunch invoked                                 ${T_AT[t0]:-?}
  t_proc dji_sdk_node process visible                      ${T_AT[t_proc]:-?}
  t_svc  service NAME listed (what bash clear checks)      ${T_AT[t_svc]:-?}
  t_fs   first flight_status message (FC link up)          ${T_AT[t_fs]:-?}
  t_gps  first gps_position message                        ${T_AT[t_gps]:-?}
  t_lpr  set_local_pos_ref succeeded                       ${T_AT[t_lpr]:-?}
  t_auth SDK control authority granted                     ${T_AT[t_auth]:-?}
  t_clr  mission_waypoint_action action:1 returned         ${T_AT[t_clr]:-n/a}

  Race-condition window (gap between "bash sees service" and "FC ready"):
      t_fs   - t_svc  = ${gap_svc_to_fs} s
      t_auth - t_svc  = ${gap_svc_to_au} s

  start_mission.sh waits 5 s before its bash clear, then loops 5x at 1 s each
  for the service NAME to appear. Total budget: ~10 s from bringup launch.
  Any gap above ~5 s means the bash clear can fire on a half-up SDK.
==============================================================================

dji_sdk is still running. Press Ctrl-C to stop and exit.
EOF

wait
