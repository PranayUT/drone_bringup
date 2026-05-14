#!/usr/bin/env python3
"""Synchronized DJI waypoint mission runner.

Differences vs waypoint_runner.py:
- Every wait is poll-until-ready (no fixed sleeps).
- Explicit "clear old mission" step: a dummy 2-WP upload primes the
  dji_sdk_node local state so the subsequent ACTION_STOP actually reaches
  the FC (the dji_sdk service handler refuses to send STOP when its local
  wpMissionVector is empty — that's why a bare STOP from a fresh process
  was a no-op against the FC).
- set_local_pos_ref, sdk_control_authority, and waypoint upload all retry
  with operator-readable status messages instead of aborting on first fail.
- Upload retry loop handles transient FC acks (209 NEED_OBTAIN_CONTROL,
  213 / 236 IN_PROGRESS, 234 DATA_NOT_ENOUGH) by re-acquiring control or sending
  ACTION_STOP between attempts.

Sequence (the user's "rough draft" mental model, mapped to the code):
    parse waypoints
    wait for telemetry + services        (poll)
    set_local_pos_ref                    (retry)
    sdk_control_authority                (retry, RC must be in F-mode)
    CLEAR OLD MISSION                    (dummy upload -> settle -> double ACTION_STOP)
    UPLOAD NEW MISSION                   (retry-with-STOP on transient acks)
    START MISSION
    track waypoint arrivals

Waypoints file: same format as waypoint_runner.py — 'lat, lon, alt[, yaw_rad]'.
"""

import argparse
import math
import sys
import time

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8
import dji_sdk.msg as dji_msg
import dji_sdk.srv as dji_srv


# DJI FlightStatus values (VehicleStatus::FlightStatus)
FLIGHT_STATUS_ON_GROUND = 1
FLIGHT_STATUS_IN_AIR    = 2

# Defaults
DEFAULT_ARRIVE_RADIUS_M = 1.0
DEFAULT_IDLE_SPEED      = 5.0

# Sync tunables
TELEMETRY_WAIT_SEC       = 60.0
SERVICE_WAIT_SEC         = 30.0
SET_LOCAL_POS_RETRIES    = 5
SET_LOCAL_POS_DELAY_SEC  = 2.0
CTRL_AUTH_RETRIES        = 30
CTRL_AUTH_DELAY_SEC      = 2.0
UPLOAD_RETRY_ATTEMPTS    = 5
UPLOAD_RETRY_DELAY_SEC   = 3.0
MISSION_STOP_SETTLE_SEC  = 2.0

# Clear: retry dummy upload on ServiceException until CLEAR_MISSION_TIMEOUT_SEC;
# then fixed double ACTION_STOP + settle (proven on FC). Upload retries handle residual busyness.
CLEAR_MISSION_TIMEOUT_SEC   = 120.0
CLEAR_STOP_POLL_DELAY_SEC   = 0.5
CLEAR_POST_DUMMY_SETTLE_SEC = 2.0   # FC/SDK busy right after waypoint upload burst

# DJI mission ack codes (see Onboard-SDK-3.8.1/osdk-core/api/src/dji_error.cpp)
ACK_NEED_OBTAIN_CONTROL  = 209   # MissionACK::Common::NEED_OBTAIN_CONTROL (0xD1)
ACK_MISSION_IN_PROGRESS  = 213   # MissionACK::Common::IN_PROGRESS        (0xD5)
ACK_DATA_NOT_ENOUGH      = 234   # MissionACK::WayPoint::DATA_NOT_ENOUGH  (0xEA)
ACK_WAYPOINT_IN_PROGRESS = 236   # MissionACK::WayPoint::IN_PROGRESS      (0xEC)
ACK_WAYPOINT_NOT_IN_PROGRESS = 237  # MissionACK::WayPoint::NOT_IN_PROGRESS (0xED)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two WGS84 points."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def yaw_rad_to_dji_deg(yaw_rad):
    """Local-frame yaw (radians, 0 = south) -> DJI target_yaw (degrees, 0 = north, +CW, (-180, 180])."""
    deg = 180.0 - math.degrees(yaw_rad)
    return (deg + 180.0) % 360.0 - 180.0


def parse_waypoints(path):
    """Return list of (lat, lon, alt, yaw_rad_or_None) tuples from a text file.

    Accepts 3-column 'lat, lon, alt' or 4-column 'lat, lon, alt, yaw_rad'.
    """
    waypoints = []
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) not in (3, 4):
                rospy.logwarn("Line %d: expected 'lat,lon,alt[,yaw_rad]' - skipping: %s",
                              lineno, raw.rstrip())
                continue
            try:
                lat = float(parts[0])
                lon = float(parts[1])
                alt = float(parts[2])
                yaw = float(parts[3]) if len(parts) == 4 else None
                waypoints.append((lat, lon, alt, yaw))
            except ValueError:
                rospy.logwarn("Line %d: could not parse floats - skipping: %s", lineno, raw.rstrip())
    return waypoints


def build_waypoint_task(waypoints, idle_speed):
    """Build a dji_sdk/MissionWaypointTask. Uses YAW_MODE_CONTROLLED iff any wp has a yaw value."""
    has_yaw = any(yaw is not None for _, _, _, yaw in waypoints)

    task = dji_msg.MissionWaypointTask()
    task.velocity_range     = 15.0
    task.idle_velocity      = float(idle_speed)
    task.action_on_finish   = dji_msg.MissionWaypointTask.FINISH_NO_ACTION
    task.mission_exec_times = 1
    task.yaw_mode           = (dji_msg.MissionWaypointTask.YAW_MODE_CONTROLLED
                               if has_yaw else dji_msg.MissionWaypointTask.YAW_MODE_AUTO)
    task.trace_mode         = dji_msg.MissionWaypointTask.TRACE_POINT
    task.action_on_rc_lost  = dji_msg.MissionWaypointTask.ACTION_AUTO
    task.gimbal_pitch_mode  = dji_msg.MissionWaypointTask.GIMBAL_PITCH_FREE

    for lat, lon, alt, yaw in waypoints:
        wp = dji_msg.MissionWaypoint()
        wp.latitude            = lat
        wp.longitude           = lon
        wp.altitude            = float(alt)
        wp.damping_distance    = 0.0
        wp.target_yaw          = int(round(yaw_rad_to_dji_deg(yaw))) if yaw is not None else 0
        wp.target_gimbal_pitch = 0
        wp.turn_mode           = 0
        wp.has_action          = 0
        wp.action_time_limit   = 0
        action = dji_msg.MissionWaypointAction()
        action.action_repeat     = 0
        action.command_list      = [0] * 16
        action.command_parameter = [0] * 16
        wp.waypoint_action = action
        task.mission_waypoint.append(wp)

    return task


def build_dummy_task(anchor_lat, anchor_lon, anchor_alt, idle_speed):
    """Tiny 2-WP mission anchored at the given point. Used ONLY to populate the
    dji_sdk_node local wpMissionVector so that a subsequent ACTION_STOP actually
    reaches the FC. Never ACTION_STARTed by us — STOP follows immediately.
    The second waypoint is ~1.1 m north of the first (sufficient FC spacing)."""
    waypoints = [
        (anchor_lat,           anchor_lon, anchor_alt, None),
        (anchor_lat + 1.0e-5,  anchor_lon, anchor_alt, None),
    ]
    return build_waypoint_task(waypoints, idle_speed)


# ---------------------------------------------------------------------------
# ROS state (written by subscribers, read by main thread)
# ---------------------------------------------------------------------------

class DroneState:
    def __init__(self):
        self.flight_status = None
        self.gps_lat       = None
        self.gps_lon       = None
        self.gps_valid     = False


_state = DroneState()


def _flight_status_cb(msg):
    _state.flight_status = msg.data


def _gps_cb(msg):
    if msg.status.status >= 0:
        _state.gps_lat   = msg.latitude
        _state.gps_lon   = msg.longitude
        _state.gps_valid = True


# ---------------------------------------------------------------------------
# Poll-until-ready helpers
# ---------------------------------------------------------------------------

def wait_until(predicate, desc, timeout):
    """Poll predicate() until it returns truthy or timeout expires. Returns bool."""
    deadline = time.time() + timeout
    last_log = 0.0
    while time.time() < deadline and not rospy.is_shutdown():
        if predicate():
            return True
        now = time.time()
        if now - last_log >= 5.0:
            rospy.loginfo("  ... still waiting for %s (%.0fs remaining)",
                          desc, deadline - now)
            last_log = now
        time.sleep(0.2)
    return False


def wait_for_service(name, timeout):
    rospy.loginfo("Waiting for service %s (timeout %.0fs)...", name, timeout)
    try:
        rospy.wait_for_service(name, timeout=timeout)
        rospy.loginfo("  %s ready", name)
        return True
    except rospy.ROSException:
        rospy.logfatal("Service %s did not appear after %.0fs.", name, timeout)
        return False


def call_service_with_retry(proxy, request, desc, n_retries, delay, hint=None):
    """Call a service repeatedly until resp.result is True or retries exhausted."""
    last_resp = None
    for attempt in range(1, n_retries + 1):
        try:
            resp = proxy(request)
        except rospy.ServiceException as e:
            rospy.logwarn("%s raised on attempt %d/%d: %s", desc, attempt, n_retries, e)
            time.sleep(delay)
            continue
        last_resp = resp
        if getattr(resp, "result", False):
            rospy.loginfo("%s OK on attempt %d/%d (ack=%s)",
                          desc, attempt, n_retries, getattr(resp, "ack_data", "n/a"))
            return resp
        ack = getattr(resp, "ack_data", None)
        msg = "%s returned result=false (ack=%s) attempt %d/%d" % (desc, ack, attempt, n_retries)
        if hint:
            msg += " — " + hint
        rospy.logwarn(msg)
        time.sleep(delay)
    return last_resp


# ---------------------------------------------------------------------------
# Mission control wrappers
# ---------------------------------------------------------------------------

def action_stop(action_srv):
    """Send ACTION_STOP. Returns (ok, ack_data). Result=false: see ack (0=often
    no local mission; 236=waypoint IN_PROGRESS; etc.)."""
    try:
        resp = action_srv(dji_srv.MissionWpActionRequest.ACTION_STOP)
    except rospy.ServiceException as e:
        rospy.logwarn("ACTION_STOP raised: %s", e)
        return False, None
    ack = getattr(resp, "ack_data", None)
    if resp.result:
        rospy.loginfo("ACTION_STOP confirmed (ack=%s).", ack)
    elif ack == ACK_WAYPOINT_IN_PROGRESS or ack == ACK_MISSION_IN_PROGRESS:
        rospy.logwarn(
            "ACTION_STOP result=false (ack=%s): mission still in progress on FC — upload retry may help.",
            ack,
        )
    elif ack == ACK_WAYPOINT_NOT_IN_PROGRESS:
        rospy.loginfo(
            "ACTION_STOP result=false (ack=%s): FC reports waypoint mission not in progress.",
            ack,
        )
    elif ack == 0:
        rospy.logwarn(
            "ACTION_STOP result=false (ack=0): often no waypoint mission in dji_sdk_node.",
        )
    else:
        rospy.logwarn("ACTION_STOP result=false (ack=%s).", ack)
    return resp.result, ack


def _read_wp_mission_count(status_srv):
    """Return waypoint_mission_count or None if mission_status call failed."""
    try:
        r = status_srv()
        return int(r.waypoint_mission_count)
    except rospy.ServiceException as e:
        rospy.logwarn("mission_status call failed: %s", e)
        return None


# FC may return these on ACTION_STOP while it is still tearing down a mission.
TRANSIENT_STOP_ACKS = frozenset(
    {
        ACK_MISSION_IN_PROGRESS,
        ACK_WAYPOINT_IN_PROGRESS,
        ACK_DATA_NOT_ENOUGH,
    }
)


def clear_old_mission_blocking(
    upload_srv,
    action_srv,
    status_srv,
    anchor_lat,
    anchor_lon,
    anchor_alt,
    idle_speed,
    timeout_sec,
):
    """Clear waypoint mission state before uploading the real mission.

    Proven sequence (same idea as the original runner, plus a short post-upload
    settle): **dummy 2-WP upload** to prime ``dji_sdk_node``, then **two**
    ``ACTION_STOP``\ s with ``MISSION_STOP_SETTLE_SEC`` between them.

    We intentionally **do not** loop on ``ACTION_STOP`` / ``ACTION_PAUSE`` here:
    on some firmware, **PAUSE** before start returns ``NOT_IN_PROGRESS`` (0xED),
    and tight **STOP** retries produce long ``MISSION_IN_PROGRESS`` storms while
    the FC is still handling the upload burst. Residual busyness is handled by
    ``upload_mission_with_retry``.

    ``timeout_sec`` applies only to **retrying the dummy upload** if the service
    raises ``ServiceException``.
    """
    deadline = time.time() + timeout_sec
    dummy = build_dummy_task(anchor_lat, anchor_lon, anchor_alt, idle_speed)
    rospy.loginfo("=== CLEAR OLD MISSION ===")
    rospy.loginfo("Priming dji_sdk local mission state (dummy 2-WP upload)...")
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            d = upload_srv(dummy)
            rospy.loginfo(
                "  Dummy upload result=%s ack=%s",
                getattr(d, "result", None),
                getattr(d, "ack_data", None),
            )
            break
        except rospy.ServiceException as e:
            rospy.logwarn("  Dummy upload raised: %s — retrying until timeout...", e)
            time.sleep(CLEAR_STOP_POLL_DELAY_SEC)
    else:
        rospy.logfatal(
            "Timed out after %.0fs waiting for dummy mission upload during clear.",
            timeout_sec,
        )
        sys.exit(1)

    rospy.loginfo(
        "Waiting %.1fs for FC/SDK to settle after dummy upload...",
        CLEAR_POST_DUMMY_SETTLE_SEC,
    )
    rospy.sleep(CLEAR_POST_DUMMY_SETTLE_SEC)

    rospy.loginfo("Sending ACTION_STOP...")
    action_stop(action_srv)
    rospy.sleep(MISSION_STOP_SETTLE_SEC)

    rospy.loginfo("Confirming with a second ACTION_STOP (defensive)...")
    action_stop(action_srv)
    rospy.sleep(MISSION_STOP_SETTLE_SEC)

    wp_final = _read_wp_mission_count(status_srv)
    if wp_final is not None:
        rospy.loginfo("  mission_status waypoint_mission_count=%d (informational)", wp_final)

    try:
        r = status_srv()
        hp = int(r.hotpoint_mission_count)
        if hp:
            rospy.logwarn(
                "Hotpoint mission count is still %d (waypoint clear done). "
                "Stop hotpoint separately if needed.",
                hp,
            )
    except rospy.ServiceException:
        pass

    rospy.loginfo("=== CLEAR OLD MISSION done ===")


def upload_mission_with_retry(upload_srv, action_srv, ctrl_srv, task):
    """Upload mission; on transient ack failures, run the appropriate
    recovery (re-acquire control / ACTION_STOP) and retry."""
    last_resp = None
    for attempt in range(1, UPLOAD_RETRY_ATTEMPTS + 1):
        rospy.loginfo("Uploading %d waypoints (attempt %d/%d)...",
                      len(task.mission_waypoint), attempt, UPLOAD_RETRY_ATTEMPTS)
        try:
            resp = upload_srv(task)
        except rospy.ServiceException as e:
            rospy.logwarn("Upload raised: %s", e)
            time.sleep(UPLOAD_RETRY_DELAY_SEC)
            continue

        last_resp = resp
        if resp.result:
            rospy.loginfo("Upload OK (ack=%d).", resp.ack_data)
            return resp

        rospy.logwarn("Upload failed (ack=%d).", resp.ack_data)
        if resp.ack_data == ACK_NEED_OBTAIN_CONTROL:
            rospy.logwarn("Re-acquiring SDK control authority...")
            try:
                ctrl_srv(dji_srv.SDKControlAuthorityRequest.REQUEST_CONTROL)
            except rospy.ServiceException as e:
                rospy.logwarn("Re-acquire raised: %s", e)
        elif resp.ack_data in (
            ACK_MISSION_IN_PROGRESS,
            ACK_WAYPOINT_IN_PROGRESS,
            ACK_DATA_NOT_ENOUGH,
        ):
            rospy.logwarn("Issuing ACTION_STOP before retry...")
            action_stop(action_srv)
        else:
            rospy.logfatal("Unrecoverable upload ack=%d. Aborting retries.", resp.ack_data)
            return resp

        time.sleep(UPLOAD_RETRY_DELAY_SEC)
    return last_resp


# ---------------------------------------------------------------------------
# Mission tracking (same shape as waypoint_runner.py)
# ---------------------------------------------------------------------------

def track_waypoints(waypoints, arrive_radius, mission_start):
    arrival_times = [None] * len(waypoints)
    next_wp = 0
    total = len(waypoints)
    last_log_time = 0.0

    rospy.loginfo("Tracking %d waypoints (arrive radius=%.1f m)...", total, arrive_radius)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and next_wp < total:
        if not _state.gps_valid:
            rospy.logwarn_throttle(5.0, "Lost GPS fix mid-mission.")
            rate.sleep()
            continue

        lat, lon = _state.gps_lat, _state.gps_lon
        wp_lat, wp_lon, wp_alt, _wp_yaw = waypoints[next_wp]
        dist = haversine_m(lat, lon, wp_lat, wp_lon)

        now = time.time()
        if now - last_log_time >= 2.0:
            rospy.loginfo(
                "Heading to WP %d/%d  |  dist: %.1f m  |  pos: (%.7f, %.7f)  |  target: (%.7f, %.7f, %.1f m)",
                next_wp + 1, total, dist, lat, lon, wp_lat, wp_lon, wp_alt
            )
            last_log_time = now

        if dist <= arrive_radius:
            arrival_times[next_wp] = time.time()
            elapsed = arrival_times[next_wp] - mission_start
            leg_time = elapsed if next_wp == 0 else arrival_times[next_wp] - arrival_times[next_wp - 1]
            rospy.loginfo("WP %d/%d REACHED  |  leg: %.2f s  |  total: %.2f s",
                          next_wp + 1, total, leg_time, elapsed)
            next_wp += 1
            if next_wp < total:
                nxt_lat, nxt_lon, nxt_alt, _nxt_yaw = waypoints[next_wp]
                rospy.loginfo("Next target: WP %d  (%.7f, %.7f, %.1f m)",
                              next_wp + 1, nxt_lat, nxt_lon, nxt_alt)

        rate.sleep()
    return arrival_times


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="DJI waypoint mission runner (synchronized)")
    parser.add_argument("waypoints_file",
                        help="Path to waypoints text file ('lat, lon, alt' or 'lat, lon, alt, yaw_rad' per line)")
    parser.add_argument("--speed", type=float, default=DEFAULT_IDLE_SPEED,
                        help="Cruise speed in m/s (default: %.1f)" % DEFAULT_IDLE_SPEED)
    parser.add_argument("--arrive-radius", type=float, default=DEFAULT_ARRIVE_RADIUS_M,
                        help="Arrival detection radius in metres (default: %.1f)" % DEFAULT_ARRIVE_RADIUS_M)
    parser.add_argument("--skip-clear", action="store_true",
                        help="Skip the prime+STOP clear step (debug/sim only).")
    parser.add_argument(
        "--clear-timeout",
        type=float,
        default=CLEAR_MISSION_TIMEOUT_SEC,
        metavar="SEC",
        help="Max seconds to retry dummy mission upload during clear if the service raises (default: %.0f)"
        % CLEAR_MISSION_TIMEOUT_SEC,
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("waypoint_runner_synchronized", anonymous=True)
    rospy.loginfo("waypoint_runner_synchronized starting | file=%s speed=%.1f arrive_radius=%.1f",
                  args.waypoints_file, args.speed, args.arrive_radius)

    # --- Parse waypoints ---
    rospy.loginfo("Loading waypoints from: %s", args.waypoints_file)
    waypoints = parse_waypoints(args.waypoints_file)
    if len(waypoints) < 2:
        rospy.logfatal("Need at least 2 waypoints; got %d. Aborting.", len(waypoints))
        sys.exit(1)
    rospy.loginfo("Loaded %d waypoints:", len(waypoints))
    for i, (lat, lon, alt, yaw) in enumerate(waypoints):
        if yaw is None:
            rospy.loginfo("  WP%d: lat=%.7f lon=%.7f alt=%.1f m  yaw=AUTO", i + 1, lat, lon, alt)
        else:
            rospy.loginfo("  WP%d: lat=%.7f lon=%.7f alt=%.1f m  yaw=%.3f rad (%.1f deg DJI)",
                          i + 1, lat, lon, alt, yaw, yaw_rad_to_dji_deg(yaw))

    # --- Telemetry subscriptions ---
    rospy.Subscriber("dji_sdk/flight_status", UInt8,     _flight_status_cb, queue_size=1)
    rospy.Subscriber("dji_sdk/gps_position",  NavSatFix, _gps_cb,           queue_size=1)

    # --- Gate: telemetry ---
    rospy.loginfo("Waiting for /dji_sdk/flight_status...")
    if not wait_until(lambda: _state.flight_status is not None, "flight_status", TELEMETRY_WAIT_SEC):
        rospy.logfatal("No flight_status — is dji_sdk_node alive and the FC connected?")
        sys.exit(1)
    rospy.loginfo("  flight_status=%d", _state.flight_status)

    rospy.loginfo("Waiting for /dji_sdk/gps_position...")
    if not wait_until(lambda: _state.gps_valid, "GPS fix", TELEMETRY_WAIT_SEC):
        rospy.logfatal("No GPS fix.")
        sys.exit(1)
    rospy.loginfo("  GPS lat=%.7f lon=%.7f", _state.gps_lat, _state.gps_lon)

    # --- Airborne check ---
    if _state.flight_status != FLIGHT_STATUS_IN_AIR:
        rospy.logfatal("Drone NOT airborne (flight_status=%d, expected %d). Take off, then re-run.",
                       _state.flight_status, FLIGHT_STATUS_IN_AIR)
        sys.exit(1)

    # --- Gate: services ---
    for svc in ("dji_sdk/set_local_pos_ref",
                "dji_sdk/sdk_control_authority",
                "dji_sdk/mission_waypoint_upload",
                "dji_sdk/mission_waypoint_action",
                "dji_sdk/mission_status"):
        if not wait_for_service(svc, SERVICE_WAIT_SEC):
            sys.exit(1)

    set_local_pos_srv = rospy.ServiceProxy("dji_sdk/set_local_pos_ref",     dji_srv.SetLocalPosRef)
    ctrl_auth_srv     = rospy.ServiceProxy("dji_sdk/sdk_control_authority", dji_srv.SDKControlAuthority)
    upload_srv        = rospy.ServiceProxy("dji_sdk/mission_waypoint_upload", dji_srv.MissionWpUpload)
    action_srv        = rospy.ServiceProxy("dji_sdk/mission_waypoint_action", dji_srv.MissionWpAction)
    mission_status_srv = rospy.ServiceProxy("dji_sdk/mission_status", dji_srv.MissionStatus)

    # --- set_local_pos_ref (retry) ---
    rospy.loginfo("Setting local position reference frame...")
    resp = call_service_with_retry(
        set_local_pos_srv, dji_srv.SetLocalPosRefRequest(),
        "set_local_pos_ref", SET_LOCAL_POS_RETRIES, SET_LOCAL_POS_DELAY_SEC,
        hint="GPS may still be converging",
    )
    if resp is None or not resp.result:
        rospy.logfatal("set_local_pos_ref failed after %d retries.", SET_LOCAL_POS_RETRIES)
        sys.exit(1)

    # --- SDK control authority (retry, operator hint) ---
    rospy.loginfo("Requesting SDK control authority...")
    resp = call_service_with_retry(
        ctrl_auth_srv,
        dji_srv.SDKControlAuthorityRequest(control_enable=dji_srv.SDKControlAuthorityRequest.REQUEST_CONTROL),
        "sdk_control_authority", CTRL_AUTH_RETRIES, CTRL_AUTH_DELAY_SEC,
        hint="RC switch must be in F-mode (Onboard)",
    )
    if resp is None or not resp.result:
        rospy.logfatal("Could not obtain SDK control authority. Is the RC switch in F-mode?")
        sys.exit(1)

    # --- CLEAR OLD MISSION ---
    if args.skip_clear:
        rospy.logwarn("--skip-clear set; not clearing FC mission state.")
    else:
        first_lat, first_lon, first_alt, _ = waypoints[0]
        clear_old_mission_blocking(
            upload_srv,
            action_srv,
            mission_status_srv,
            first_lat,
            first_lon,
            first_alt,
            args.speed,
            args.clear_timeout,
        )

    # --- UPLOAD NEW MISSION ---
    rospy.loginfo("=== UPLOAD NEW MISSION ===")
    task = build_waypoint_task(waypoints, args.speed)
    up_resp = upload_mission_with_retry(upload_srv, action_srv, ctrl_auth_srv, task)
    if up_resp is None or not up_resp.result:
        ack = up_resp.ack_data if up_resp is not None else "n/a"
        rospy.logfatal("Mission upload failed after %d retries (last ack=%s).",
                       UPLOAD_RETRY_ATTEMPTS, ack)
        sys.exit(1)
    rospy.loginfo("=== UPLOAD NEW MISSION done ===")

    # --- START MISSION ---
    rospy.loginfo("=== START MISSION ===")
    try:
        start_resp = action_srv(dji_srv.MissionWpActionRequest.ACTION_START)
    except rospy.ServiceException as e:
        rospy.logfatal("ACTION_START raised: %s", e)
        sys.exit(1)
    if not start_resp.result:
        rospy.logfatal("ACTION_START failed (ack=%d).", start_resp.ack_data)
        sys.exit(1)
    mission_start = time.time()
    rospy.loginfo("Mission running.")

    # --- Track ---
    arrival_times = track_waypoints(waypoints, args.arrive_radius, mission_start)

    # --- Summary ---
    reached = sum(1 for t in arrival_times if t is not None)
    last_time = arrival_times[-1] if arrival_times and arrival_times[-1] else time.time()
    total_time = last_time - mission_start

    rospy.loginfo("=== Mission complete ===")
    rospy.loginfo("Waypoints reached : %d / %d", reached, len(waypoints))
    rospy.loginfo("Total mission time: %.2f s", total_time)
    if reached >= 2:
        rospy.loginfo("Per-leg breakdown:")
        prev = mission_start
        for i, t in enumerate(arrival_times):
            if t is None:
                rospy.loginfo("  WP%d : not reached", i + 1)
            else:
                rospy.loginfo("  WP%d : %.2f s", i + 1, t - prev)
                prev = t


if __name__ == "__main__":
    main()
