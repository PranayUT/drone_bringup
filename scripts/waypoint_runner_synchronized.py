#!/usr/bin/env python3
"""Synchronized DJI waypoint mission runner (ROS1 + dji_sdk).

Pipeline
--------
1. Parse waypoints, subscribe to FC telemetry (incl. ``/dji_sdk/gps_health``).
2. Wait for GPS fix, ``gps_health > 3`` (``set_local_pos_ref`` gate in ``dji_sdk_node``),
   then airborne (``flight_status == IN_AIR``).
3. ``set_local_pos_ref`` and ``sdk_control_authority`` with retries.
4. **Clear** old waypoint state (see ``clear_waypoint_stack``).
5. Upload real mission (retries + STOP on transient acks), ``ACTION_START``, track.

Clear semantics (important)
---------------------------
``mission_status.waypoint_mission_count`` is ``wpMissionVector.size()`` in
``dji_sdk_node`` (SDK-side), not a direct FC “Go mission” mirror.

- **Drain:** if the SDK already holds waypoint mission object(s), ``ACTION_STOP``
  until count reaches **0** (strict; handles ``pkill`` mid-mission).
- **Prime:** if count is **0**, a bare ``ACTION_STOP`` does not reach the FC
  (``dji_sdk`` rejects STOP with empty vector). We upload a **2-point dummy**
  near WP1, then try to disarm it.
- **Disarm dummy:** some firmware returns ``WAYPOINT_IN_PROGRESS`` (236) on
  STOP while the upload pipeline is still “hot”, and count may stay **1** for a
  long time. We use **PAUSE**, backoff, and repeated **STOP** within a **bounded**
  budget; we prefer ``count == 0`` or a **successful STOP**, but if the budget
  expires we **warn and continue** — the real **upload** step still must return
  ``result=True`` before ``ACTION_START``, so we do not fly an un-uploaded plan.

Waypoints file: same as ``waypoint_runner.py`` — ``lat, lon, alt[, yaw_rad]``.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8

import dji_sdk.msg as dji_msg
import dji_sdk.srv as dji_srv

# ---------------------------------------------------------------------------
# FC / protocol constants
# ---------------------------------------------------------------------------

FLIGHT_STATUS_ON_GROUND = 1
FLIGHT_STATUS_IN_AIR = 2

DEFAULT_ARRIVE_RADIUS_M = 1.0
DEFAULT_IDLE_SPEED = 5.0

TELEMETRY_WAIT_SEC = 60.0
SERVICE_WAIT_SEC = 30.0

SET_LOCAL_POS_RETRIES = 5
SET_LOCAL_POS_DELAY_SEC = 2.0
CTRL_AUTH_RETRIES = 30
CTRL_AUTH_DELAY_SEC = 2.0

UPLOAD_RETRY_ATTEMPTS = 5
UPLOAD_RETRY_DELAY_SEC = 3.0
MISSION_STOP_SETTLE_SEC = 2.0

CLEAR_MISSION_TIMEOUT_SEC = 300.0
CLEAR_STOP_POLL_DELAY_SEC = 0.5
CLEAR_POST_DUMMY_SETTLE_SEC = 2.0
# After dummy upload, do not spin forever on mission_status==0 (FC can ack 236).
CLEAR_DISARM_BUDGET_SEC = 45.0
CLEAR_DISARM_PAUSE_EVERY = 8  # re-issue PAUSE every N STOP attempts

ACK_NEED_OBTAIN_CONTROL = 209
ACK_MISSION_IN_PROGRESS = 213
ACK_DATA_NOT_ENOUGH = 234
ACK_WAYPOINT_IN_PROGRESS = 236
ACK_WAYPOINT_NOT_IN_PROGRESS = 237

TRANSIENT_STOP_ACKS = frozenset(
    (ACK_MISSION_IN_PROGRESS, ACK_WAYPOINT_IN_PROGRESS, ACK_DATA_NOT_ENOUGH)
)


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two WGS84 points."""
    r_earth = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return r_earth * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def yaw_rad_to_dji_deg(yaw_rad):
    """Local yaw (rad, 0=south) -> DJI target_yaw (deg, 0=north, +CW, (-180,180])."""
    deg = 180.0 - math.degrees(yaw_rad)
    return (deg + 180.0) % 360.0 - 180.0


def parse_waypoints(path):
    """Parse ``lat, lon, alt[, yaw_rad]`` lines into a list of tuples."""
    waypoints = []
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) not in (3, 4):
                rospy.logwarn(
                    "Line %d: expected 'lat,lon,alt[,yaw_rad]' - skipping: %s",
                    lineno,
                    raw.rstrip(),
                )
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
    """Build ``dji_sdk/MissionWaypointTask``. Uses ``YAW_MODE_WAYPOINT`` when any
    waypoint has ``yaw`` (per-waypoint ``target_yaw``); else ``YAW_MODE_AUTO``."""
    has_yaw = any(yaw is not None for _, _, _, yaw in waypoints)
    task = dji_msg.MissionWaypointTask()
    task.velocity_range = 15.0
    task.idle_velocity = float(idle_speed)
    task.action_on_finish = dji_msg.MissionWaypointTask.FINISH_NO_ACTION
    task.mission_exec_times = 1
    task.yaw_mode = (
        dji_msg.MissionWaypointTask.YAW_MODE_WAYPOINT
        if has_yaw
        else dji_msg.MissionWaypointTask.YAW_MODE_AUTO
    )
    task.trace_mode = dji_msg.MissionWaypointTask.TRACE_POINT
    task.action_on_rc_lost = dji_msg.MissionWaypointTask.ACTION_AUTO
    task.gimbal_pitch_mode = dji_msg.MissionWaypointTask.GIMBAL_PITCH_FREE

    for lat, lon, alt, yaw in waypoints:
        wp = dji_msg.MissionWaypoint()
        wp.latitude = lat
        wp.longitude = lon
        wp.altitude = float(alt)
        wp.damping_distance = 0.0
        wp.target_yaw = int(round(yaw_rad_to_dji_deg(yaw))) if yaw is not None else 0
        wp.target_gimbal_pitch = 0
        wp.turn_mode = 0
        wp.has_action = 0
        wp.action_time_limit = 0
        action = dji_msg.MissionWaypointAction()
        action.action_repeat = 0
        action.command_list = [0] * 16
        action.command_parameter = [0] * 16
        wp.waypoint_action = action
        task.mission_waypoint.append(wp)
    return task


def build_dummy_task(anchor_lat, anchor_lon, anchor_alt, idle_speed):
    """2-WP dummy near anchor — primes ``dji_sdk_node`` so STOP can reach the FC."""
    wps = [
        (anchor_lat, anchor_lon, anchor_alt, None),
        (anchor_lat + 1.0e-5, anchor_lon, anchor_alt, None),
    ]
    return build_waypoint_task(wps, idle_speed)


# ---------------------------------------------------------------------------
# ROS state
# ---------------------------------------------------------------------------


class DroneState:
    __slots__ = ("flight_status", "gps_lat", "gps_lon", "gps_valid", "gps_health")

    def __init__(self):
        self.flight_status = None
        self.gps_lat = None
        self.gps_lon = None
        self.gps_valid = False
        self.gps_health = None


_state = DroneState()


def _flight_status_cb(msg):
    _state.flight_status = msg.data


def _gps_cb(msg):
    if msg.status.status >= 0:
        _state.gps_lat = msg.latitude
        _state.gps_lon = msg.longitude
        _state.gps_valid = True


def _gps_health_cb(msg):
    _state.gps_health = int(msg.data)


# ---------------------------------------------------------------------------
# ROS I/O helpers
# ---------------------------------------------------------------------------


def wait_until(predicate, desc, timeout_sec):
    """Poll ``predicate()`` until true or timeout. Returns bool."""
    deadline = time.time() + timeout_sec
    last_log = 0.0
    while time.time() < deadline and not rospy.is_shutdown():
        if predicate():
            return True
        now = time.time()
        if now - last_log >= 5.0:
            rospy.loginfo("  ... still waiting for %s (%.0fs remaining)", desc, deadline - now)
            last_log = now
        time.sleep(0.2)
    return False


def wait_for_service(name, timeout_sec):
    rospy.loginfo("Waiting for service %s (timeout %.0fs)...", name, timeout_sec)
    try:
        rospy.wait_for_service(name, timeout=timeout_sec)
        rospy.loginfo("  %s ready", name)
        return True
    except rospy.ROSException:
        rospy.logfatal("Service %s did not appear after %.0fs.", name, timeout_sec)
        return False


def call_service_with_retry(proxy, request, desc, n_retries, delay_sec, hint=None):
    """Call service until ``result`` or retries exhausted."""
    last_resp = None
    for attempt in range(1, n_retries + 1):
        rospy.loginfo("%s service call (attempt %d/%d)...", desc, attempt, n_retries)
        try:
            resp = proxy(request)
        except rospy.ServiceException as e:
            rospy.logwarn("%s raised on attempt %d/%d: %s", desc, attempt, n_retries, e)
            time.sleep(delay_sec)
            continue
        last_resp = resp
        if getattr(resp, "result", False):
            rospy.loginfo(
                "%s OK on attempt %d/%d (ack=%s)",
                desc,
                attempt,
                n_retries,
                getattr(resp, "ack_data", "n/a"),
            )
            return resp
        ack = getattr(resp, "ack_data", None)
        msg = "%s returned result=false (ack=%s) attempt %d/%d" % (desc, ack, attempt, n_retries)
        if hint:
            msg += " — " + hint
        rospy.logwarn(msg)
        time.sleep(delay_sec)
    return last_resp


def read_wp_mission_count(status_srv):
    try:
        return int(status_srv().waypoint_mission_count)
    except rospy.ServiceException as e:
        rospy.logwarn("mission_status call failed: %s", e)
        return None


def action_pause(action_srv):
    """Best-effort PAUSE for waypoint mission."""
    try:
        resp = action_srv(dji_srv.MissionWpActionRequest.ACTION_PAUSE)
    except rospy.ServiceException as e:
        rospy.logwarn("ACTION_PAUSE raised: %s", e)
        return False, None
    ack = getattr(resp, "ack_data", None)
    if resp.result:
        rospy.loginfo("ACTION_PAUSE OK (ack=%s).", ack)
    else:
        rospy.logwarn("ACTION_PAUSE result=false (ack=%s).", ack)
    return resp.result, ack


def action_stop(action_srv):
    try:
        resp = action_srv(dji_srv.MissionWpActionRequest.ACTION_STOP)
    except rospy.ServiceException as e:
        rospy.logwarn("ACTION_STOP raised: %s", e)
        return False, None
    ack = getattr(resp, "ack_data", None)
    if resp.result:
        rospy.loginfo("ACTION_STOP OK (ack=%s).", ack)
    elif ack in (ACK_WAYPOINT_IN_PROGRESS, ACK_MISSION_IN_PROGRESS):
        rospy.logwarn(
            "ACTION_STOP result=false (ack=%s): waypoint/mission still busy on FC.",
            ack,
        )
    elif ack == ACK_WAYPOINT_NOT_IN_PROGRESS:
        rospy.loginfo("ACTION_STOP result=false (ack=%s): FC reports WP mission not in progress.", ack)
    elif ack == 0:
        rospy.logwarn("ACTION_STOP result=false (ack=0): often no WP mission in dji_sdk_node.")
    else:
        rospy.logwarn("ACTION_STOP result=false (ack=%s).", ack)
    return resp.result, ack


# ---------------------------------------------------------------------------
# Clear stack (drain → prime → disarm)
# ---------------------------------------------------------------------------


def _drain_sdk_waypoints(action_srv, status_srv, deadline):
    """STOP until ``waypoint_mission_count`` is 0 (strict)."""
    rospy.loginfo("--- Clear: drain SDK (STOP until waypoint_mission_count==0) ---")
    while time.time() < deadline and not rospy.is_shutdown():
        cnt = read_wp_mission_count(status_srv)
        if cnt is None:
            time.sleep(CLEAR_STOP_POLL_DELAY_SEC)
            continue
        if cnt == 0:
            rospy.loginfo("  waypoint_mission_count=0 (SDK empty).")
            return
        rospy.loginfo("  waypoint_mission_count=%d — ACTION_STOP", cnt)
        action_stop(action_srv)
        time.sleep(MISSION_STOP_SETTLE_SEC)
    rospy.logfatal("Clear drain timed out: waypoint_mission_count never reached 0.")
    sys.exit(1)


def _upload_dummy_until_ok(upload_srv, action_srv, dummy_task, deadline):
    """Upload dummy mission; on failure STOP lightly and retry."""
    rospy.loginfo("--- Clear: dummy 2-WP upload (FC prime) ---")
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            resp = upload_srv(dummy_task)
        except rospy.ServiceException as e:
            rospy.logwarn("  Dummy upload raised: %s — retrying...", e)
            time.sleep(CLEAR_STOP_POLL_DELAY_SEC)
            continue
        rospy.loginfo(
            "  Dummy upload result=%s ack=%s",
            getattr(resp, "result", None),
            getattr(resp, "ack_data", None),
        )
        if getattr(resp, "result", False):
            return
        rospy.logwarn("  Dummy upload rejected — ACTION_STOP then retry...")
        action_stop(action_srv)
        time.sleep(CLEAR_STOP_POLL_DELAY_SEC)
    rospy.logfatal("Dummy upload never succeeded before clear timeout.")
    sys.exit(1)


def _disarm_after_dummy(action_srv, status_srv, disarm_deadline):
    """Try PAUSE + STOP + backoff until count==0, STOP success, or budget exhausted."""
    rospy.loginfo(
        "--- Clear: disarm dummy (PAUSE + STOP; budget %.0fs; may warn if FC stays 236) ---",
        CLEAR_DISARM_BUDGET_SEC,
    )
    rospy.sleep(CLEAR_POST_DUMMY_SETTLE_SEC)

    action_pause(action_srv)
    rospy.sleep(1.0)

    stop_round = 0
    saw_stop_ok = False
    while time.time() < disarm_deadline and not rospy.is_shutdown():
        cnt = read_wp_mission_count(status_srv)
        if cnt == 0:
            rospy.loginfo("  waypoint_mission_count=0 after disarm — OK to upload real mission.")
            return

        if stop_round > 0 and stop_round % CLEAR_DISARM_PAUSE_EVERY == 0:
            rospy.loginfo("  Re-trying ACTION_PAUSE (round %d)...", stop_round)
            action_pause(action_srv)
            rospy.sleep(0.8)

        rospy.loginfo("  waypoint_mission_count=%s — ACTION_STOP (round %d)", cnt, stop_round + 1)
        ok, ack = action_stop(action_srv)
        if ok:
            saw_stop_ok = True
            rospy.sleep(2.0)
            cnt2 = read_wp_mission_count(status_srv)
            if cnt2 == 0:
                rospy.loginfo("  waypoint_mission_count=0 after successful STOP.")
                return
        settle = MISSION_STOP_SETTLE_SEC
        if not ok and ack in TRANSIENT_STOP_ACKS:
            settle = min(8.0, 2.0 + 0.35 * stop_round)
        time.sleep(settle)
        stop_round += 1

    cnt = read_wp_mission_count(status_srv)
    if cnt == 0:
        rospy.loginfo("  waypoint_mission_count=0 at disarm deadline.")
        return

    rospy.logwarn(
        "Clear disarm budget exhausted (last waypoint_mission_count=%s, saw_STOP_ok=%s). "
        "Some FC builds return ack=236 on STOP after a fresh dummy upload while the SDK "
        "still shows count=1. Proceeding — real mission upload must still return result=True.",
        cnt,
        saw_stop_ok,
    )


def clear_waypoint_stack(upload_srv, action_srv, status_srv, anchor_lat, anchor_lon, anchor_alt, idle_speed, timeout_sec):
    """Full clear: drain → optional prime → disarm dummy."""
    rospy.loginfo("=== CLEAR WAYPOINT STACK (timeout %.0fs) ===", timeout_sec)
    deadline = time.time() + timeout_sec
    dummy = build_dummy_task(anchor_lat, anchor_lon, anchor_alt, idle_speed)

    _drain_sdk_waypoints(action_srv, status_srv, deadline)

    _upload_dummy_until_ok(upload_srv, action_srv, dummy, deadline)

    disarm_deadline = min(time.time() + CLEAR_DISARM_BUDGET_SEC, deadline)
    _disarm_after_dummy(action_srv, status_srv, disarm_deadline)

    try:
        hp = int(status_srv().hotpoint_mission_count)
        if hp:
            rospy.logwarn(
                "Hotpoint mission count is still %d (waypoint clear done). Stop hotpoint separately if needed.",
                hp,
            )
    except rospy.ServiceException:
        pass

    rospy.loginfo("=== CLEAR WAYPOINT STACK done ===")


def upload_mission_with_retry(upload_srv, action_srv, ctrl_srv, task):
    last_resp = None
    for attempt in range(1, UPLOAD_RETRY_ATTEMPTS + 1):
        rospy.loginfo(
            "Uploading %d waypoints (attempt %d/%d)...",
            len(task.mission_waypoint),
            attempt,
            UPLOAD_RETRY_ATTEMPTS,
        )
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
                next_wp + 1,
                total,
                dist,
                lat,
                lon,
                wp_lat,
                wp_lon,
                wp_alt,
            )
            last_log_time = now
        if dist <= arrive_radius:
            arrival_times[next_wp] = time.time()
            elapsed = arrival_times[next_wp] - mission_start
            leg_time = elapsed if next_wp == 0 else arrival_times[next_wp] - arrival_times[next_wp - 1]
            rospy.loginfo("WP %d/%d REACHED  |  leg: %.2f s  |  total: %.2f s", next_wp + 1, total, leg_time, elapsed)
            next_wp += 1
            if next_wp < total:
                nxt_lat, nxt_lon, nxt_alt, _nxt_yaw = waypoints[next_wp]
                rospy.loginfo("Next target: WP %d  (%.7f, %.7f, %.1f m)", next_wp + 1, nxt_lat, nxt_lon, nxt_alt)
        rate.sleep()
    return arrival_times


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="DJI waypoint mission runner (synchronized)")
    parser.add_argument(
        "waypoints_file",
        help="Path to waypoints text file ('lat, lon, alt' or 'lat, lon, alt, yaw_rad' per line)",
    )
    parser.add_argument("--speed", type=float, default=DEFAULT_IDLE_SPEED, help="Cruise speed in m/s")
    parser.add_argument("--arrive-radius", type=float, default=DEFAULT_ARRIVE_RADIUS_M, help="Arrival radius (m)")
    parser.add_argument("--skip-clear", action="store_true", help="Skip clear (debug/sim only).")
    parser.add_argument(
        "--clear-timeout",
        type=float,
        default=CLEAR_MISSION_TIMEOUT_SEC,
        metavar="SEC",
        help="Max seconds for drain + dummy + disarm (default: %.0f)" % CLEAR_MISSION_TIMEOUT_SEC,
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    sys.stderr.write("waypoint_runner_synchronized: process started (before rospy.init_node)\n")
    sys.stderr.flush()

    rospy.init_node("waypoint_runner_synchronized", anonymous=True)
    rospy.loginfo(
        "waypoint_runner_synchronized | file=%s speed=%.1f arrive_radius=%.1f",
        args.waypoints_file,
        args.speed,
        args.arrive_radius,
    )

    rospy.loginfo("Loading waypoints from: %s", args.waypoints_file)
    waypoints = parse_waypoints(args.waypoints_file)
    if len(waypoints) < 2:
        rospy.logfatal("Need at least 2 waypoints; got %d.", len(waypoints))
        sys.exit(1)
    rospy.loginfo("Loaded %d waypoints:", len(waypoints))
    for i, (lat, lon, alt, yaw) in enumerate(waypoints):
        if yaw is None:
            rospy.loginfo("  WP%d: lat=%.7f lon=%.7f alt=%.1f m  yaw=AUTO", i + 1, lat, lon, alt)
        else:
            rospy.loginfo(
                "  WP%d: lat=%.7f lon=%.7f alt=%.1f m  yaw=%.3f rad (%.1f deg DJI)",
                i + 1,
                lat,
                lon,
                alt,
                yaw,
                yaw_rad_to_dji_deg(yaw),
            )

    rospy.Subscriber("dji_sdk/flight_status", UInt8, _flight_status_cb, queue_size=1)
    rospy.Subscriber("dji_sdk/gps_position", NavSatFix, _gps_cb, queue_size=1)
    rospy.Subscriber("dji_sdk/gps_health", UInt8, _gps_health_cb, queue_size=1)

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

    rospy.loginfo("Waiting for /dji_sdk/gps_health > 3 (set_local_pos_ref gate in dji_sdk)...")
    if not wait_until(
        lambda: _state.gps_health is not None and _state.gps_health > 3,
        "gps_health>3",
        TELEMETRY_WAIT_SEC,
    ):
        rospy.logfatal(
            "GPS health never exceeded 3 (last=%s). Open sky / wait for convergence, then retry.",
            _state.gps_health,
        )
        sys.exit(1)
    rospy.loginfo("  gps_health=%d", _state.gps_health)

    if _state.flight_status != FLIGHT_STATUS_IN_AIR:
        rospy.logfatal(
            "Drone NOT airborne (flight_status=%d, expected %d). Take off, then re-run.",
            _state.flight_status,
            FLIGHT_STATUS_IN_AIR,
        )
        sys.exit(1)

    for svc in (
        "dji_sdk/set_local_pos_ref",
        "dji_sdk/sdk_control_authority",
        "dji_sdk/mission_waypoint_upload",
        "dji_sdk/mission_waypoint_action",
        "dji_sdk/mission_status",
    ):
        if not wait_for_service(svc, SERVICE_WAIT_SEC):
            sys.exit(1)

    set_local_pos_srv = rospy.ServiceProxy("dji_sdk/set_local_pos_ref", dji_srv.SetLocalPosRef)
    ctrl_auth_srv = rospy.ServiceProxy("dji_sdk/sdk_control_authority", dji_srv.SDKControlAuthority)
    upload_srv = rospy.ServiceProxy("dji_sdk/mission_waypoint_upload", dji_srv.MissionWpUpload)
    action_srv = rospy.ServiceProxy("dji_sdk/mission_waypoint_action", dji_srv.MissionWpAction)
    mission_status_srv = rospy.ServiceProxy("dji_sdk/mission_status", dji_srv.MissionStatus)

    rospy.loginfo("Setting local position reference frame...")
    resp = call_service_with_retry(
        set_local_pos_srv,
        dji_srv.SetLocalPosRefRequest(),
        "set_local_pos_ref",
        SET_LOCAL_POS_RETRIES,
        SET_LOCAL_POS_DELAY_SEC,
        hint="check /dji_sdk/gps_health>3, multipath, indoors",
    )
    if resp is None or not resp.result:
        rospy.logfatal("set_local_pos_ref failed after %d retries.", SET_LOCAL_POS_RETRIES)
        sys.exit(1)

    rospy.loginfo("Requesting SDK control authority...")
    resp = call_service_with_retry(
        ctrl_auth_srv,
        dji_srv.SDKControlAuthorityRequest(control_enable=dji_srv.SDKControlAuthorityRequest.REQUEST_CONTROL),
        "sdk_control_authority",
        CTRL_AUTH_RETRIES,
        CTRL_AUTH_DELAY_SEC,
        hint="RC switch must be in F-mode (Onboard)",
    )
    if resp is None or not resp.result:
        rospy.logfatal("Could not obtain SDK control authority.")
        sys.exit(1)

    if args.skip_clear:
        rospy.logwarn("--skip-clear: skipping waypoint stack clear.")
    else:
        first_lat, first_lon, first_alt, _ = waypoints[0]
        clear_waypoint_stack(
            upload_srv,
            action_srv,
            mission_status_srv,
            first_lat,
            first_lon,
            first_alt,
            args.speed,
            args.clear_timeout,
        )

    rospy.loginfo("=== UPLOAD NEW MISSION ===")
    task = build_waypoint_task(waypoints, args.speed)
    up_resp = upload_mission_with_retry(upload_srv, action_srv, ctrl_auth_srv, task)
    if up_resp is None or not up_resp.result:
        ack = up_resp.ack_data if up_resp is not None else "n/a"
        rospy.logfatal("Mission upload failed after %d retries (last ack=%s).", UPLOAD_RETRY_ATTEMPTS, ack)
        sys.exit(1)
    rospy.loginfo("=== UPLOAD NEW MISSION done ===")

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

    arrival_times = track_waypoints(waypoints, args.arrive_radius, mission_start)
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
