#!/usr/bin/env python3
"""
DJI waypoint mission runner - loiter orbit variant.

Reads lat,lon,alt waypoints from a text file, uploads a straight-line mission
to the DJI SDK, starts it, and loiters at each waypoint after arrival.
Orbit radius at each waypoint is locked from live height above takeoff on arrival.

Usage:
    python waypoint_runner2.py <waypoints_file> [--speed <m/s>] [--arrive-radius <m>] [--loiter <s>]

Waypoints file format (one waypoint per line, blank/# lines ignored):
    lat, lon, alt
    30.1234, -97.5678, 20.0

Requirements: ROS Noetic + dji_sdk running
    source /opt/ros/noetic/setup.bash
    source <catkin_ws>/devel/setup.bash
"""

import sys
import math
import time
import argparse
import rospy
from std_msgs.msg import UInt8, Float32
from sensor_msgs.msg import NavSatFix
import dji_sdk.srv as dji_srv
import dji_sdk.msg as dji_msg

# DJI FlightStatus values (VehicleStatus::FlightStatus)
FLIGHT_STATUS_ON_GROUND = 1
FLIGHT_STATUS_IN_AIR    = 2

DEFAULT_ARRIVE_RADIUS_M = 1.0
DEFAULT_IDLE_SPEED      = 5.0   # m/s
DEFAULT_LOITER_SECS     = 10.0  # seconds to orbit at each waypoint
MIN_ORBIT_RADIUS        = 2.0   # metres - floor when live height is unavailable or tiny
ARRIVE_SAMPLES_REQUIRED = 3     # consecutive GPS samples inside arrive radius
MIN_NAV_SEC_BEFORE_LOITER = 3.0 # ignore early GPS hits before the mission has flown
MISSION_IN_PROGRESS_ACK = 213   # DJI MissionACK::Common::IN_PROGRESS (0xD5)
MISSION_OBTAIN_CONTROL_ACK = 209  # DJI MissionACK::Common::OBTAIN_CONTROL_REQUIRED (0xD1)
MISSION_DATA_NOT_ENOUGH_ACK = 234  # DJI MissionACK::WayPoint::DATA_NOT_ENOUGH (0xEA)
MAX_HOTPOINT_YAW_RATE_DEG_S = 15.0  # DJI OSDK default hotpoint yaw rate limit
MISSION_STOP_SETTLE_SECS  = 5.0
LOITER_ARM_DELAY_SECS     = 2.0
HOTPOINT_RETRY_ATTEMPTS   = 3
HOTPOINT_RETRY_DELAY_SECS = 2.0
UPLOAD_RETRY_ATTEMPTS   = 5
UPLOAD_RETRY_DELAY_SECS = 2.0
EARTH_R                 = 6_371_000.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two WGS84 points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def parse_waypoints(path):
    """Return list of (lat, lon, alt) tuples from a text file."""
    waypoints = []
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 3:
                rospy.logwarn("Line %d: expected 'lat,lon,alt' - skipping: %s", lineno, raw.rstrip())
                continue
            try:
                waypoints.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                rospy.logwarn("Line %d: could not parse floats - skipping: %s", lineno, raw.rstrip())
    return waypoints


def build_waypoint_task(waypoints, idle_speed):
    """Build a dji_sdk/MissionWaypointTask for straight-line point-to-point flight."""
    task = dji_msg.MissionWaypointTask()
    task.velocity_range    = 15.0
    task.idle_velocity     = float(idle_speed)
    task.action_on_finish  = dji_msg.MissionWaypointTask.FINISH_NO_ACTION
    task.mission_exec_times = 1
    task.yaw_mode          = dji_msg.MissionWaypointTask.YAW_MODE_AUTO
    task.trace_mode        = dji_msg.MissionWaypointTask.TRACE_POINT
    task.action_on_rc_lost = dji_msg.MissionWaypointTask.ACTION_AUTO
    task.gimbal_pitch_mode = dji_msg.MissionWaypointTask.GIMBAL_PITCH_FREE

    for lat, lon, alt in waypoints:
        wp = dji_msg.MissionWaypoint()
        wp.latitude           = lat
        wp.longitude          = lon
        wp.altitude           = float(alt)
        wp.damping_distance   = 0.0
        wp.target_yaw         = 0
        wp.target_gimbal_pitch = 0
        wp.turn_mode          = 0
        wp.has_action         = 0
        wp.action_time_limit  = 0
        action = dji_msg.MissionWaypointAction()
        action.action_repeat     = 0
        action.command_list      = [0] * 16
        action.command_parameter = [0] * 16
        wp.waypoint_action = action
        task.mission_waypoint.append(wp)

    return task


def hotpoint_yaw_rate_deg_s(linear_speed, radius_m):
    """Convert tangential speed to DJI hotpoint yaw rate, clamped to FC limits."""
    if radius_m <= 0.0:
        return MAX_HOTPOINT_YAW_RATE_DEG_S
    yaw_rate = math.degrees(float(linear_speed) / float(radius_m))
    if yaw_rate > MAX_HOTPOINT_YAW_RATE_DEG_S:
        rospy.logwarn(
            "Hotpoint yaw rate %.1f deg/s exceeds %.1f deg/s; clamping.",
            yaw_rate, MAX_HOTPOINT_YAW_RATE_DEG_S
        )
        return MAX_HOTPOINT_YAW_RATE_DEG_S
    return yaw_rate


def build_hotpoint_task(center_lat, center_lon, alt_m, radius, linear_speed, clockwise=True):
    """Build a dji_sdk/MissionHotpointTask for a circular loiter."""
    task = dji_msg.MissionHotpointTask()
    task.latitude = float(center_lat)
    task.longitude = float(center_lon)
    task.altitude = float(alt_m)
    task.radius = float(radius)
    task.angular_speed = float(hotpoint_yaw_rate_deg_s(linear_speed, radius))
    task.is_clockwise = 1 if clockwise else 0
    task.start_point = 4  # VIEW_NEARBY, DJI OSDK default
    task.yaw_mode = 1       # YAW_INSIDE, DJI OSDK default
    return task


def continuation_waypoints(waypoints, current_index):
    """Build the next straight-line segment after a loiter stop."""
    remaining = waypoints[current_index + 1:]
    if not remaining:
        return []
    if len(remaining) >= 2:
        return remaining
    if not _state.gps_valid:
        rospy.logwarn("Only one waypoint left but no GPS to seed continuation leg.")
        return remaining
    cur_lat = _state.gps_lat
    cur_lon = _state.gps_lon
    cur_alt = float(_state.height_above_takeoff) if _state.height_valid else remaining[0][2]
    return [(cur_lat, cur_lon, cur_alt), remaining[0]]


def run_hotpoint_loiter(center_lat, center_lon, alt_m, radius_m, linear_speed, loiter_secs, clockwise=True):
    """Run a DJI hotpoint orbit for loiter_secs at a fixed radius and altitude."""
    task = build_hotpoint_task(center_lat, center_lon, alt_m, radius_m, linear_speed, clockwise)
    rospy.loginfo(
        "  Hotpoint loiter: radius=%.1f m  alt=%.1f m  yaw=%.1f deg/s  loiter=%.1f s",
        radius_m, alt_m, task.angular_speed, loiter_secs
    )

    rospy.wait_for_service("dji_sdk/mission_hotpoint_upload", timeout=15.0)
    upload_srv = rospy.ServiceProxy("dji_sdk/mission_hotpoint_upload", dji_srv.MissionHpUpload)
    rospy.wait_for_service("dji_sdk/mission_hotpoint_action", timeout=10.0)
    hp_action_srv = rospy.ServiceProxy("dji_sdk/mission_hotpoint_action", dji_srv.MissionHpAction)

    for attempt in range(1, HOTPOINT_RETRY_ATTEMPTS + 1):
        stop_active_missions()
        if attempt > 1:
            rospy.logwarn("Retrying hotpoint loiter (attempt %d/%d)...", attempt, HOTPOINT_RETRY_ATTEMPTS)

        upload_resp = upload_srv(task)
        if not upload_resp.result:
            rospy.logwarn("Hotpoint upload failed (ack_data=%d).", upload_resp.ack_data)
            if attempt == HOTPOINT_RETRY_ATTEMPTS:
                return False
            rospy.sleep(HOTPOINT_RETRY_DELAY_SECS)
            continue

        start_resp = hp_action_srv(dji_srv.MissionHpActionRequest.ACTION_START)
        if start_resp.result:
            break

        rospy.logwarn("Hotpoint start failed (ack_data=%d).", start_resp.ack_data)
        if attempt == HOTPOINT_RETRY_ATTEMPTS:
            return False
        rospy.sleep(HOTPOINT_RETRY_DELAY_SECS)

    rospy.loginfo("  Hotpoint orbit running for %.1f s...", loiter_secs)
    rospy.sleep(loiter_secs)

    stop_resp = hp_action_srv(dji_srv.MissionHpActionRequest.ACTION_STOP)
    if not stop_resp.result:
        rospy.logwarn("Hotpoint stop failed (ack_data=%d).", stop_resp.ack_data)
        return False

    rospy.loginfo("  Hotpoint orbit complete.")
    return True


def orbit_radius_from_drone(min_radius=MIN_ORBIT_RADIUS):
    """Use live height above takeoff as the horizontal orbit radius."""
    alt = _state.height_above_takeoff
    if alt is None or not math.isfinite(alt):
        rospy.logwarn("No height_above_takeoff - using %.1f m orbit radius.", min_radius)
        return min_radius
    if alt < min_radius:
        rospy.logwarn(
            "Height %.2f m below minimum orbit radius %.1f m - clamping.",
            alt, min_radius
        )
        return min_radius
    return float(alt)


# ---------------------------------------------------------------------------
# ROS state holders (written by subscribers, read by main thread)
# ---------------------------------------------------------------------------

class DroneState:
    def __init__(self):
        self.flight_status = None
        self.gps_lat       = None
        self.gps_lon       = None
        self.gps_valid     = False
        self.height_above_takeoff = None
        self.height_valid  = False

_state = DroneState()


def _flight_status_cb(msg):
    _state.flight_status = msg.data


def _gps_cb(msg):
    if msg.status.status >= 0:
        _state.gps_lat   = msg.latitude
        _state.gps_lon   = msg.longitude
        _state.gps_valid = True


def _height_cb(msg):
    _state.height_above_takeoff = float(msg.data)
    _state.height_valid = True


# ---------------------------------------------------------------------------
# Main logic
# ---------------------------------------------------------------------------

def wait_for_flight_status(timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _state.flight_status is not None:
            return True
        time.sleep(0.1)
    return False


def wait_for_gps(timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _state.gps_valid:
            return True
        time.sleep(0.1)
    return False


def wait_for_height(timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _state.height_valid:
            return True
        time.sleep(0.1)
    return False


def ensure_sdk_mission_control():
    """Set local position reference and request SDK control before mission upload."""
    rospy.loginfo("Setting local position reference frame...")
    rospy.wait_for_service("dji_sdk/set_local_pos_ref", timeout=10.0)
    set_ref = rospy.ServiceProxy("dji_sdk/set_local_pos_ref", dji_srv.SetLocalPosRef)
    if not set_ref().result:
        rospy.logfatal("set_local_pos_ref failed.")
        sys.exit(1)

    rospy.loginfo("Requesting SDK control authority...")
    rospy.wait_for_service("dji_sdk/sdk_control_authority", timeout=10.0)
    control_srv = rospy.ServiceProxy("dji_sdk/sdk_control_authority", dji_srv.SDKControlAuthority)
    if not control_srv(dji_srv.SDKControlAuthorityRequest.REQUEST_CONTROL).result:
        rospy.logfatal("SDK control authority request failed.")
        sys.exit(1)
    rospy.loginfo("SDK control authority obtained.")


def stop_active_missions():
    """Stop any waypoint or hotpoint mission left over from a prior run."""
    try:
        rospy.wait_for_service("dji_sdk/mission_waypoint_action", timeout=5.0)
        wp_action_srv = rospy.ServiceProxy("dji_sdk/mission_waypoint_action", dji_srv.MissionWpAction)
        wp_action_srv(dji_srv.MissionWpActionRequest.ACTION_STOP)
    except rospy.ROSException:
        rospy.logwarn("Waypoint stop service unavailable while clearing mission state.")

    try:
        rospy.wait_for_service("dji_sdk/mission_hotpoint_action", timeout=5.0)
        hp_action_srv = rospy.ServiceProxy("dji_sdk/mission_hotpoint_action", dji_srv.MissionHpAction)
        hp_action_srv(dji_srv.MissionHpActionRequest.ACTION_STOP)
    except rospy.ROSException:
        rospy.logwarn("Hotpoint stop service unavailable while clearing mission state.")

    rospy.sleep(MISSION_STOP_SETTLE_SECS)


def upload_waypoint_mission(task):
    """Upload a waypoint mission, retrying while the FC reports IN_PROGRESS."""
    rospy.wait_for_service("dji_sdk/mission_waypoint_upload", timeout=15.0)
    upload_srv = rospy.ServiceProxy("dji_sdk/mission_waypoint_upload", dji_srv.MissionWpUpload)

    for attempt in range(1, UPLOAD_RETRY_ATTEMPTS + 1):
        rospy.loginfo("Uploading %d waypoints (attempt %d/%d)...",
                        len(task.mission_waypoint), attempt, UPLOAD_RETRY_ATTEMPTS)
        upload_resp = upload_srv(task)
        if upload_resp.result:
            return upload_resp

        if upload_resp.ack_data == MISSION_OBTAIN_CONTROL_ACK:
            rospy.logwarn("Waypoint upload needs SDK control (ack_data=209); requesting authority.")
            ensure_sdk_mission_control()
        elif upload_resp.ack_data in (MISSION_IN_PROGRESS_ACK, MISSION_DATA_NOT_ENOUGH_ACK):
            rospy.logwarn(
                "Waypoint upload retry (ack_data=%d); clearing missions and waiting.",
                upload_resp.ack_data
            )
            stop_active_missions()
        else:
            return upload_resp

        rospy.sleep(UPLOAD_RETRY_DELAY_SECS)

    return upload_resp


def track_waypoints(waypoints, arrive_radius, mission_start):
    """Monitor GPS and record arrival time for each uploaded waypoint."""
    arrival_times = [None] * len(waypoints)
    next_wp = 0
    total = len(waypoints)

    rospy.loginfo("Tracking %d waypoints (arrive radius=%.1f m)...", total, arrive_radius)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and next_wp < total:
        if not _state.gps_valid:
            rate.sleep()
            continue

        lat, lon = _state.gps_lat, _state.gps_lon
        wp_lat, wp_lon, _ = waypoints[next_wp]
        dist = haversine_m(lat, lon, wp_lat, wp_lon)

        if dist <= arrive_radius:
            arrival_times[next_wp] = time.time()
            elapsed = arrival_times[next_wp] - mission_start
            if next_wp == 0:
                leg_time = elapsed
            else:
                leg_time = arrival_times[next_wp] - arrival_times[next_wp - 1]
            print("[WP %d/%d] reached  |  leg: %.2f s  |  total: %.2f s"
                  % (next_wp + 1, total, leg_time, elapsed))
            next_wp += 1

        rate.sleep()

    return arrival_times


def track_and_loiter_waypoints(waypoints, arrive_radius, loiter_secs, orbit_speed, mission_start, action_srv):
    """
    Monitor GPS, stop the waypoint mission at each arrival, loiter, then upload
    and start the remaining straight-line segment.
    """
    arrival_times = [None] * len(waypoints)
    next_wp = 0
    total = len(waypoints)
    dwell_count = 0

    rospy.loginfo(
        "Tracking %d waypoints (arrive radius=%.1f m, loiter=%.1f s)...",
        total, arrive_radius, loiter_secs
    )

    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and next_wp < total:
        if not _state.gps_valid:
            rate.sleep()
            continue

        lat, lon = _state.gps_lat, _state.gps_lon
        wp_lat, wp_lon, wp_alt = waypoints[next_wp]
        dist = haversine_m(lat, lon, wp_lat, wp_lon)

        if dist <= arrive_radius:
            dwell_count += 1
        else:
            dwell_count = 0

        if dwell_count < ARRIVE_SAMPLES_REQUIRED:
            rate.sleep()
            continue

        if (time.time() - mission_start) < MIN_NAV_SEC_BEFORE_LOITER:
            rate.sleep()
            continue

        arrival_times[next_wp] = time.time()
        elapsed = arrival_times[next_wp] - mission_start
        if next_wp == 0:
            leg_time = elapsed
        else:
            leg_time = arrival_times[next_wp] - arrival_times[next_wp - 1]
        print("[WP %d/%d] reached  |  leg: %.2f s  |  total: %.2f s"
              % (next_wp + 1, total, leg_time, elapsed))

        locked_radius = orbit_radius_from_drone()
        locked_alt = float(_state.height_above_takeoff if _state.height_valid else wp_alt)
        rospy.loginfo(
            "  Locked loiter circle: radius=%.1f m  alt=%.1f m",
            locked_radius, locked_alt
        )

        rospy.sleep(LOITER_ARM_DELAY_SECS)
        center_lat = _state.gps_lat if _state.gps_valid else wp_lat
        center_lon = _state.gps_lon if _state.gps_valid else wp_lon
        loiter_ok = run_hotpoint_loiter(
            center_lat, center_lon, locked_alt, locked_radius, orbit_speed, loiter_secs, clockwise=True
        )
        if not loiter_ok:
            rospy.logwarn("Loiter circle failed at WP %d/%d.", next_wp + 1, total)

        if next_wp < total - 1:
            segment = continuation_waypoints(waypoints, next_wp)
            if len(segment) < 2:
                rospy.logfatal(
                    "Cannot continue toward WP %d/%d with fewer than 2 waypoints.",
                    next_wp + 2, total
                )
                break
            stop_active_missions()
            upload_resp = upload_waypoint_mission(build_waypoint_task(segment, orbit_speed))
            if not upload_resp.result:
                rospy.logfatal(
                    "Waypoint re-upload failed at WP %d (ack_data=%d).",
                    next_wp + 1, upload_resp.ack_data
                )
                break
            start_resp = action_srv(dji_srv.MissionWpActionRequest.ACTION_START)
            if not start_resp.result:
                rospy.logfatal(
                    "Mission start failed toward WP %d/%d (ack_data=%d).",
                    next_wp + 2, total, start_resp.ack_data
                )
                break
            rospy.loginfo("Mission started toward WP %d/%d.", next_wp + 2, total)

        next_wp += 1
        dwell_count = 0
        rate.sleep()

    return arrival_times


def main():
    parser = argparse.ArgumentParser(description="DJI waypoint mission runner - loiter orbit")
    parser.add_argument("waypoints_file", help="Path to waypoints text file (lat,lon,alt per line)")
    parser.add_argument("--speed",         type=float, default=DEFAULT_IDLE_SPEED,
                        help="Cruise/orbit speed in m/s (default: %.1f)" % DEFAULT_IDLE_SPEED)
    parser.add_argument("--arrive-radius", type=float, default=DEFAULT_ARRIVE_RADIUS_M,
                        help="Arrival detection radius in metres (default: %.1f)" % DEFAULT_ARRIVE_RADIUS_M)
    parser.add_argument("--loiter",        type=float, default=DEFAULT_LOITER_SECS,
                        help="Seconds to orbit at each waypoint (default: %.1f)" % DEFAULT_LOITER_SECS)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("waypoint_runner2", anonymous=True)

    waypoints = parse_waypoints(args.waypoints_file)
    if len(waypoints) < 2:
        rospy.logfatal("Need at least 2 waypoints; got %d. Aborting.", len(waypoints))
        sys.exit(1)
    rospy.loginfo("Loaded %d waypoints from %s", len(waypoints), args.waypoints_file)
    for i, (lat, lon, alt) in enumerate(waypoints):
        rospy.loginfo("  WP%d: lat=%.7f  lon=%.7f  alt=%.1f m", i + 1, lat, lon, alt)

    rospy.Subscriber("dji_sdk/flight_status", UInt8, _flight_status_cb, queue_size=1)
    rospy.Subscriber("dji_sdk/gps_position",  NavSatFix, _gps_cb, queue_size=1)
    rospy.Subscriber("dji_sdk/height_above_takeoff", Float32, _height_cb, queue_size=1)

    rospy.loginfo("Waiting for flight status...")
    if not wait_for_flight_status(timeout=10.0):
        rospy.logfatal("No flight_status received in 10 s. Is dji_sdk running?")
        sys.exit(1)

    rospy.loginfo("Waiting for GPS fix...")
    if not wait_for_gps(timeout=10.0):
        rospy.logfatal("No GPS fix received in 10 s.")
        sys.exit(1)

    rospy.loginfo("Waiting for height above takeoff...")
    if not wait_for_height(timeout=10.0):
        rospy.logfatal("No height_above_takeoff received in 10 s.")
        sys.exit(1)

    if _state.flight_status != FLIGHT_STATUS_IN_AIR:
        rospy.logfatal(
            "Drone is NOT in the air (flight_status=%s, expected %d for IN_AIR). "
            "Take off first, then re-run.",
            _state.flight_status, FLIGHT_STATUS_IN_AIR
        )
        sys.exit(1)
    rospy.loginfo("Drone is airborne. Proceeding with mission upload.")

    ensure_sdk_mission_control()
    stop_active_missions()

    upload_resp = upload_waypoint_mission(build_waypoint_task(waypoints, args.speed))
    if not upload_resp.result:
        rospy.logfatal(
            "Waypoint upload FAILED (ack_data=%d, 234=DATA_NOT_ENOUGH). Check dji_sdk logs.",
            upload_resp.ack_data,
        )
        sys.exit(1)
    rospy.loginfo("Upload successful.")

    action_srv = rospy.ServiceProxy("dji_sdk/mission_waypoint_action", dji_srv.MissionWpAction)

    rospy.loginfo("Starting mission...")
    action_resp = action_srv(dji_srv.MissionWpActionRequest.ACTION_START)
    if not action_resp.result:
        rospy.logfatal("Mission start FAILED (ack_data=%d).", action_resp.ack_data)
        sys.exit(1)

    mission_start = time.time()
    rospy.loginfo("Mission started at t=0.00 s")
    print("\n%-10s  %-12s  %-12s" % ("Waypoint", "Leg time", "Total time"))
    print("-" * 38)

    arrival_times = track_waypoints(waypoints, args.arrive_radius, mission_start)
    # Hotpoint loiter disabled: use track_and_loiter_waypoints(...) to orbit at each waypoint.
    # arrival_times = track_and_loiter_waypoints(
    #     waypoints, args.arrive_radius, args.loiter, args.speed, mission_start, action_srv
    # )

    reached = sum(1 for t in arrival_times if t is not None)
    total_time = (arrival_times[-1] - mission_start) if arrival_times[-1] else (time.time() - mission_start)

    print("\n=== Mission Summary ===")
    print("Waypoints reached : %d / %d" % (reached, len(waypoints)))
    print("Total mission time: %.2f s" % total_time)
    if reached >= 2:
        print("\nPer-leg breakdown:")
        prev = mission_start
        for i, t in enumerate(arrival_times):
            if t is None:
                print("  WP%d -> WP%d : --" % (i, i + 1))
            else:
                print("  WP%d -> WP%d : %.2f s" % (i, i + 1, t - prev))
                prev = t


if __name__ == "__main__":
    main()
