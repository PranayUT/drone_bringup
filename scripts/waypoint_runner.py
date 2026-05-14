#!/usr/bin/env python3
"""
DJI waypoint mission runner.

Reads lat,lon,alt waypoints from a text file, uploads a straight-line mission
to the DJI SDK, starts it, and prints per-leg and total elapsed times.

Usage:
    python waypoint_runner.py <waypoints_file> [--speed <m/s>] [--arrive-radius <m>]

Waypoints file format (one waypoint per line, blank/# lines ignored):
    lat, lon, alt
    30.1234, -97.5678, 20.0

Requirements: ROS Noetic + dji_sdk running
    source /opt/ros/noetic/setup.bash
    source ~/Documents/catkin_ws/devel/setup.bash
"""

import sys
import math
import time
import argparse
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import NavSatFix
import dji_sdk.srv as dji_srv
import dji_sdk.msg as dji_msg

# DJI FlightStatus values (VehicleStatus::FlightStatus)
FLIGHT_STATUS_ON_GROUND = 1
FLIGHT_STATUS_IN_AIR    = 2

# How close (metres) to a waypoint centre before we call it "reached"
DEFAULT_ARRIVE_RADIUS_M = 1.0
DEFAULT_IDLE_SPEED      = 5.0   # m/s


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
                rospy.logwarn("Line %d: expected 'lat,lon,alt' — skipping: %s", lineno, raw.rstrip())
                continue
            try:
                waypoints.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                rospy.logwarn("Line %d: could not parse floats — skipping: %s", lineno, raw.rstrip())
    return waypoints


def build_waypoint_task(waypoints, idle_speed):
    """Build a dji_sdk/MissionWaypointTask for straight-line point-to-point flight."""
    task = dji_msg.MissionWaypointTask()
    task.velocity_range    = 15.0
    task.idle_velocity     = float(idle_speed)
    task.action_on_finish  = dji_msg.MissionWaypointTask.FINISH_NO_ACTION
    task.mission_exec_times = 1
    task.yaw_mode          = dji_msg.MissionWaypointTask.YAW_MODE_AUTO
    task.trace_mode        = dji_msg.MissionWaypointTask.TRACE_POINT  # straight-line stop-and-go
    task.action_on_rc_lost = dji_msg.MissionWaypointTask.ACTION_AUTO
    task.gimbal_pitch_mode = dji_msg.MissionWaypointTask.GIMBAL_PITCH_FREE

    for lat, lon, alt in waypoints:
        wp = dji_msg.MissionWaypoint()
        wp.latitude           = lat
        wp.longitude          = lon
        wp.altitude           = float(10)
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


# ---------------------------------------------------------------------------
# ROS state holders (written by subscribers, read by main thread)
# ---------------------------------------------------------------------------

class DroneState:
    def __init__(self):
        self.flight_status = None   # UInt8 data
        self.gps_lat       = None
        self.gps_lon       = None
        self.gps_valid     = False

_state = DroneState()


def _flight_status_cb(msg):
    _state.flight_status = msg.data


def _gps_cb(msg):
    # NavSatFix: status.status >= 0 means fix
    if msg.status.status >= 0:
        _state.gps_lat   = msg.latitude
        _state.gps_lon   = msg.longitude
        _state.gps_valid = True


# ---------------------------------------------------------------------------
# Main logic
# ---------------------------------------------------------------------------

def wait_for_flight_status(timeout=10.0):
    """Block until we get at least one flight_status message."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _state.flight_status is not None:
            return True
        time.sleep(0.1)
    return False


def wait_for_gps(timeout=10.0):
    """Block until we have a valid GPS fix."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _state.gps_valid:
            return True
        time.sleep(0.1)
    return False


def track_waypoints(waypoints, arrive_radius, mission_start):
    """
    Monitor GPS and record the time each waypoint is 'reached'.
    Returns list of arrival timestamps parallel to waypoints[].
    Blocks until all waypoints are visited or rospy shuts down.
    """
    arrival_times = [None] * len(waypoints)
    next_wp = 0
    total = len(waypoints)
    last_log_time = 0.0

    rospy.loginfo("Tracking %d waypoints (arrive radius=%.1f m)...", total, arrive_radius)

    rate = rospy.Rate(5)  # 5 Hz poll
    while not rospy.is_shutdown() and next_wp < total:
        if not _state.gps_valid:
            rospy.logwarn_throttle(5.0, "Waiting for GPS fix...")
            rate.sleep()
            continue

        lat, lon = _state.gps_lat, _state.gps_lon
        wp_lat, wp_lon, wp_alt = waypoints[next_wp]
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
            rospy.loginfo(
                "WP %d/%d REACHED  |  leg: %.2f s  |  total: %.2f s",
                next_wp + 1, total, leg_time, elapsed
            )
            next_wp += 1
            if next_wp < total:
                rospy.loginfo("Next target: WP %d  (%.7f, %.7f, %.1f m)",
                              next_wp + 1, *waypoints[next_wp])

        rate.sleep()

    return arrival_times


def main():
    parser = argparse.ArgumentParser(description="DJI waypoint mission runner")
    parser.add_argument("waypoints_file", help="Path to waypoints text file (lat,lon,alt per line)")
    parser.add_argument("--speed",         type=float, default=DEFAULT_IDLE_SPEED,
                        help="Cruise speed in m/s (default: %.1f)" % DEFAULT_IDLE_SPEED)
    parser.add_argument("--arrive-radius", type=float, default=DEFAULT_ARRIVE_RADIUS_M,
                        help="Arrival detection radius in metres (default: %.1f)" % DEFAULT_ARRIVE_RADIUS_M)
    # rospy.myargv strips ROS remapping args (e.g. __name:=, __log:=) before argparse sees them
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # --- ROS init ---
    rospy.init_node("waypoint_runner", anonymous=True)
    rospy.loginfo("waypoint_runner started  |  file: %s  |  speed: %.1f m/s  |  arrive radius: %.1f m",
                  args.waypoints_file, args.speed, args.arrive_radius)

    # --- Parse waypoints ---
    rospy.loginfo("Loading waypoints from: %s", args.waypoints_file)
    waypoints = parse_waypoints(args.waypoints_file)
    if len(waypoints) < 2:
        rospy.logfatal("Need at least 2 waypoints; got %d. Aborting.", len(waypoints))
        sys.exit(1)
    rospy.loginfo("Loaded %d waypoints:", len(waypoints))
    for i, (lat, lon, alt) in enumerate(waypoints):
        rospy.loginfo("  WP%d: lat=%.7f  lon=%.7f  alt=%.1f m", i + 1, lat, lon, alt)

    # --- Subscribe to telemetry ---
    rospy.Subscriber("dji_sdk/flight_status", UInt8, _flight_status_cb, queue_size=1)
    rospy.Subscriber("dji_sdk/gps_position",  NavSatFix, _gps_cb,       queue_size=1)

    # --- Wait for telemetry ---
    rospy.loginfo("Waiting for flight_status topic...")
    if not wait_for_flight_status(timeout=10.0):
        rospy.logfatal("No flight_status received in 10 s — is dji_sdk running?")
        sys.exit(1)
    rospy.loginfo("flight_status OK (value=%s)", _state.flight_status)

    rospy.loginfo("Waiting for GPS fix...")
    if not wait_for_gps(timeout=10.0):
        rospy.logfatal("No GPS fix received in 10 s.")
        sys.exit(1)
    rospy.loginfo("GPS fix OK  (lat=%.7f, lon=%.7f)", _state.gps_lat, _state.gps_lon)

    # --- Check airborne ---
    if _state.flight_status != FLIGHT_STATUS_IN_AIR:
        rospy.logfatal(
            "Drone is NOT airborne (flight_status=%s, expected %d=IN_AIR). "
            "Take off first, then re-run.",
            _state.flight_status, FLIGHT_STATUS_IN_AIR
        )
        sys.exit(1)
    rospy.loginfo("Drone is airborne. Proceeding with mission upload.")

    # --- Set local position reference frame ---
    rospy.loginfo("Setting local position reference frame...")
    try:
        rospy.wait_for_service("dji_sdk/set_local_pos_ref", timeout=10.0)
    except rospy.ROSException:
        rospy.logfatal("Service dji_sdk/set_local_pos_ref not available after 10 s. Aborting.")
        sys.exit(1)
    set_local_pos_srv = rospy.ServiceProxy("dji_sdk/set_local_pos_ref", dji_srv.SetLocalPosRef)
    local_pos_resp = set_local_pos_srv()
    if not local_pos_resp.result:
        rospy.logfatal("set_local_pos_ref FAILED. Aborting.")
        sys.exit(1)
    rospy.loginfo("Local position reference set.")

    # --- Obtain SDK control authority ---
    rospy.loginfo("Requesting SDK control authority...")
    try:
        rospy.wait_for_service("dji_sdk/sdk_control_authority", timeout=10.0)
    except rospy.ROSException:
        rospy.logfatal("Service dji_sdk/sdk_control_authority not available after 10 s. Aborting.")
        sys.exit(1)
    control_srv = rospy.ServiceProxy("dji_sdk/sdk_control_authority", dji_srv.SDKControlAuthority)
    control_resp = control_srv(dji_srv.SDKControlAuthorityRequest.REQUEST_CONTROL)
    if not control_resp.result:
        rospy.logfatal("SDK control authority request FAILED (ack_data=%d). Aborting.", control_resp.ack_data)
        sys.exit(1)
    rospy.loginfo("SDK control authority obtained.")

    # --- Upload waypoints ---
    rospy.loginfo("Waiting for service: dji_sdk/mission_waypoint_upload ...")
    try:
        rospy.wait_for_service("dji_sdk/mission_waypoint_upload", timeout=15.0)
    except rospy.ROSException:
        rospy.logfatal("Service dji_sdk/mission_waypoint_upload not available after 15 s. Aborting.")
        sys.exit(1)

    upload_srv = rospy.ServiceProxy("dji_sdk/mission_waypoint_upload", dji_srv.MissionWpUpload)
    task = build_waypoint_task(waypoints, args.speed)
    rospy.loginfo("Uploading %d waypoints at %.1f m/s...", len(waypoints), args.speed)
    upload_resp = upload_srv(task)
    if not upload_resp.result:
        rospy.logfatal("Waypoint upload FAILED (ack_data=%d) — check dji_sdk logs.", upload_resp.ack_data)
        sys.exit(1)
    rospy.loginfo("Waypoint upload successful.")

    # --- Start mission ---
    rospy.loginfo("Waiting for service: dji_sdk/mission_waypoint_action ...")
    try:
        rospy.wait_for_service("dji_sdk/mission_waypoint_action", timeout=10.0)
    except rospy.ROSException:
        rospy.logfatal("Service dji_sdk/mission_waypoint_action not available after 10 s. Aborting.")
        sys.exit(1)

    action_srv = rospy.ServiceProxy("dji_sdk/mission_waypoint_action", dji_srv.MissionWpAction)
    rospy.loginfo("Sending ACTION_START...")
    action_resp = action_srv(dji_srv.MissionWpActionRequest.ACTION_START)
    if not action_resp.result:
        rospy.logfatal("Mission start FAILED (ack_data=%d).", action_resp.ack_data)
        sys.exit(1)

    mission_start = time.time()
    rospy.loginfo("Mission is running. Tracking %d waypoints...", len(waypoints))

    # --- Track waypoint arrivals ---
    arrival_times = track_waypoints(waypoints, args.arrive_radius, mission_start)

    # --- Final summary ---
    reached = sum(1 for t in arrival_times if t is not None)
    total_time = (arrival_times[-1] - mission_start) if arrival_times[-1] else (time.time() - mission_start)

    rospy.loginfo("=== Mission Complete ===")
    rospy.loginfo("Waypoints reached : %d / %d", reached, len(waypoints))
    rospy.loginfo("Total mission time: %.2f s", total_time)
    if reached >= 2:
        rospy.loginfo("Per-leg breakdown:")
        prev = mission_start
        for i, t in enumerate(arrival_times):
            if t is None:
                rospy.loginfo("  WP%d -> WP%d : not reached", i + 1, i + 2)
            else:
                rospy.loginfo("  WP%d -> WP%d : %.2f s", i + 1, i + 2, t - prev)
                prev = t


if __name__ == "__main__":
    main()
