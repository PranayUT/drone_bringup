#!/usr/bin/env python3

import math
from typing import Optional

import rospy
from sensor_msgs.msg import NavSatFix

from dji_sdk.msg import MissionWaypoint, MissionWaypointTask
from dji_sdk.srv import MissionWpAction, MissionWpActionRequest, MissionWpUpload, MissionWpUploadRequest


def _finite(x: float) -> bool:
    return x is not None and not (math.isnan(x) or math.isinf(x))


class GpsTargetToDjiWaypointMission:
    def __init__(self) -> None:
        self.current_fix: Optional[NavSatFix] = None
        self.last_target_hash: Optional[int] = None

        self.current_fix_topic = rospy.get_param("~current_fix_topic", "/fix")
        self.target_fix_topic = rospy.get_param("~target_fix_topic", "/gps_target")

        self.relative_alt_m = float(rospy.get_param("~relative_alt_m", 20.0))
        self.idle_velocity = float(rospy.get_param("~idle_velocity", 2.0))
        self.velocity_range = float(rospy.get_param("~velocity_range", 5.0))
        self.finish_action = int(rospy.get_param("~finish_action", int(MissionWaypointTask.FINISH_NO_ACTION)))
        self.trace_mode = int(rospy.get_param("~trace_mode", int(MissionWaypointTask.TRACE_POINT)))
        self.yaw_mode = int(rospy.get_param("~yaw_mode", int(MissionWaypointTask.YAW_MODE_AUTO)))

        self.auto_start = bool(rospy.get_param("~auto_start", True))
        self.include_current_as_wp0 = bool(rospy.get_param("~include_current_as_wp0", True))
        self.allow_target_altitude = bool(rospy.get_param("~allow_target_altitude", False))

        self.upload_service = rospy.get_param("~upload_service", "dji_sdk/mission_waypoint_upload")
        self.action_service = rospy.get_param("~action_service", "dji_sdk/mission_waypoint_action")

        rospy.loginfo("Subscribing current fix: %s", self.current_fix_topic)
        rospy.loginfo("Subscribing target fix:  %s", self.target_fix_topic)
        rospy.loginfo("Using DJI services upload=%s action=%s", self.upload_service, self.action_service)

        self._upload = rospy.ServiceProxy(self.upload_service, MissionWpUpload)
        self._action = rospy.ServiceProxy(self.action_service, MissionWpAction)

        rospy.Subscriber(self.current_fix_topic, NavSatFix, self._on_current_fix, queue_size=10)
        rospy.Subscriber(self.target_fix_topic, NavSatFix, self._on_target_fix, queue_size=10)

    def _on_current_fix(self, msg: NavSatFix) -> None:
        self.current_fix = msg

    def _on_target_fix(self, msg: NavSatFix) -> None:
        if not (_finite(msg.latitude) and _finite(msg.longitude)):
            rospy.logwarn_throttle(5.0, "Target fix lat/lon not finite; ignoring.")
            return

        h = hash((round(float(msg.latitude), 7), round(float(msg.longitude), 7), round(float(msg.altitude), 2)))
        if self.last_target_hash == h:
            return
        self.last_target_hash = h

        if self.include_current_as_wp0 and self.current_fix is None:
            rospy.logwarn("No current fix yet on %s; can't build waypoint mission.", self.current_fix_topic)
            return

        try:
            rospy.wait_for_service(self.upload_service, timeout=2.0)
            rospy.wait_for_service(self.action_service, timeout=2.0)
        except rospy.ROSException:
            rospy.logerr("DJI mission services not available (is `dji_sdk` running?)")
            return

        task = MissionWaypointTask()
        task.velocity_range = self.velocity_range
        task.idle_velocity = self.idle_velocity
        task.action_on_finish = self.finish_action
        task.mission_exec_times = 1
        task.yaw_mode = self.yaw_mode
        task.trace_mode = self.trace_mode
        task.action_on_rc_lost = int(MissionWaypointTask.ACTION_AUTO)
        task.gimbal_pitch_mode = int(MissionWaypointTask.GIMBAL_PITCH_FREE)

        waypoints = []
        if self.include_current_as_wp0 and self.current_fix is not None:
            waypoints.append(self._mk_waypoint(self.current_fix.latitude, self.current_fix.longitude, self.relative_alt_m))

        # Waypoint mission altitude is interpreted (OSDK ROS 3.8) as meters above takeoff point,
        # so do not feed GPS MSL altitude here unless your publisher already converted it.
        if self.allow_target_altitude and _finite(msg.altitude) and abs(float(msg.altitude)) >= 1e-3:
            alt = float(msg.altitude)
        else:
            if _finite(msg.altitude) and abs(float(msg.altitude)) >= 1e-3:
                rospy.logwarn_throttle(
                    5.0,
                    "Target fix altitude=%.2f provided but ignored; using relative_alt_m=%.2f (meters above takeoff).",
                    float(msg.altitude),
                    self.relative_alt_m,
                )
            alt = self.relative_alt_m
        waypoints.append(self._mk_waypoint(msg.latitude, msg.longitude, alt))
        task.mission_waypoint = waypoints

        req = MissionWpUploadRequest()
        req.waypoint_task = task

        rospy.loginfo(
            "Uploading waypoint mission with %d wp(s). Target=(%.7f, %.7f, alt=%.2f)",
            len(waypoints),
            msg.latitude,
            msg.longitude,
            alt,
        )

        try:
            resp = self._upload(req)
        except rospy.ServiceException as e:
            rospy.logerr("Mission upload failed: %s", e)
            return

        if not resp.result:
            rospy.logerr("Mission upload rejected (cmd_set=%s cmd_id=%s ack=%s)", resp.cmd_set, resp.cmd_id, resp.ack_data)
            return

        rospy.loginfo("Mission upload OK.")

        if self.auto_start:
            try:
                a = MissionWpActionRequest()
                a.action = int(MissionWpActionRequest.ACTION_START)
                aresp = self._action(a)
            except rospy.ServiceException as e:
                rospy.logerr("Mission start failed: %s", e)
                return

            if not aresp.result:
                rospy.logerr("Mission start rejected (cmd_set=%s cmd_id=%s ack=%s)", aresp.cmd_set, aresp.cmd_id, aresp.ack_data)
                return
            rospy.loginfo("Mission started.")

    @staticmethod
    def _mk_waypoint(lat: float, lon: float, rel_alt_m: float) -> MissionWaypoint:
        wp = MissionWaypoint()
        wp.latitude = float(lat)
        wp.longitude = float(lon)
        wp.altitude = float(rel_alt_m)
        wp.damping_distance = 0.0
        wp.target_yaw = 0
        wp.target_gimbal_pitch = 0
        wp.turn_mode = 0
        wp.has_action = 0
        wp.action_time_limit = 0
        return wp


def main() -> None:
    rospy.init_node("gps_target_to_dji_waypoint_mission", anonymous=False)
    GpsTargetToDjiWaypointMission()
    rospy.loginfo("gps_target_to_dji_waypoint_mission ready.")
    rospy.spin()


if __name__ == "__main__":
    main()

