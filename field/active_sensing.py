#!/usr/bin/env python3
"""
Read drone position from the DJI SDK, convert attitude to yaw, and send a
WaypointList protobuf to Spot over TCP. Spot handles mission stack priority.

Requires dji_sdk_node (mission bringup) publishing:
  /dji_sdk/gps_position
  /dji_sdk/attitude
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "protobuf"))
import waypoints_pb2

DEFAULT_SPOT_ADDR = ("192.168.2.26", 9000)
DEFAULT_TELEMETRY_WAIT_SEC = 60.0
GPS_TOPIC = "/dji_sdk/gps_position"
ATTITUDE_TOPIC = "/dji_sdk/attitude"


class DroneTelemetry:
    """Cache latest DJI GPS + attitude from ROS subscribers."""

    def __init__(self) -> None:
        self._lat: float | None = None
        self._lon: float | None = None
        self._alt: float | None = None
        self._attitude: tuple[float, float, float, float] | None = None

    @property
    def ready(self) -> bool:
        return self._lat is not None and self._attitude is not None

    def wait_ready(self, timeout_sec: float) -> bool:
        import rospy

        deadline = time.monotonic() + timeout_sec
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.ready:
                return True
            rospy.sleep(0.1)
        return self.ready

    def snapshot(self) -> tuple[float, float, float, float]:
        if not self.ready:
            raise RuntimeError(
                f"DJI telemetry not ready — is dji_sdk_node running? "
                f"Check: rostopic echo -n1 {GPS_TOPIC}"
            )
        lat, lon = self._lat, self._lon
        w, x, y, z = self._attitude
        return lat, lon, attitude_to_yaw(w, x, y, z)

    def start_subscribers(self) -> None:
        from geometry_msgs.msg import QuaternionStamped
        from sensor_msgs.msg import NavSatFix
        import rospy

        def _gps_cb(msg: NavSatFix) -> None:
            self._lat = msg.latitude
            self._lon = msg.longitude
            self._alt = msg.altitude

        def _att_cb(msg: QuaternionStamped) -> None:
            q = msg.quaternion
            self._attitude = (q.w, q.x, q.y, q.z)

        rospy.Subscriber(GPS_TOPIC, NavSatFix, _gps_cb, queue_size=1)
        rospy.Subscriber(ATTITUDE_TOPIC, QuaternionStamped, _att_cb, queue_size=1)


def attitude_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Quaternion → yaw (same convention as entire_pipeline spot waypoints)."""
    yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.pi / 2.0 - yaw_rad


def send_drone_waypoint(
    lat: float,
    lon: float,
    yaw: float,
    spot_addr: tuple[str, int] = DEFAULT_SPOT_ADDR,
) -> None:
    msg = waypoints_pb2.WaypointList()
    msg.timestamp_ns = time.time_ns()
    wp = msg.waypoints.add()
    wp.lat, wp.lon, wp.yaw = lat, lon, yaw

    payload = msg.SerializeToString()
    with socket.create_connection(spot_addr, timeout=5.0) as sock:
        sock.sendall(len(payload).to_bytes(4, "big") + payload)


def _wait_for_telemetry(telemetry: DroneTelemetry, wait_sec: float) -> None:
    import rospy

    rospy.loginfo("Waiting for %s and %s (timeout %.0fs)...", GPS_TOPIC, ATTITUDE_TOPIC, wait_sec)
    if telemetry.wait_ready(wait_sec):
        lat, lon, yaw = telemetry.snapshot()
        rospy.loginfo("Telemetry OK: lat=%.8f lon=%.8f yaw=%.4f", lat, lon, yaw)
        return

    rospy.logfatal(
        "Timed out waiting for DJI telemetry. Start mission bringup first, e.g.\n"
        "  roslaunch drone_bringup bringup_synchronized.launch ...\n"
        "Then verify:\n"
        "  rostopic echo -n1 %s\n"
        "  rostopic echo -n1 %s",
        GPS_TOPIC,
        ATTITUDE_TOPIC,
    )
    sys.exit(1)


def run_once(spot_addr: tuple[str, int], telemetry_wait_sec: float) -> None:
    import rospy

    rospy.init_node("active_sensing", anonymous=True)
    telemetry = DroneTelemetry()
    telemetry.start_subscribers()
    _wait_for_telemetry(telemetry, telemetry_wait_sec)

    lat, lon, yaw = telemetry.snapshot()
    send_drone_waypoint(lat, lon, yaw, spot_addr=spot_addr)
    print(f"Sent drone waypoint to Spot: lat={lat:.8f} lon={lon:.8f} yaw={yaw:.4f} rad")


def run_stream(spot_addr: tuple[str, int], rate_hz: float, telemetry_wait_sec: float) -> None:
    import rospy

    rospy.init_node("active_sensing", anonymous=False)
    telemetry = DroneTelemetry()
    telemetry.start_subscribers()
    _wait_for_telemetry(telemetry, telemetry_wait_sec)

    period = 1.0 / rate_hz
    rospy.loginfo("active_sensing: sending drone position to Spot at %.2f Hz", rate_hz)

    while not rospy.is_shutdown():
        t0 = time.monotonic()
        try:
            lat, lon, yaw = telemetry.snapshot()
            send_drone_waypoint(lat, lon, yaw, spot_addr=spot_addr)
            rospy.loginfo("Sent drone waypoint: lat=%.8f lon=%.8f yaw=%.4f", lat, lon, yaw)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "active_sensing: send failed: %s", exc)
        elapsed = time.monotonic() - t0
        sleep = max(0.0, period - elapsed)
        if sleep:
            rospy.sleep(sleep)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send current drone GPS + yaw to Spot as a WaypointList protobuf",
    )
    parser.add_argument("--spot-host", default=DEFAULT_SPOT_ADDR[0])
    parser.add_argument("--spot-port", type=int, default=DEFAULT_SPOT_ADDR[1])
    parser.add_argument(
        "--telemetry-wait",
        type=float,
        default=DEFAULT_TELEMETRY_WAIT_SEC,
        help="Seconds to wait for DJI topics at startup (default: %.0f)" % DEFAULT_TELEMETRY_WAIT_SEC,
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=0.0,
        metavar="HZ",
        help="If > 0, keep sending at this rate (Hz). Default: send once and exit.",
    )
    args = parser.parse_args()
    spot_addr = (args.spot_host, args.spot_port)

    if args.rate > 0:
        run_stream(spot_addr, args.rate, args.telemetry_wait)
    else:
        run_once(spot_addr, args.telemetry_wait)


if __name__ == "__main__":
    main()
