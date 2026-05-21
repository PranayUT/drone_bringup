#!/usr/bin/env python3
"""
Active sensing: when the drone detects a person, send a priority waypoint to Spot.

Spot pushes each incoming WaypointList to the top of its mission stack; this node
only sends the waypoint (lat, lon, yaw). GPS and attitude come from the DJI SDK;
person location is projected from YOLO mask polygons published on /yolo/masks_json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pymap3d

# Protobuf + shared projection / sender from the field pipeline
_FIELD_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_FIELD_DIR.parent / "protobuf"))
sys.path.insert(0, str(_FIELD_DIR))

import waypoints_pb2  # noqa: E402
from entire_pipeline import masks_to_gps, proto_sender  # noqa: E402

PERSON_LABELS = frozenset({"person", "people", "human"})
DEFAULT_MASKS_TOPIC = "/yolo/masks_json"
DEFAULT_SPOT_ADDR = ("192.168.2.26", 9000)
DEFAULT_COOLDOWN_SEC = 15.0
DEFAULT_MIN_CONFIDENCE = 0.5


def _grab_gps(timeout: float = 10.0) -> list[float]:
    from sensor_msgs.msg import NavSatFix
    import rospy

    msg = rospy.wait_for_message("/dji_sdk/gps_position", NavSatFix, timeout=timeout)
    return [msg.latitude, msg.longitude, msg.altitude]


def _grab_attitude(timeout: float = 10.0) -> list[float]:
    from geometry_msgs.msg import QuaternionStamped
    import rospy

    msg = rospy.wait_for_message("/dji_sdk/attitude", QuaternionStamped, timeout=timeout)
    q = msg.quaternion
    return [q.w, q.x, q.y, q.z]


def _mask_from_polygon(polygon: list, h: int, w: int) -> np.ndarray:
    binary = np.zeros((h, w), dtype=np.uint8)
    pts = np.asarray(polygon, dtype=np.int32).reshape(-1, 1, 2)
    if len(pts) >= 3:
        cv2.fillPoly(binary, [pts], 1)
    return binary.astype(bool)


def _yaw_approach(drone_lat: float, drone_lon: float, target_lat: float, target_lon: float) -> float:
    """Spot yaw convention (same as entire_pipeline observation vectors)."""
    e, n, _ = pymap3d.geodetic2enu(target_lat, target_lon, 0.0, drone_lat, drone_lon, 0.0)
    return math.atan2(e, -n)


def _pick_person(instances: list[dict], min_confidence: float) -> dict | None:
    persons = [
        inst
        for inst in instances
        if inst.get("class_name", "").lower() in PERSON_LABELS
        and float(inst.get("confidence", 0.0)) >= min_confidence
    ]
    if not persons:
        return None
    return max(persons, key=lambda inst: float(inst["confidence"]))


def send_person_waypoint(
    person: dict,
    image_h: int,
    image_w: int,
    gps: list[float],
    attitude: list[float],
    spot_addr: tuple[str, int] = DEFAULT_SPOT_ADDR,
) -> bool:
    """Project person mask to GPS and send a single priority waypoint to Spot."""
    polygon = person.get("polygon")
    if not polygon:
        return False

    mask = _mask_from_polygon(polygon, image_h, image_w)
    lat, lon, _ = masks_to_gps([mask], np.zeros((image_h, image_w, 3), np.uint8), gps, attitude, labels=["person"])[0]
    if lat is None or lon is None:
        return False

    yaw = _yaw_approach(gps[0], gps[1], lat, lon)
    proto_sender([(lat, lon, yaw)], spot_addr=spot_addr)
    return True


class ActiveSensingNode:
    def __init__(
        self,
        masks_topic: str,
        spot_addr: tuple[str, int],
        cooldown_sec: float,
        min_confidence: float,
    ) -> None:
        import rospy
        from std_msgs.msg import String

        self._cooldown_sec = cooldown_sec
        self._min_confidence = min_confidence
        self._spot_addr = spot_addr
        self._last_sent = 0.0

        rospy.Subscriber(masks_topic, String, self._on_masks, queue_size=1)
        rospy.loginfo(
            "active_sensing: listening on %s (person labels=%s, cooldown=%.1fs, spot=%s:%d)",
            masks_topic,
            sorted(PERSON_LABELS),
            cooldown_sec,
            spot_addr[0],
            spot_addr[1],
        )

    def _on_masks(self, msg) -> None:
        import rospy

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            rospy.logwarn_throttle(30.0, "active_sensing: bad masks JSON: %s", exc)
            return

        person = _pick_person(data.get("instances", []), self._min_confidence)
        if person is None:
            return

        now = time.time()
        if now - self._last_sent < self._cooldown_sec:
            return

        try:
            gps = _grab_gps(timeout=2.0)
            attitude = _grab_attitude(timeout=2.0)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "active_sensing: DJI telemetry unavailable: %s", exc)
            return

        h = int(data.get("image_height", 0))
        w = int(data.get("image_width", 0))
        if h <= 0 or w <= 0:
            rospy.logwarn_throttle(30.0, "active_sensing: masks JSON missing image size")
            return

        if send_person_waypoint(person, h, w, gps, attitude, spot_addr=self._spot_addr):
            self._last_sent = now
            rospy.loginfo(
                "active_sensing: sent priority person waypoint to Spot (conf=%.2f, class=%s)",
                float(person["confidence"]),
                person.get("class_name"),
            )


def run_node(
    masks_topic: str = DEFAULT_MASKS_TOPIC,
    spot_host: str = DEFAULT_SPOT_ADDR[0],
    spot_port: int = DEFAULT_SPOT_ADDR[1],
    cooldown_sec: float = DEFAULT_COOLDOWN_SEC,
    min_confidence: float = DEFAULT_MIN_CONFIDENCE,
) -> None:
    import rospy

    rospy.init_node("active_sensing", anonymous=False)
    ActiveSensingNode(
        masks_topic=masks_topic,
        spot_addr=(spot_host, spot_port),
        cooldown_sec=cooldown_sec,
        min_confidence=min_confidence,
    )
    rospy.spin()


def main() -> None:
    parser = argparse.ArgumentParser(description="Send priority Spot waypoints on person detection")
    parser.add_argument("--masks-topic", default=DEFAULT_MASKS_TOPIC)
    parser.add_argument("--spot-host", default=DEFAULT_SPOT_ADDR[0])
    parser.add_argument("--spot-port", type=int, default=DEFAULT_SPOT_ADDR[1])
    parser.add_argument("--cooldown", type=float, default=DEFAULT_COOLDOWN_SEC,
                        help="Minimum seconds between Spot sends (default: %.1f)" % DEFAULT_COOLDOWN_SEC)
    parser.add_argument("--min-confidence", type=float, default=DEFAULT_MIN_CONFIDENCE)
    args = parser.parse_args()
    run_node(
        masks_topic=args.masks_topic,
        spot_host=args.spot_host,
        spot_port=args.spot_port,
        cooldown_sec=args.cooldown,
        min_confidence=args.min_confidence,
    )


if __name__ == "__main__":
    main()
