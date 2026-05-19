#!/usr/bin/env python3
"""
Convert an image topic from a ROS1 bag to an MP4 file (H.264 via ffmpeg when available).

  source /opt/ros/noetic/setup.bash
  source ~/Documents/catkin_ws/devel/setup.bash
  python3 bag_to_video.py /path/to/camera_test.bag
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import rosbag
from cv_bridge import CvBridge


def msg_to_bgr(bridge: CvBridge, msg) -> "object":
    """Convert sensor_msgs Image/CompressedImage to BGR numpy array."""
    if hasattr(msg, "format"):  # CompressedImage
        return bridge.compressed_img_to_cv2(msg, desired_encoding="bgr8")
    if hasattr(msg, "encoding"):  # sensor_msgs/Image
        enc = (msg.encoding or "").lower()
        if enc == "rgb8":
            rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if enc in ("bgr8", "bgra8", "rgba8", "mono8"):
            return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        raw = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if len(raw.shape) == 3 and raw.shape[2] == 4:
            return cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
        if len(raw.shape) == 3 and raw.shape[2] == 3:
            return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        return raw
    raise TypeError(f"not an image message: {type(msg)}")


def list_image_topics(bag_path: str) -> dict[str, str]:
    with rosbag.Bag(bag_path, "r") as bag:
        return {
            t: m.msg_type
            for t, m in bag.get_type_and_topic_info()[1].items()
            if "Image" in m.msg_type
        }


def resolve_topic(bag_path: str, topic: str | None) -> str:
    topics = list_image_topics(bag_path)
    if topic:
        if topic not in topics:
            print(f"ERROR: topic not in bag: {topic}", file=sys.stderr)
            for t, ty in sorted(topics.items()):
                print(f"  {t}  ({ty})", file=sys.stderr)
            sys.exit(1)
        return topic

    for candidate in (
        "/csi_cam_0/image_raw",
        "/csi_cam_0/image_raw/compressed",
        "/cam0/image_raw",
        "/cam0/image_raw/compressed",
    ):
        if candidate in topics:
            return candidate

    if len(topics) == 1:
        return next(iter(topics))

    print("ERROR: specify -t. Image topics in bag:", file=sys.stderr)
    for t, ty in sorted(topics.items()):
        print(f"  {t}  ({ty})", file=sys.stderr)
    sys.exit(1)


def estimate_fps(timestamps: list[float], default: float) -> float:
    if len(timestamps) < 2:
        return default
    deltas = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
    deltas = [d for d in deltas if d > 1e-6]
    if not deltas:
        return default
    import statistics

    return max(1.0, min(120.0, 1.0 / statistics.median(deltas)))


class FfmpegWriter:
    def __init__(self, path: Path, w: int, h: int, fps: float) -> None:
        self._proc = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-f",
                "rawvideo",
                "-vcodec",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{w}x{h}",
                "-r",
                f"{fps:.3f}",
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "fast",
                "-crf",
                "23",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(path),
            ],
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.path = path

    def write(self, frame) -> None:
        assert self._proc.stdin is not None
        self._proc.stdin.write(frame.tobytes())

    def close(self) -> None:
        if self._proc.stdin:
            self._proc.stdin.close()
        err = self._proc.stderr.read().decode("utf-8", errors="replace") if self._proc.stderr else ""
        rc = self._proc.wait()
        if rc != 0:
            raise RuntimeError(f"ffmpeg failed ({rc}):\n{err[-2000:]}")


def main() -> None:
    ap = argparse.ArgumentParser(description="ROS bag image topic -> MP4")
    ap.add_argument("bag", help="Path to .bag file")
    ap.add_argument("-t", "--topic", default=None)
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument(
        "--opencv",
        action="store_true",
        help="Use OpenCV mp4v (often broken in players; not recommended)",
    )
    args = ap.parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_file():
        print(f"ERROR: bag not found: {bag_path}", file=sys.stderr)
        sys.exit(1)

    topic = resolve_topic(str(bag_path), args.topic)
    out_path = Path(args.output) if args.output else bag_path.with_suffix(".mp4")
    use_ffmpeg = not args.opencv and shutil.which("ffmpeg") is not None

    bridge = CvBridge()
    written = 0
    skipped = 0
    first_encoding: str | None = None

    print(f"Reading: {bag_path}")
    print(f"Topic:   {topic}")
    print(f"Output:  {out_path}")
    print(f"Encoder: {'ffmpeg libx264' if use_ffmpeg else 'OpenCV mp4v (try: sudo apt install ffmpeg)'}")

    ts_scan: list[float] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _topic, _msg, t in bag.read_messages(topics=[topic]):
            ts_scan.append(t.to_sec())
            if args.max_frames and len(ts_scan) >= args.max_frames:
                break
    if not ts_scan:
        print("ERROR: no messages on topic", file=sys.stderr)
        sys.exit(1)

    fps = args.fps if args.fps else estimate_fps(ts_scan, 30.0)
    print(f"FPS:     {fps:.2f}  ({len(ts_scan)} frames)")

    writer = None
    cv_writer: cv2.VideoWriter | None = None

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _topic, msg, t in bag.read_messages(topics=[topic]):
            if args.max_frames and written >= args.max_frames:
                break

            try:
                if first_encoding is None and hasattr(msg, "encoding"):
                    first_encoding = msg.encoding
                frame = msg_to_bgr(bridge, msg)
            except Exception as e:
                skipped += 1
                if skipped <= 5:
                    print(f"WARN: skip frame: {e}", file=sys.stderr)
                continue

            if written == 0:
                mean_px = float(frame.mean())
                print(f"Encoding: {first_encoding}  size: {frame.shape[1]}x{frame.shape[0]}  mean: {mean_px:.1f}")
                if mean_px < 5.0:
                    print("WARN: frames look almost black — check camera/lens, not just the encoder", file=sys.stderr)

            h, w = frame.shape[:2]

            if writer is None and cv_writer is None:
                if use_ffmpeg:
                    writer = FfmpegWriter(out_path, w, h, fps)
                else:
                    cv_writer = cv2.VideoWriter(
                        str(out_path),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        fps,
                        (w, h),
                    )
                    if not cv_writer.isOpened():
                        print("ERROR: VideoWriter failed. Install ffmpeg: sudo apt install ffmpeg", file=sys.stderr)
                        sys.exit(1)

            if writer is not None:
                writer.write(frame)
            else:
                assert cv_writer is not None
                cv_writer.write(frame)

            written += 1
            if written % 100 == 0:
                print(f"  {written} frames...")

    if written == 0:
        print("ERROR: no frames written", file=sys.stderr)
        sys.exit(1)

    if writer is not None:
        writer.close()
    if cv_writer is not None:
        cv_writer.release()
        print("NOTE: OpenCV mp4 may show 0:00/black in some players. Re-run without --opencv after: sudo apt install ffmpeg", file=sys.stderr)

    print(f"Done: {written} frames -> {out_path}")
    if skipped:
        print(f"Skipped {skipped} bad frames")


if __name__ == "__main__":
    main()
