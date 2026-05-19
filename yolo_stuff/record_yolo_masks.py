#!/usr/bin/env python3
"""
Record YOLO segmentation masks and benchmark metrics for research / papers.

Exports per-frame mask data in several numeric forms (NumPy-friendly) plus
timing statistics (latency, FPS, Ultralytics preprocess/inference/postprocess).

Mask representations (per detection)
------------------------------------
  binary          uint8 H×W full-resolution mask (rasterized from polygon)
  polygon         N×2 float32 vertex list (pixel coordinates)
  polygon_norm    N×2 float32 vertices normalized to [0, 1]
  rle             COCO-style run-length encoding {size, counts}
  ellipse         (cx, cy, major_axis, minor_axis, angle_deg) fitted to mask
  hu_moments      7 Hu invariant moments (shape descriptors)

Output layout (default on external SSD, not workspace)
-------------
  /media/drone/extreme1/yolo_records/<timestamp>/   (or extreme/, then local fallback)
  <out_dir>/
    run_manifest.json       experiment config + environment
    timing_summary.json     aggregate stats (paper-ready)
    timing_per_frame.csv    per-frame latencies
    detections_summary.csv  per-frame detection counts
    frames/frame_XXXXXX/
      detections.json       metadata (no large arrays)
      masks.json              human-readable masks (default; polygons, RLE, etc.)
      masks.npz               optional numpy archive (--mask-format npz or both)
      preview.jpg             optional overlay (--save-preview)
    paper_table.md            LaTeX-friendly markdown table snippet

Example (TensorRT seg model, static image, 50 runs):
  export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
  python3 record_yolo_masks.py --weights best-v1.engine --task segment \\
      --source image --image bus.jpg --iters 50 --out-dir runs/paper_run_01

Real-world (live CSI, run until Ctrl+C) — use this instead of bench_best_v1.py:
  export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
  python3 record_yolo_masks.py --realworld --weights best-v1.engine --half

  Same flight as ROS camera + rosbag (no CSI conflict — reads /csi_cam_0/image_raw):
  # Terminal 1: SKIP_NVARGUS_RESTART=1 ./test_camera_record.sh 30
  #   OR: ./start_mission_synchronized_camera.sh waypoints.txt
  # Terminal 2:
  python3 record_yolo_masks.py --realworld-ros --weights best-v1.engine --half

  Publishes (when --ros-publish, default for --source rostopic):
    /yolo/detections_json   std_msgs/String  (same as frames/.../detections.json)
    /yolo/masks_json        std_msgs/String  (same as frames/.../masks.json)
    /yolo/timing_json       std_msgs/String  (per-frame timing row)
    /yolo/preview/image     sensor_msgs/Image  (optional, --ros-publish-preview)
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, TextIO

import cv2
import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent

# External SSD paths (same drives as mission rosbag logs). Tried in order.
_EXTERNAL_YOLO_ROOTS = (
    Path("/media/drone/extreme1/yolo_records"),
    Path("/media/drone/extreme/yolo_records"),
)


def resolve_output_dir(explicit: str) -> tuple[Path, str]:
    """
    Prefer external Extreme SSD so internal eMMC/RAM pressure stays low.
    Override with --out-dir or env YOLO_RECORD_BASE=/media/drone/extreme1/yolo_records
    """
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if explicit:
        out = Path(explicit).expanduser()
        out.mkdir(parents=True, exist_ok=True)
        return out, "explicit"

    env_base = os.environ.get("YOLO_RECORD_BASE", "").strip()
    if env_base:
        out = Path(env_base).expanduser() / stamp
        out.mkdir(parents=True, exist_ok=True)
        return out, "env"

    for root in _EXTERNAL_YOLO_ROOTS:
        try:
            root.mkdir(parents=True, exist_ok=True)
            out = root / stamp
            out.mkdir(parents=True, exist_ok=True)
            return out, "external"
        except OSError:
            continue

    out = _SCRIPT_DIR / "runs" / f"mask_record_{stamp}"
    out.mkdir(parents=True, exist_ok=True)
    return out, "local_fallback"


def _read_csv_column(path: Path, col: int) -> list[float]:
    vals: list[float] = []
    if not path.exists():
        return vals
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) > col and row[col]:
                vals.append(float(row[col]))
    return vals


class TimingStream:
    """Append timing rows to disk immediately (low RAM on long runs)."""

    def __init__(self, out_dir: Path) -> None:
        self.out_dir = out_dir
        self.timing_path = out_dir / "timing_per_frame.csv"
        self.det_path = out_dir / "detections_summary.csv"
        self._timing_f: TextIO = self.timing_path.open("w", newline="", encoding="utf-8")
        self._det_f: TextIO = self.det_path.open("w", newline="", encoding="utf-8")
        self._timing_w = csv.writer(self._timing_f)
        self._det_w = csv.writer(self._det_f)
        self._timing_w.writerow(
            [
                "frame",
                "inference_wall_ms",
                "mask_extract_ms",
                "num_detections",
                "ultra_preprocess_ms",
                "ultra_inference_ms",
                "ultra_postprocess_ms",
            ]
        )
        self._det_w.writerow(["frame", "num_detections"])
        self._timing_f.flush()
        self._det_f.flush()
        self.recent_wall_ms: deque[float] = deque(maxlen=200)
        self.frame_count = 0
        self.det_total = 0
        self.det_min: int | None = None
        self.det_max: int | None = None

    def append(
        self,
        wall_ms: float,
        extract_ms: float,
        num_det: int,
        pre_ms: float,
        infer_ms: float,
        post_ms: float,
    ) -> None:
        self.frame_count += 1
        self._timing_w.writerow(
            [
                self.frame_count,
                f"{wall_ms:.4f}",
                f"{extract_ms:.4f}",
                num_det,
                f"{pre_ms:.4f}",
                f"{infer_ms:.4f}",
                f"{post_ms:.4f}",
            ]
        )
        self._det_w.writerow([self.frame_count, num_det])
        self._timing_f.flush()
        self._det_f.flush()
        self.recent_wall_ms.append(wall_ms)
        self.det_total += num_det
        self.det_min = num_det if self.det_min is None else min(self.det_min, num_det)
        self.det_max = num_det if self.det_max is None else max(self.det_max, num_det)

    def close(self) -> None:
        self._timing_f.flush()
        self._det_f.flush()
        self._timing_f.close()
        self._det_f.close()
        try:
            os.sync()
        except OSError:
            pass

    def detection_stats(self) -> dict[str, float | int | None]:
        n = self.frame_count
        if n == 0:
            return {"mean": None, "min": None, "max": None}
        dets = _read_csv_column(self.det_path, 1)
        arr = np.asarray(dets, dtype=np.float64) if dets else np.array([])
        return {
            "mean": float(arr.mean()) if len(arr) else None,
            "std": float(arr.std(ddof=1)) if len(arr) > 1 else 0.0,
            "min": self.det_min,
            "max": self.det_max,
        }


def _percentiles(values: list[float], ps: list[float]) -> dict[float, float | None]:
    if not values:
        return {p: None for p in ps}
    xs = sorted(values)
    n = len(xs)
    out: dict[float, float | None] = {}
    for p in ps:
        if n == 1:
            out[p] = xs[0]
            continue
        k = (p / 100.0) * (n - 1)
        f = int(k)
        c = min(f + 1, n - 1)
        out[p] = xs[f] if c == f else xs[f] + (xs[c] - xs[f]) * (k - f)
    return out


def _stats_ms(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {
            "n": 0,
            "mean": None,
            "std": None,
            "min": None,
            "max": None,
            "p50": None,
            "p90": None,
            "p95": None,
            "p99": None,
            "fps_mean": None,
        }
    arr = np.asarray(values, dtype=np.float64)
    p = _percentiles(values, [50, 90, 95, 99])
    mean = float(arr.mean())
    return {
        "n": len(values),
        "mean": mean,
        "std": float(arr.std(ddof=1)) if len(values) > 1 else 0.0,
        "min": float(arr.min()),
        "max": float(arr.max()),
        "p50": p[50],
        "p90": p[90],
        "p95": p[95],
        "p99": p[99],
        "fps_mean": 1000.0 / mean if mean > 0 else None,
    }


def _patch_torchvision_nms_for_jetson() -> None:
    try:
        import torch
        from torchvision.ops import nms

        boxes = torch.rand(2, 4, device="cuda")
        scores = torch.rand(2, device="cuda")
        nms(boxes, scores, 0.5)
    except Exception:
        import types

        from ultralytics.utils.nms import TorchNMS

        ops = types.SimpleNamespace(
            nms=lambda boxes, scores, iou_threshold: TorchNMS.nms(
                boxes, scores, iou_threshold
            )
        )
        tv = types.ModuleType("torchvision")
        tv.ops = ops
        sys.modules["torchvision"] = tv
        sys.modules["torchvision.ops"] = ops


class RosImageSource:
    """Subscribe to sensor_msgs/Image (e.g. gscam /csi_cam_0/image_raw)."""

    def __init__(self, topic: str, queue_size: int = 2) -> None:
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        self.topic = topic
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._bridge = CvBridge()
        self._received = 0

        def _cb(msg: Image) -> None:
            try:
                enc = (msg.encoding or "").lower()
                if enc == "rgb8":
                    bgr = cv2.cvtColor(
                        self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"),
                        cv2.COLOR_RGB2BGR,
                    )
                else:
                    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                return
            with self._lock:
                self._frame = bgr
                self._received += 1

        self._sub = rospy.Subscriber(topic, Image, _cb, queue_size=queue_size)

    def wait_for_frame(self, timeout_sec: float = 60.0) -> bool:
        import rospy

        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                if self._frame is not None:
                    return True
            rospy.sleep(0.05)
        return False

    def read(self) -> np.ndarray | None:
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()


class RosBagPublishers:
    """Mirror disk JSON to ROS topics so rosbag record -a can capture YOLO output."""

    def __init__(self, publish_preview: bool = False) -> None:
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

        self._String = String
        self._pub_det = rospy.Publisher("/yolo/detections_json", String, queue_size=10)
        self._pub_masks = rospy.Publisher("/yolo/masks_json", String, queue_size=10)
        self._pub_timing = rospy.Publisher("/yolo/timing_json", String, queue_size=10)
        self._pub_preview = None
        self._bridge = None
        if publish_preview:
            self._bridge = CvBridge()
            self._pub_preview = rospy.Publisher("/yolo/preview/image", Image, queue_size=2)

    def publish(
        self,
        frame_meta: dict[str, Any],
        masks_json: dict[str, Any],
        timing_row: dict[str, Any],
        preview_bgr: np.ndarray | None = None,
    ) -> None:
        import rospy

        det = self._String()
        det.data = json.dumps(frame_meta)
        self._pub_det.publish(det)

        masks = self._String()
        masks.data = json.dumps(masks_json)
        self._pub_masks.publish(masks)

        timing = self._String()
        timing.data = json.dumps(timing_row)
        self._pub_timing.publish(timing)

        if self._pub_preview is not None and preview_bgr is not None and self._bridge is not None:
            msg = self._bridge.cv2_to_imgmsg(preview_bgr, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            self._pub_preview.publish(msg)


def _build_gst_pipeline(sensor_id: int, width: int, height: int, fps: int) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"! video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12 "
        f"! nvvidconv flip-method=0 "
        f"! video/x-raw,width={width},height={height},format=BGRx "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink drop=1"
    )


def mask_to_rle(binary: np.ndarray) -> dict[str, Any]:
    """COCO-style RLE (column-major flatten)."""
    h, w = binary.shape
    flat = binary.astype(np.uint8).flatten(order="F")
    flat = np.concatenate([[0], flat, [0]])
    runs = np.where(flat[1:] != flat[:-1])[0] + 1
    runs[1::2] -= runs[::2]
    return {"size": [int(h), int(w)], "counts": runs.astype(int).tolist()}


def polygon_to_binary(polygon_xy: np.ndarray, height: int, width: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    if polygon_xy is None or len(polygon_xy) < 3:
        return mask
    pts = np.asarray(polygon_xy, dtype=np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(mask, [pts], 1)
    return mask


def fit_ellipse(binary: np.ndarray) -> tuple[float, float, float, float, float] | None:
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if len(cnt) < 5:
        return None
    (cx, cy), (ma, mi), angle = cv2.fitEllipse(cnt)
    return float(cx), float(cy), float(ma), float(mi), float(angle)


def hu_moments(binary: np.ndarray) -> np.ndarray:
    m = cv2.moments(binary.astype(np.uint8))
    hu = cv2.HuMoments(m).flatten()
    # log-scale for numerical stability in papers / storage
    return -np.sign(hu) * np.log10(np.abs(hu) + 1e-12)


@dataclass
class DetectionRecord:
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: list[float]
    mask_area_px: int
    mask_area_fraction: float
    polygon_n_vertices: int
    ellipse: list[float] | None
    hu_moments: list[float]
    rle_counts_len: int


def _json_instance(
    cls_id: int,
    class_name: str,
    conf: float,
    xyxy: np.ndarray,
    poly: np.ndarray,
    poly_n: np.ndarray,
    rle: dict[str, Any],
    ellipse: tuple[float, float, float, float, float] | None,
    hu: np.ndarray,
    area_px: int,
    h: int,
    w: int,
    include_binary: bool,
    binary: np.ndarray,
) -> dict[str, Any]:
    inst: dict[str, Any] = {
        "class_id": cls_id,
        "class_name": class_name,
        "confidence": round(conf, 6),
        "bbox_xyxy": [round(float(x), 2) for x in xyxy.tolist()],
        "polygon": [[round(float(x), 2), round(float(y), 2)] for x, y in poly.tolist()],
        "polygon_normalized": [
            [round(float(x), 6), round(float(y), 6)] for x, y in poly_n.tolist()
        ],
        "rle": rle,
        "ellipse": {
            "cx": ellipse[0],
            "cy": ellipse[1],
            "major_axis": ellipse[2],
            "minor_axis": ellipse[3],
            "angle_deg": ellipse[4],
        }
        if ellipse
        else None,
        "hu_moments": [round(float(v), 6) for v in hu.tolist()],
        "mask_area_px": area_px,
        "mask_area_fraction": round(area_px / float(h * w), 8),
    }
    if include_binary:
        inst["binary_mask"] = binary.astype(int).tolist()
    return inst


def extract_frame_detections(
    result,
    frame_shape: tuple[int, int, int],
    include_binary_in_json: bool = False,
) -> tuple[list[dict], dict[str, Any], dict[str, Any]]:
    """Build summary rows, masks.json payload, and NPZ arrays from one Ultralytics result."""
    h, w = frame_shape[:2]
    names = result.names if hasattr(result, "names") else {}
    records: list[DetectionRecord] = []
    instances: list[dict[str, Any]] = []

    binaries: list[np.ndarray] = []
    polygons: list[np.ndarray] = []
    polygons_norm: list[np.ndarray] = []
    rle_list: list[dict] = []
    ellipses: list[np.ndarray] = []
    hu_list: list[np.ndarray] = []
    class_ids: list[int] = []
    confidences: list[float] = []
    bboxes: list[np.ndarray] = []

    empty_masks = {
        "image_height": h,
        "image_width": w,
        "num_instances": 0,
        "instances": [],
    }
    empty_npz = {
        "image_height": np.int32(h),
        "image_width": np.int32(w),
        "binary_masks": np.zeros((0, h, w), dtype=np.uint8),
        "polygons": np.array([], dtype=object),
        "polygons_norm": np.array([], dtype=object),
        "rle": np.array([], dtype=object),
        "ellipse_params": np.zeros((0, 5), dtype=np.float32),
        "hu_moments": np.zeros((0, 7), dtype=np.float32),
        "class_ids": np.zeros((0,), dtype=np.int32),
        "confidences": np.zeros((0,), dtype=np.float32),
        "bboxes_xyxy": np.zeros((0, 4), dtype=np.float32),
    }

    if result.masks is None or result.boxes is None:
        return [], empty_masks, empty_npz

    boxes = result.boxes
    masks_xy = result.masks.xy
    masks_xyn = result.masks.xyn

    for i in range(len(boxes)):
        cls_id = int(boxes.cls[i].item())
        conf = float(boxes.conf[i].item())
        class_name = str(names.get(cls_id, str(cls_id)))
        xyxy = boxes.xyxy[i].cpu().numpy().astype(np.float32)
        poly = np.asarray(masks_xy[i], dtype=np.float32) if i < len(masks_xy) else np.zeros((0, 2))
        poly_n = np.asarray(masks_xyn[i], dtype=np.float32) if i < len(masks_xyn) else np.zeros((0, 2))

        binary = polygon_to_binary(poly, h, w)
        area_px = int(binary.sum())
        ellipse = fit_ellipse(binary)
        hu = hu_moments(binary)
        rle = mask_to_rle(binary)

        binaries.append(binary)
        polygons.append(poly)
        polygons_norm.append(poly_n)
        rle_list.append(rle)
        ellipses.append(
            np.asarray(ellipse if ellipse else [np.nan] * 5, dtype=np.float32)
        )
        hu_list.append(hu.astype(np.float32))
        class_ids.append(cls_id)
        confidences.append(conf)
        bboxes.append(xyxy)

        instances.append(
            _json_instance(
                cls_id,
                class_name,
                conf,
                xyxy,
                poly,
                poly_n,
                rle,
                ellipse,
                hu,
                area_px,
                h,
                w,
                include_binary_in_json,
                binary,
            )
        )
        records.append(
            DetectionRecord(
                class_id=cls_id,
                class_name=class_name,
                confidence=conf,
                bbox_xyxy=xyxy.tolist(),
                mask_area_px=area_px,
                mask_area_fraction=area_px / float(h * w),
                polygon_n_vertices=int(len(poly)),
                ellipse=list(ellipse) if ellipse else None,
                hu_moments=hu.tolist(),
                rle_counts_len=len(rle["counts"]),
            )
        )

    masks_json = {
        "image_height": h,
        "image_width": w,
        "num_instances": len(instances),
        "instances": instances,
    }
    npz = {
        "image_height": np.int32(h),
        "image_width": np.int32(w),
        "binary_masks": np.stack(binaries, axis=0) if binaries else np.zeros((0, h, w), dtype=np.uint8),
        "polygons": np.array(polygons, dtype=object),
        "polygons_norm": np.array(polygons_norm, dtype=object),
        "rle": np.array(rle_list, dtype=object),
        "ellipse_params": np.stack(ellipses, axis=0) if ellipses else np.zeros((0, 5), dtype=np.float32),
        "hu_moments": np.stack(hu_list, axis=0) if hu_list else np.zeros((0, 7), dtype=np.float32),
        "class_ids": np.asarray(class_ids, dtype=np.int32),
        "confidences": np.asarray(confidences, dtype=np.float32),
        "bboxes_xyxy": np.stack(bboxes, axis=0) if bboxes else np.zeros((0, 4), dtype=np.float32),
    }
    return [asdict(r) for r in records], masks_json, npz


def collect_environment() -> dict[str, Any]:
    env: dict[str, Any] = {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "processor": platform.processor(),
        "hostname": platform.node(),
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    }
    dt_model = Path("/proc/device-tree/model")
    if dt_model.exists():
        try:
            env["device_tree_model"] = dt_model.read_bytes().decode("utf-8").strip("\x00")
        except OSError:
            pass
    try:
        import torch

        env["torch_version"] = torch.__version__
        env["cuda_available"] = torch.cuda.is_available()
        if torch.cuda.is_available():
            env["cuda_device_name"] = torch.cuda.get_device_name(0)
    except ImportError:
        env["torch_version"] = None
    try:
        from ultralytics import __version__ as ultralytics_version

        env["ultralytics_version"] = ultralytics_version
    except ImportError:
        env["ultralytics_version"] = None
    return env


def write_paper_table(path: Path, summary: dict[str, Any], manifest: dict[str, Any]) -> None:
    inf = summary.get("inference_wall_ms", {})
    ul = summary.get("ultralytics_speed_ms", {})
    lines = [
        "# YOLO segmentation benchmark (auto-generated)",
        "",
        "| Metric | Value |",
        "|--------|------:|",
        f"| Frames | {summary.get('n_frames', 0)} |",
        f"| Model | `{manifest.get('weights_basename', '')}` |",
        f"| Task | `{manifest.get('task', '')}` |",
        f"| Input size | {manifest.get('imgsz', '')} |",
        f"| Device | `{manifest.get('device', '')}` |",
        f"| FP16 | {manifest.get('half', False)} |",
        f"| Mean latency (ms) | {inf.get('mean', '—'):.2f} |" if inf.get("mean") else "| Mean latency (ms) | — |",
        f"| Std (ms) | {inf.get('std', '—'):.2f} |" if inf.get("std") is not None else "| Std (ms) | — |",
        f"| p50 (ms) | {inf.get('p50', '—'):.2f} |" if inf.get("p50") else "| p50 (ms) | — |",
        f"| p95 (ms) | {inf.get('p95', '—'):.2f} |" if inf.get("p95") else "| p95 (ms) | — |",
        f"| Mean FPS | {inf.get('fps_mean', '—'):.2f} |" if inf.get("fps_mean") else "| Mean FPS | — |",
        f"| Detections / frame (mean) | {summary.get('detections_per_frame', {}).get('mean', '—')} |",
        "",
        "## Ultralytics internal timing (ms)",
        "",
        "| Stage | Mean |",
        "|-------|-----:|",
    ]
    for stage in ("preprocess", "inference", "postprocess"):
        val = ul.get(stage, {}).get("mean")
        lines.append(f"| {stage} | {val:.3f} |" if val is not None else f"| {stage} | — |")
    lines.extend(
        [
            "",
            "## Loading masks in Python",
            "",
            "```python",
            "import numpy as np",
            "import json",
            "with open('frames/frame_000001/masks.json') as f:",
            "    data = json.load(f)",
            "for inst in data['instances']:",
            "    print(inst['class_name'], inst['confidence'], inst['polygon'])",
            "```",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Primary YOLO tool: live inference + mask recording + timing stats. "
            "Use --realworld for CSI camera until Ctrl+C."
        )
    )
    ap.add_argument(
        "--realworld",
        action="store_true",
        help="Preset: camera, preview, infer-every 2, iters 0, save-every 5, 1280x720@30",
    )
    ap.add_argument(
        "--realworld-ros",
        action="store_true",
        help=(
            "Preset: --source rostopic on /csi_cam_0/image_raw, same infer/save as --realworld, "
            "publish YOLO JSON topics for rosbag (use with gscam already running)."
        ),
    )
    ap.add_argument("--weights", default="best-v1.engine")
    ap.add_argument("--task", default="segment", help="Use 'segment' for best-v1 seg weights")
    ap.add_argument(
        "--source",
        choices=["image", "camera", "video", "rostopic"],
        default="camera",
    )
    ap.add_argument(
        "--ros-topic",
        default="/csi_cam_0/image_raw",
        help="Image topic when --source rostopic (default: gscam output)",
    )
    ap.add_argument(
        "--ros-publish",
        action="store_true",
        help="Publish /yolo/* topics for rosbag (default: on for --source rostopic)",
    )
    ap.add_argument(
        "--no-ros-publish",
        action="store_true",
        help="Disable /yolo/* publishers even for --source rostopic",
    )
    ap.add_argument(
        "--ros-publish-preview",
        action="store_true",
        help="Also publish /yolo/preview/image (large; omit for smaller bags)",
    )
    ap.add_argument("--image", default="bus.jpg")
    ap.add_argument("--video", default="", help="Video file when --source video")
    ap.add_argument("--sensor-id", type=int, default=0)
    ap.add_argument("--cam-width", type=int, default=1280)
    ap.add_argument("--cam-height", type=int, default=720)
    ap.add_argument("--cam-fps", type=int, default=30)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    ap.add_argument("--half", action="store_true", help="FP16 on GPU (recommended on Jetson)")
    ap.add_argument("--no-half", action="store_true", help="Disable FP16")
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument(
        "--iters",
        type=int,
        default=None,
        help="Inference frames to process (default: 0=camera/video until Ctrl+C, 30=image)",
    )
    ap.add_argument(
        "--out-dir",
        default="",
        help=(
            "Output directory (default: /media/drone/extreme1|extreme/yolo_records/<stamp>, "
            "else workspace runs/). Override base with env YOLO_RECORD_BASE."
        ),
    )
    ap.add_argument(
        "--save-every",
        type=int,
        default=1,
        help="Write masks.npz / detections.json every N inference frames (timing always logged)",
    )
    ap.add_argument(
        "--timing-only",
        action="store_true",
        help="Do not write per-frame masks to disk; only CSV + summary (lighter for long runs)",
    )
    ap.add_argument(
        "--mask-format",
        choices=["json", "npz", "both"],
        default="json",
        help="Save masks as masks.json (readable), masks.npz, or both (default: json)",
    )
    ap.add_argument(
        "--include-binary-in-json",
        action="store_true",
        help="Include full HxW binary mask arrays in masks.json (very large files)",
    )
    ap.add_argument("--save-preview", action="store_true", help="Save annotated preview when saving masks")
    ap.add_argument("--preview", action="store_true", help="Show live preview window")
    ap.add_argument("--infer-every", type=int, default=1, help="Camera/video: run YOLO every N captured frames")
    args = ap.parse_args()

    if args.realworld:
        args.source = "camera"
        args.preview = True
        if args.iters is None:
            args.iters = 0
        if args.infer_every == 1:
            args.infer_every = 2
        if args.save_every == 1:
            args.save_every = 5
        if args.cam_width == 1280 and args.cam_height == 720:
            pass  # already realworld-friendly defaults

    if args.realworld_ros:
        args.source = "rostopic"
        args.preview = True
        if args.iters is None:
            args.iters = 0
        if args.infer_every == 1:
            args.infer_every = 2
        if args.save_every == 1:
            args.save_every = 5

    if args.iters is None:
        args.iters = 0 if args.source in ("camera", "video", "rostopic") else 30

    ros_publish = args.source == "rostopic" and not args.no_ros_publish
    if args.ros_publish:
        ros_publish = True
    if args.save_every < 1:
        print("ERROR: --save-every must be >= 1", file=sys.stderr)
        return 2

    weights = Path(args.weights).expanduser()
    if not weights.is_absolute():
        candidate = _SCRIPT_DIR / weights
        if candidate.exists():
            weights = candidate
    if not weights.exists():
        print(f"ERROR: weights not found: {weights}", file=sys.stderr)
        return 2

    out_dir, storage_kind = resolve_output_dir(args.out_dir)
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(exist_ok=True)
    if storage_kind == "external":
        print(f"Saving to external SSD: {out_dir}")
    elif storage_kind == "local_fallback":
        print(
            f"WARNING: Extreme SSD not mounted — saving under workspace (more eMMC load): {out_dir}",
            file=sys.stderr,
        )
    else:
        print(f"Saving to: {out_dir}")

    cap = None
    static_img = None
    ros_source: RosImageSource | None = None
    ros_pubs: RosBagPublishers | None = None
    video_path = Path(args.video).expanduser() if args.video else None

    if args.source == "rostopic":
        import rospy

        rospy.init_node("yolo_mask_recorder", anonymous=True)
        ros_source = RosImageSource(args.ros_topic)
        print(f"Waiting for ROS images on {args.ros_topic}...")
        if not ros_source.wait_for_frame(60.0):
            print(f"ERROR: no messages on {args.ros_topic} within 60s", file=sys.stderr)
            print("Start gscam first (e.g. test_camera_record.sh or csi_camera_only.launch).", file=sys.stderr)
            return 2
        print(f"  receiving from {args.ros_topic}")
        if ros_publish:
            ros_pubs = RosBagPublishers(publish_preview=args.ros_publish_preview)
            print(
                "ROS publish: /yolo/detections_json /yolo/masks_json /yolo/timing_json"
                + (" /yolo/preview/image" if args.ros_publish_preview else "")
            )
    elif args.source == "camera":
        pipeline = _build_gst_pipeline(args.sensor_id, args.cam_width, args.cam_height, args.cam_fps)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("ERROR: failed to open CSI camera", file=sys.stderr)
            return 2
    elif args.source == "video":
        if not video_path or not video_path.exists():
            print("ERROR: --video required and must exist for --source video", file=sys.stderr)
            return 2
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            print(f"ERROR: failed to open video: {video_path}", file=sys.stderr)
            return 2
    else:
        image_path = Path(args.image)
        if not image_path.is_absolute():
            image_path = _SCRIPT_DIR / image_path
        if not image_path.exists():
            print(f"ERROR: image not found: {image_path}", file=sys.stderr)
            return 2
        static_img = cv2.imread(str(image_path))
        if static_img is None:
            print(f"ERROR: failed to read image: {image_path}", file=sys.stderr)
            return 2

    _patch_torchvision_nms_for_jetson()
    from ultralytics import YOLO

    model = YOLO(str(weights), task=args.task) if args.task else YOLO(str(weights))
    is_cpu = str(args.device).lower() == "cpu"
    use_half = not args.no_half and not is_cpu

    manifest = {
        "weights": str(weights.resolve()),
        "weights_basename": weights.name,
        "task": args.task,
        "source": args.source,
        "imgsz": args.imgsz,
        "device": args.device,
        "half": use_half,
        "warmup_iters": args.warmup,
        "record_iters": args.iters,
        "infer_every": args.infer_every,
        "save_every": args.save_every,
        "timing_only": args.timing_only,
        "mask_format": args.mask_format,
        "include_binary_in_json": args.include_binary_in_json,
        "realworld_preset": args.realworld,
        "realworld_ros_preset": args.realworld_ros,
        "ros_topic": args.ros_topic if args.source == "rostopic" else None,
        "ros_publish": ros_publish,
        "ros_publish_preview": args.ros_publish_preview,
        "storage_kind": storage_kind,
        "output_dir": str(out_dir),
        "environment": collect_environment(),
    }
    (out_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    timing_log = TimingStream(out_dir)

    def get_frame() -> np.ndarray | None:
        if ros_source is not None:
            return ros_source.read()
        if cap is not None:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("Frame read failed")
            return frame
        return static_img.copy()

    def predict(img):
        return model.predict(
            source=img,
            imgsz=args.imgsz,
            device=args.device,
            half=use_half,
            verbose=False,
        )[0]

    for _ in range(max(args.warmup, 0)):
        warm = get_frame()
        while warm is None and args.source == "rostopic":
            import rospy

            rospy.sleep(0.01)
            warm = get_frame()
        if warm is None:
            break
        predict(warm)

    infer_every = max(1, args.infer_every)
    frame_idx = 0
    recorded = 0
    last_result = None
    last_report = time.perf_counter()

    print(f"Recording to {out_dir}")
    print(
        f"model={weights.name} task={args.task} source={args.source} "
        f"device={args.device} half={use_half} infer_every={infer_every} "
        f"save_every={args.save_every} timing_only={args.timing_only}"
    )
    if args.iters == 0 and args.source in ("camera", "video", "rostopic"):
        print("Running until Ctrl+C (or q in preview window).")

    try:
        while True:
            if args.source == "rostopic":
                import rospy

                if rospy.is_shutdown():
                    break

            frame_idx += 1
            if args.iters > 0 and recorded >= args.iters:
                break

            img = get_frame()
            if img is None:
                if args.source == "rostopic":
                    import rospy

                    rospy.sleep(0.01)
                    continue
                raise RuntimeError("Frame read failed")

            do_infer = (
                args.source not in ("camera", "video", "rostopic")
                or frame_idx % infer_every == 0
            )

            if do_infer:
                t0 = time.perf_counter()
                last_result = predict(img)
                t1 = time.perf_counter()

                t2 = time.perf_counter()
                det_json, masks_json, npz_payload = extract_frame_detections(
                    last_result,
                    img.shape,
                    include_binary_in_json=args.include_binary_in_json,
                )
                t3 = time.perf_counter()

                wall = (t1 - t0) * 1000.0
                extract = (t3 - t2) * 1000.0
                speed = getattr(last_result, "speed", None) or {}
                pre = float(speed.get("preprocess", 0.0))
                infer = float(speed.get("inference", 0.0))
                post = float(speed.get("postprocess", 0.0))
                timing_log.append(wall, extract, len(det_json), pre, infer, post)

                recorded += 1
                save_frame = not args.timing_only and (
                    recorded == 1 or recorded % args.save_every == 0
                )
                if save_frame:
                    frame_dir = frames_dir / f"frame_{recorded:06d}"
                    frame_dir.mkdir(exist_ok=True)
                    frame_meta = {
                        "frame_index": recorded,
                        "source_frame_index": frame_idx,
                        "image_shape": list(img.shape),
                        "num_detections": len(det_json),
                        "inference_wall_ms": wall,
                        "mask_extract_ms": extract,
                        "ultralytics_speed_ms": speed,
                        "detections": det_json,
                    }
                    (frame_dir / "detections.json").write_text(
                        json.dumps(frame_meta, indent=2), encoding="utf-8"
                    )
                    if args.mask_format in ("json", "both"):
                        (frame_dir / "masks.json").write_text(
                            json.dumps(masks_json, indent=2), encoding="utf-8"
                        )
                    if args.mask_format in ("npz", "both"):
                        np.savez_compressed(frame_dir / "masks.npz", **npz_payload)
                    if args.save_preview:
                        cv2.imwrite(str(frame_dir / "preview.jpg"), last_result.plot(img=img))

                if ros_pubs is not None:
                    timing_row = {
                        "frame_index": recorded,
                        "source_frame_index": frame_idx,
                        "inference_wall_ms": wall,
                        "mask_extract_ms": extract,
                        "num_detections": len(det_json),
                        "ultralytics_speed_ms": speed,
                    }
                    frame_meta = {
                        "frame_index": recorded,
                        "source_frame_index": frame_idx,
                        "image_shape": list(img.shape),
                        "num_detections": len(det_json),
                        "inference_wall_ms": wall,
                        "mask_extract_ms": extract,
                        "ultralytics_speed_ms": speed,
                        "detections": det_json,
                    }
                    preview_bgr = None
                    if args.ros_publish_preview:
                        preview_bgr = last_result.plot(img=img)
                    ros_pubs.publish(frame_meta, masks_json, timing_row, preview_bgr)

            if args.preview:
                vis = (
                    last_result.plot(img=img)
                    if last_result is not None
                    else img
                )
                cv2.imshow("YOLO mask recorder", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            now = time.perf_counter()
            if do_infer and timing_log.recent_wall_ms and (
                now - last_report >= 2.0 or recorded == 1
            ):
                s = _stats_ms(list(timing_log.recent_wall_ms))
                print(
                    f"infer={recorded} capture={frame_idx} "
                    f"last={timing_log.recent_wall_ms[-1]:.2f}ms "
                    f"avg={s['mean']:.2f}ms fps={s['fps_mean']:.2f}"
                )
                last_report = now

            if args.source == "image":
                if args.iters <= 0:
                    break
                if recorded >= args.iters:
                    break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        timing_log.close()
        if cap is not None:
            cap.release()
        if args.preview:
            cv2.destroyAllWindows()

    wall_all = _read_csv_column(timing_log.timing_path, 1)
    extract_all = _read_csv_column(timing_log.timing_path, 2)
    pre_all = _read_csv_column(timing_log.timing_path, 4)
    infer_all = _read_csv_column(timing_log.timing_path, 5)
    post_all = _read_csv_column(timing_log.timing_path, 6)

    timing_summary = {
        "n_frames": recorded,
        "storage_kind": storage_kind,
        "output_dir": str(out_dir),
        "inference_wall_ms": _stats_ms(wall_all),
        "mask_extract_ms": _stats_ms(extract_all),
        "ultralytics_speed_ms": {
            "preprocess": _stats_ms(pre_all),
            "inference": _stats_ms(infer_all),
            "postprocess": _stats_ms(post_all),
        },
        "detections_per_frame": timing_log.detection_stats(),
        "total_recorded_detections": timing_log.det_total,
    }
    (out_dir / "timing_summary.json").write_text(
        json.dumps(timing_summary, indent=2), encoding="utf-8"
    )

    write_paper_table(out_dir / "paper_table.md", timing_summary, manifest)

    s = timing_summary["inference_wall_ms"]
    print("\n=== Recording complete ===")
    print(f"Output: {out_dir}")
    print(f"Frames: {recorded}")
    if s.get("mean") is not None:
        print(
            f"Latency: mean={s['mean']:.2f}ms std={s['std']:.2f}ms "
            f"p50={s['p50']:.2f} p95={s['p95']:.2f} fps={s['fps_mean']:.2f}"
        )
    print(f"See timing_summary.json and paper_table.md for paper-ready tables.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
