#!/usr/bin/env python3
"""
Convert existing masks.npz files to human-readable masks.json (for mentor / papers).

Usage:
  python3 npz_to_masks_json.py /media/drone/extreme1/yolo_records/2026-05-18_11-28-57
  python3 npz_to_masks_json.py /path/to/frames/frame_000005
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def npz_to_masks_json(npz_path: Path, out_path: Path | None = None) -> Path:
    out_path = out_path or npz_path.with_suffix(".json").with_name("masks.json")
    data = np.load(npz_path, allow_pickle=True)

    h = int(data["image_height"])
    w = int(data["image_width"])
    n = int(len(data["class_ids"])) if "class_ids" in data else 0

    instances = []
    for i in range(n):
        poly = np.asarray(data["polygons"][i], dtype=np.float32)
        poly_n = np.asarray(data["polygons_norm"][i], dtype=np.float32)
        rle = data["rle"][i].item() if hasattr(data["rle"][i], "item") else data["rle"][i]
        ellipse_row = np.asarray(data["ellipse_params"][i], dtype=np.float64)
        ellipse = None
        if not np.isnan(ellipse_row).any():
            ellipse = {
                "cx": float(ellipse_row[0]),
                "cy": float(ellipse_row[1]),
                "major_axis": float(ellipse_row[2]),
                "minor_axis": float(ellipse_row[3]),
                "angle_deg": float(ellipse_row[4]),
            }
        instances.append(
            {
                "class_id": int(data["class_ids"][i]),
                "confidence": round(float(data["confidences"][i]), 6),
                "bbox_xyxy": [round(float(x), 2) for x in data["bboxes_xyxy"][i].tolist()],
                "polygon": [[round(float(x), 2), round(float(y), 2)] for x, y in poly.tolist()],
                "polygon_normalized": [
                    [round(float(x), 6), round(float(y), 6)] for x, y in poly_n.tolist()
                ],
                "rle": rle,
                "ellipse": ellipse,
                "hu_moments": [round(float(v), 6) for v in data["hu_moments"][i].tolist()],
                "mask_area_px": int(data["binary_masks"][i].sum()),
                "mask_area_fraction": round(float(data["binary_masks"][i].sum()) / (h * w), 8),
            }
        )

    payload = {
        "image_height": h,
        "image_width": w,
        "num_instances": len(instances),
        "instances": instances,
        "converted_from": str(npz_path),
    }
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser(description="Convert masks.npz → masks.json")
    ap.add_argument("path", help="Run folder (with frames/) or single frame_* folder")
    ap.add_argument("--delete-npz", action="store_true", help="Remove .npz after successful conversion")
    args = ap.parse_args()

    root = Path(args.path).expanduser()
    if not root.exists():
        print(f"ERROR: not found: {root}", file=sys.stderr)
        return 2

    if root.is_dir() and (root / "frames").is_dir():
        npz_files = sorted((root / "frames").glob("frame_*/masks.npz"))
    elif root.name.startswith("frame_") and (root / "masks.npz").exists():
        npz_files = [root / "masks.npz"]
    elif list(root.glob("frame_*/masks.npz")):
        npz_files = sorted(root.glob("frame_*/masks.npz"))
    else:
        print(f"ERROR: no masks.npz under {root}", file=sys.stderr)
        return 2

    print(f"Converting {len(npz_files)} file(s)...")
    for npz in npz_files:
        out = npz_to_masks_json(npz)
        print(f"  {out}")
        if args.delete_npz:
            npz.unlink()

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
