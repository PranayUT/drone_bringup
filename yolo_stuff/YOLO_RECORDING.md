# YOLO mask recording guide

Primary script: **`record_yolo_masks.py`**  
Location: `~/Documents/catkin_ws/src/drone_bringup/yolo_stuff/`

Use this for real flights and research. It runs YOLO on the CSI camera, shows a live preview, logs timing/FPS, and saves mask data to the **Extreme SSD** (not the Jetson internal storage).

---

## Before you start

1. **Mount the Extreme SSD** (same drive as mission logs):
   ```bash
   ls /media/drone/extreme1
   # or: ls /media/drone/extreme
   ```

2. **Weights** in `yolo_stuff/` (or pass full path):
   - `best-v1.engine` (TensorRT, fast — recommended)
   - `best-v1.pt` (PyTorch, slower)

3. **TensorRT on Jetson** — set once per terminal:
   ```bash
   export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
   ```

4. **Camera not in use** — for `--realworld` only (direct GStreamer). For **`--realworld-ros`**, start **`gscam`** first (`test_camera_record.sh` or mission camera bringup); YOLO subscribes to `/csi_cam_0/image_raw` and does **not** open CSI.

---

## How to record (real world, direct camera)

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/yolo_stuff
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
python3 record_yolo_masks.py --realworld --half
```

- Runs until you press **Ctrl+C** (or **q** in the preview window).
- Saves to external SSD by default.
- Saves mask files every **5th** inference (keeps disk/CPU load reasonable).
- Writes **`masks.json`** (human-readable) by default.

## How to record with ROS camera + rosbag (recommended with mission)

**Terminal 1** — ROS owns CSI (see `drone_stuff/CSI_CAMERA_RECORDING.md`):

```bash
SKIP_NVARGUS_RESTART=1 ./test_camera_record.sh 30
# OR: ./start_mission_synchronized_camera.sh ~/path/to/waypoints.txt
```

**Terminal 2** — same mask/timing files **plus** ROS topics for rosbag:

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/yolo_stuff
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
python3 record_yolo_masks.py --realworld-ros --half
```

ROS topics (JSON strings, same content as disk):

- `/yolo/detections_json`
- `/yolo/masks_json`
- `/yolo/timing_json`
- `/yolo/preview/image` (only with `--ros-publish-preview`)

### Useful options

| Flag | What it does |
|------|----------------|
| `--realworld` | Preset: direct CSI camera, preview, infer every 2, save every 5th |
| `--realworld-ros` | Preset: subscribe `/csi_cam_0/image_raw`, publish `/yolo/*`, same save cadence |
| `--source rostopic --ros-topic /csi_cam_0/image_raw` | Same as above, explicit |
| `--half` | FP16 on GPU (recommended on Jetson) |
| `--save-preview` | Also save `preview.jpg` (image with masks drawn) |
| `--save-every 1` | Save masks every inference (more data, heavier) |
| `--timing-only` | Only CSV/timing — no per-frame mask files |
| `--mask-format both` | Write both `masks.json` and `masks.npz` |
| `--out-dir /path/to/folder` | Force a specific output folder |

Example with preview images:

```bash
python3 record_yolo_masks.py --realworld --half --save-preview
```

---

## Where recordings are saved

**Default (preferred):**

```text
/media/drone/extreme1/yolo_records/2026-05-18_11-28-57/
```

If `extreme1` is missing, tries `/media/drone/extreme/yolo_records/`.  
If neither is mounted, falls back to:

```text
~/Documents/catkin_ws/src/drone_bringup/yolo_stuff/runs/mask_record_<timestamp>/
```

On start, the script prints either:

- `Saving to external SSD: /media/drone/extreme1/yolo_records/...`
- or a **WARNING** that it fell back to local storage.

### cd into a run

```bash
cd /media/drone/extreme1/yolo_records
ls -lt
cd 2026-05-18_11-28-57
```

Latest run in one step:

```bash
cd "$(ls -td /media/drone/extreme1/yolo_records/*/ 2>/dev/null | head -1)"
```

---

## Folder layout — what each file is

```text
2026-05-18_11-28-57/
├── run_manifest.json       # Config snapshot (model, camera, device, paths)
├── timing_per_frame.csv    # Every inference: latency ms, detection count
├── detections_summary.csv  # frame, num_detections (simple)
├── timing_summary.json     # End-of-run averages (mean FPS, p50, p95, …)
├── paper_table.md          # Table for papers (created when script exits cleanly)
└── frames/
    ├── frame_000001/
    ├── frame_000005/       # Every 5th inference with --realworld
    └── ...
```

### Inside each `frames/frame_XXXXXX/`

| File | Description |
|------|-------------|
| **`detections.json`** | Frame metadata: timing, number of objects, short per-object summary |
| **`masks.json`** | **Main mask file** — polygons, RLE, ellipse, Hu moments (open in any text editor) |
| **`masks.npz`** | NumPy archive (only if `--mask-format npz` or `both`) |
| **`preview.jpg`** | Annotated image (only with `--save-preview`) |

**Note:** `frame_000005` means the **5th YOLO inference**, not “camera frame 5”. With `--realworld`, folders are saved every 5 inferences: `000001`, `000005`, `000010`, …

### `detections_summary.csv` columns

```text
2789,0
 │    └── detections that frame (0 = nothing found)
 └─────── inference number
```

### `timing_per_frame.csv` columns

- `frame` — inference index  
- `inference_wall_ms` — total time (ms) → FPS ≈ 1000 / this  
- `mask_extract_ms` — time to build mask data  
- `num_detections` — object count  
- `ultra_preprocess_ms`, `ultra_inference_ms`, `ultra_postprocess_ms` — Ultralytics breakdown  

---

## How to check logging is working

```bash
# Newest run folder
ls -lt /media/drone/extreme1/yolo_records/ | head -5

# CSV growing?
tail -5 /media/drone/extreme1/yolo_records/*/timing_per_frame.csv

# Process running?
pgrep -af record_yolo_masks
```

---

## Reading `masks.json` (for mentor / paper)

Open in VS Code, browser, or terminal:

```bash
cat frames/frame_000500/masks.json
```

Example structure:

```json
{
  "image_height": 720,
  "image_width": 1280,
  "num_instances": 1,
  "instances": [
    {
      "class_name": "ball",
      "confidence": 0.87,
      "bbox_xyxy": [100, 200, 300, 400],
      "polygon": [[x, y], ...],
      "rle": { "size": [720, 1280], "counts": [...] },
      "ellipse": { "cx": ..., "cy": ..., "major_axis": ..., "minor_axis": ..., "angle_deg": ... },
      "hu_moments": [...]
    }
  ]
}
```

Python (no extra install beyond numpy if needed):

```python
import json
with open("frames/frame_000500/masks.json") as f:
    data = json.load(f)
print(data["num_instances"], data["instances"])
```

---

## Convert old `masks.npz` → `masks.json`

For runs recorded before JSON was default:

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/yolo_stuff
python3 npz_to_masks_json.py /media/drone/extreme1/yolo_records/2026-05-18_11-28-57
```

Creates `masks.json` next to each existing `masks.npz`.

---

## When `paper_table.md` appears

Written **when the script exits normally** (Ctrl+C in its terminal).  
If missing, use `timing_summary.json` instead — same statistics.

```bash
cat paper_table.md
cat timing_summary.json
```

---

## Requirements checklist

| Item | Command / value |
|------|------------------|
| Segmentation model | `--task segment` (default) |
| TensorRT weights | `--weights best-v1.engine` (default) |
| GPU | `--device 0` (default) |
| LD_PRELOAD | `export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1` |

---

## Related scripts

| Script | Use |
|--------|-----|
| `record_yolo_masks.py` | **Main** — record masks + timing |
| `npz_to_masks_json.py` | Convert old `.npz` to `.json` |
| `bench_best_v1.py` | Quick preview/FPS only, no saved masks |
| `build_engine.py` | One-time TensorRT export |

Quick commands also in **`cmds.txt`**.

---

## Troubleshooting

| Problem | What to try |
|---------|-------------|
| Saves to `runs/` not Extreme | Mount SSD: `ls /media/drone/extreme1` |
| All `,0` in detections CSV | Nothing detected — check preview, lighting, targets in frame, `--task segment` |
| No `paper_table.md` | Stop with Ctrl+C in the recorder terminal; read `timing_summary.json` |
| Camera fails to open | Another process using CSI; restart nvargus or stop `start_mission.sh` |
| `best-v1.engine` not found | Run from `yolo_stuff/` or pass full path to weights |
