# Mission + CSI camera + YOLO + rosbag — command pipeline

One flight: DJI waypoint mission, CSI video in rosbag, YOLO masks on disk (+ YOLO JSON in rosbag).

**Rule:** only **one** process owns the CSI camera (`gscam` in the mission script). YOLO uses `--realworld-ros` (subscribes to ROS), not `--realworld`.

---

## Before flight

```bash
# SSD mounted
ls /media/drone/extreme1

# ROS workspace
source /opt/ros/noetic/setup.bash
source ~/Documents/catkin_ws/devel/setup.bash

# Edit clear-anchor waypoints (FC clear only; not the flown mission)
nano ~/Documents/catkin_ws/src/drone_bringup/drone_stuff/Dynamic_Waypoint/clear_anchor_waypoints.txt

# Take off manually (runner requires IN_AIR)
```

---

## Terminal 1 — mission + camera + rosbag

Starts `gscam`, DJI SDK, waypoint runner, and `rosbag record -a` → **`mission.bag`**.

```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/catkin_ws/devel/setup.bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff

SKIP_NVARGUS_RESTART=1 ./start_mission_synchronized_camera.sh
```

Wait until logs show gates passed / camera publishing. Note the log folder printed at start, e.g.:

`/media/drone/extreme1/logs/2026-05-18_14-30-00/`

**Dynamic mission (default):** after FC clear, runner waits for the mission file. In **Terminal 3** (or before clear if you already have it):

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff/Dynamic_Waypoint
./create_dynamic_mission.sh ~/path/to/waypoints.txt
```

**Fixed mission (no wait):**

```bash
USE_DYNAMIC_MISSION=0 ./start_mission_synchronized_camera.sh ~/path/to/waypoints.txt 0.5 1.0
```

**Stop:** Ctrl+C in Terminal 1 (finalizes `mission.bag`).

---

## Terminal 2 — YOLO masks (+ rosbag topics)

Start **after** `/csi_cam_0/image_raw` is publishing (mission script up).

```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/catkin_ws/devel/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
cd ~/Documents/catkin_ws/src/drone_bringup/yolo_stuff

python3 record_yolo_masks.py --realworld-ros --weights best-v1.engine --half
```

**Stop:** Ctrl+C in Terminal 2 (writes `timing_summary.json`, `paper_table.md`).

Optional preview frames in bag (large):

```bash
python3 record_yolo_masks.py --realworld-ros --weights best-v1.engine --half --ros-publish-preview
```

---

## What you get

| Output | Location |
|--------|----------|
| Rosbag (all topics: camera, DJI, `/yolo/*`, …) | `/media/drone/extreme1/logs/<timestamp>/mission.bag` |
| Mission / bringup logs | same folder: `start_mission_camera.log`, `bringup.log`, `bag.log` |
| YOLO masks + timing CSV/JSON | `/media/drone/extreme1/yolo_records/<timestamp>/` |

Rosbag includes at least: `/csi_cam_0/image_raw`, `/dji_sdk/*`, `/yolo/detections_json`, `/yolo/masks_json`, `/yolo/timing_json`.

---

## After flight — camera MP4 from bag

```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/catkin_ws/devel/setup.bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff

python3 bag_to_video.py /media/drone/extreme1/logs/<timestamp>/mission.bag
# → mission.mp4 next to the bag
```

```bash
rosbag info /media/drone/extreme1/logs/<timestamp>/mission.bag
```

YOLO run folder:

```bash
ls -lt /media/drone/extreme1/yolo_records/ | head -3
cd "$(ls -td /media/drone/extreme1/yolo_records/*/ | head -1)"
```

---

## Quick checklist

1. SSD mounted  
2. Take off  
3. Terminal 1: `start_mission_synchronized_camera.sh`  
4. Terminal 2: `record_yolo_masks.py --realworld-ros ...`  
5. (If dynamic) create `Dynamic_Waypoint/dynamic_mission.txt`  
6. Land / mission ends  
7. Ctrl+C Terminal 2, then Terminal 1  
8. `bag_to_video.py` on `mission.bag`

---

## Do not

- Run `record_yolo_masks.py --realworld` while the mission script owns the camera  
- Start YOLO before `/csi_cam_0/image_raw` exists  
- Kill Terminal 1 with `kill -9` if you need a valid `mission.bag` (use Ctrl+C)

More detail: `drone_stuff/CSI_CAMERA_RECORDING.md`, `yolo_stuff/YOLO_RECORDING.md`, `drone_stuff/Dynamic_Waypoint/README.md`.





# For expert allocation
## Terminal 1
roslaunch jetson_csi_cam jetson_csi_cam.launch


## Terminal 2
cd ~/Desktop
./camera-preview-ros.sh 

## Terminal 3
cd /home/drone/Documents/catkin_ws/src/drone_bringup/drone_stuff/Dynamic_Waypoint
./create_dynamic_mission.sh clear_anchor_waypoints.txt

## Terminal 4
cd /home/drone/Documents/catkin_ws/src/drone_bringup/drone_stuff
SKIP_NVARGUS_RESTART=1 ./start_mission_synchronized_camera.sh