# CSI camera recording (experimental)

Separate from your normal mission scripts. If this works, you can merge into
`bringup_synchronized.launch` / `start_mission_synchronized.sh` later.

| Normal (unchanged) | Experimental |
|--------------------|--------------|
| `bringup_synchronized.launch` | `bringup_synchronized_camera.launch` |
| `start_mission_synchronized.sh` | `start_mission_synchronized_camera.sh` |
| `record_rosbag.sh` | `record_rosbag_camera.sh` |
| — | `csi_camera_only.launch` |
| — | `test_camera_record.sh` |

`bringup.launch` is still unchanged (CSI block commented out).

## Step 1 — camera only (no FC)

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff
./test_camera_record.sh 30
```

Or launch camera alone:

```bash
roslaunch drone_bringup csi_camera_only.launch
```

Preview: `rosrun rqt_image_view rqt_image_view /image:=/csi_cam_0/image_raw`

## Step 2 — synchronized mission + CSI

**Dynamic mission (default):** edit `Dynamic_Waypoint/clear_anchor_waypoints.txt` for your
takeoff area, take off, start the script, then create the real mission when ready:

```bash
./start_mission_synchronized_camera.sh
# after FC clear (see bringup.log), in another terminal:
./Dynamic_Waypoint/create_dynamic_mission.sh ~/path/to/waypoints.txt
```

The runner waits for `Dynamic_Waypoint/dynamic_mission.txt` (only one mission file in that folder).

**Fixed mission file** (no dynamic wait):

```bash
USE_DYNAMIC_MISSION=0 ./start_mission_synchronized_camera.sh ~/path/to/waypoints.txt
```

Records `mission.bag` with `rosbag record -a` (includes `/csi_cam_0/image_raw` when gscam is up).

## Rosbag → MP4 video

After you have a `.bag`:

```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/catkin_ws/devel/setup.bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff

python3 bag_to_video.py /media/drone/extreme1/logs/camera_test_YYYY-MM-DD_HH-MM-SS/camera_test.bag
```

Writes **`camera_test.mp4`** next to the bag (same name, `.mp4` extension).

Options:

```bash
python3 bag_to_video.py mission.bag -o ~/Videos/flight.mp4
python3 bag_to_video.py camera_test.bag -t /csi_cam_0/image_raw/compressed
python3 bag_to_video.py camera_test.bag --fps 30
```

List image topics in a bag: `rosbag info your.bag`

## YOLO masks + timing + rosbag (same flight, no CSI fight)

**One** process owns the camera (`gscam`). YOLO reads ROS; disk output is unchanged (`yolo_records/...`).

**Terminal 1** — camera (+ optional mission bag):

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff
SKIP_NVARGUS_RESTART=1 ./test_camera_record.sh 30
# OR mission:
# ./start_mission_synchronized_camera.sh ~/path/to/waypoints.txt
```

Wait for **`camera publishing`** / recording started.

**Terminal 2** — YOLO (after `/csi_cam_0/image_raw` is up):

```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/catkin_ws/devel/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
cd ~/Documents/catkin_ws/src/drone_bringup/yolo_stuff
python3 record_yolo_masks.py --realworld-ros --weights best-v1.engine --half
```

Still writes to **`/media/drone/extreme1/yolo_records/<stamp>/`** (masks.json, timing CSV, etc.).

Also publishes for rosbag (`rosbag record -a` in mission script picks these up):

| Topic | Type | Same as on disk |
|-------|------|-----------------|
| `/yolo/detections_json` | `std_msgs/String` | `frames/.../detections.json` body |
| `/yolo/masks_json` | `std_msgs/String` | `frames/.../masks.json` |
| `/yolo/timing_json` | `std_msgs/String` | per-frame timing row |

Optional preview in bag (large): add `--ros-publish-preview` → `/yolo/preview/image`.

**Do not** use `--realworld` (direct GStreamer) while ROS owns the camera.

## Notes

- Only one **CSI driver** at a time (`gscam` **or** `record_yolo_masks.py --realworld`, not both).
- `--realworld-ros` is fine alongside `test_camera_record.sh` / mission camera bringup.
- Default resolution: 1280×720 @ 30 (override via launch args or `CAMERA_WIDTH` etc. on the test script).
