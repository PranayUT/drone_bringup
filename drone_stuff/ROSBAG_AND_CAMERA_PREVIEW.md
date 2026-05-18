# Rosbag checks and camera preview (ROS Noetic)

Load ROS in **every new terminal**:

```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

Replace bag paths with your log folder, e.g. `~/logs/2026-05-14_15-15-22/mission.bag`.

---

## 1. Inspect a bag (metadata, topics, message counts)

```bash
rosbag info ~/logs/YOUR_STAMP/mission.bag
```

If you only have an unfinished recording:

```bash
rosbag info ~/logs/YOUR_STAMP/mission.bag.active
```

List topic names only:

```bash
rosbag info ~/logs/YOUR_STAMP/mission.bag | sed -n '/topics:/,$p'
```

---

## 2. Play back a bag (full stack or “clock only”)

**Terminal A — roscore (if nothing else is running):**

```bash
roscore
```

**Terminal B — play bag and publish `/clock` (good for nodes that use `use_sim_time`):**

```bash
rosbag play --clock ~/logs/YOUR_STAMP/mission.bag
```

If your nodes expect sim time:

```bash
rosparam set /use_sim_time true
rosbag play --clock ~/logs/YOUR_STAMP/mission.bag
```

Pause / seek / rate (examples):

```bash
rosbag play -r 0.5 --pause ~/logs/YOUR_STAMP/mission.bag   # half speed, start paused (space to toggle)
```

Record **only** some topics next time (smaller bags), e.g.:

```bash
LOG_DIR=~/logs/manual ./record_rosbag.sh /dji_sdk/gps_position /dji_sdk/flight_status /dji_sdk/main_camera_images
```

---

## 3. Camera preview (live flight or playback)

### 3a. Find which image topics exist

While **dji_sdk** is running or while **playing a bag** that recorded images:

```bash
rostopic list | grep -Ei 'image|camera|compressed'
```

`dji_sdk` (Onboard SDK ROS 3.8) can publish **`sensor_msgs/Image`** topics such as:

| Topic | Notes |
|--------|--------|
| `/dji_sdk/main_camera_images` | Main camera RGB stream (when streaming is enabled) |
| `/dji_sdk/fpv_camera_images` | FPV stream (when enabled) |
| `/dji_sdk/stereo_*_images` | Stereo feeds (when advanced sensing / subscriptions are active) |

Your `dji_sdk/launch/sdk.launch` currently has **`advanced_sensing: false`**. Without advanced sensing and without starting a camera stream via the DJI APIs/services, **there may be no image topics** and nothing to preview in the bag.

To get live camera into ROS you typically need:

- `advanced_sensing: true` in `sdk.launch` (if your platform supports it), and/or  
- Calling the **`dji_sdk/setup_camera_stream`** (and related subscription) services as documented for your aircraft — see DJI Onboard SDK ROS docs for your airframe.

### 3b. Preview with `rqt_image_view`

```bash
sudo apt-get install -y ros-noetic-rqt-image-view
rqt_image_view
```

In the GUI, choose the topic (e.g. `/dji_sdk/main_camera_images`).

**CLI one-liner (opens a viewer on a fixed topic):**

```bash
rosrun rqt_image_view rqt_image_view /image:=/dji_sdk/main_camera_images
```

(Adjust `/dji_sdk/main_camera_images` to whatever `rostopic list` shows.)

### 3c. Preview with RViz

```bash
rviz
```

Add **Image** display → set **Image Topic** to your topic (e.g. `/dji_sdk/main_camera_images`).

---

## 4. Run waypoint mission + record (your usual flow)

```bash
cd ~/Documents/drone_stuff
./start_mission_synchronized.sh ~/Documents/drone_stuff/ramsay_wps.txt
```

That starts `rosbag record -a` (all topics). If camera topics are not publishing, the bag will still have GPS, `flight_status`, IMU, etc.

---

## 5. Quick troubleshooting

| Symptom | What to try |
|---------|----------------|
| `rosbag: command not found` | `source /opt/ros/noetic/setup.bash` |
| Only `mission.bag.active` | Stop recording with **Ctrl+C** on `start_mission_synchronized.sh` so rosbag can finalize; or `rosbag reindex ...bag.active` |
| No image topics | Enable streaming / `advanced_sensing`; confirm hardware supports FPV/main feed over OSDK |
| Black / no frames in `rqt_image_view` | `rostopic hz /dji_sdk/main_camera_images` — if 0 Hz, stream is not running |

---

## 6. Optional: extract images from a bag to disk

Requires knowing the exact topic name and message type:

```bash
sudo apt-get install -y ros-noetic-image-view
rosrun image_view extract_images _sec_per_frame:=0.1 image:=/dji_sdk/main_camera_images _filename_format:=frame_%04d.jpg
# In another terminal:
rosbag play ~/logs/YOUR_STAMP/mission.bag
```

(`extract_images` saves under `~/.ros` by default unless you set `_filename_format` with a path.)
