# Dynamic waypoint mission

Drop **one** mission file here: `dynamic_mission.txt`.

The synchronized camera mission script clears the flight controller, then the
waypoint runner **waits** until `dynamic_mission.txt` exists, loads it, and
uploads that mission.

## Files

| File | Purpose |
|------|---------|
| `dynamic_mission.txt` | **The** mission to fly (created by you or `create_dynamic_mission.sh`) |
| `clear_anchor_waypoints.txt` | Bootstrap only — two points near home used to clear/prime the FC before the dynamic file is read |

Only `dynamic_mission.txt` is the active mission. Do not keep multiple mission
`.txt` files in this folder.

## Format

Same as other waypoint files — one per line:

```
lat, lon, alt_above_takeoff_m[, yaw_rad]
```

Lines starting with `#` and blank lines are ignored. Need at least 2 waypoints.

## Create / update mission

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff/Dynamic_Waypoint
./create_dynamic_mission.sh /path/to/waypoints.txt
# or edit dynamic_mission.txt directly
```

## Run

```bash
cd ~/Documents/catkin_ws/src/drone_bringup/drone_stuff
./start_mission_synchronized_camera.sh
# optional: custom clear anchor (default: Dynamic_Waypoint/clear_anchor_waypoints.txt)
# ./start_mission_synchronized_camera.sh /path/to/clear_anchor.txt 0.5 1.0
```

Take off first. After FC clear, logs will show waiting for
`Dynamic_Waypoint/dynamic_mission.txt` until you create it.

Disable dynamic mode (upload only from CLI waypoints file):

```bash
USE_DYNAMIC_MISSION=0 ./start_mission_synchronized_camera.sh ~/path/to/waypoints.txt
```
