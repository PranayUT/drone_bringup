#!/bin/bash
#
# camera-preview-ros-yolo.sh
# Fullscreen camera preview sourced from a ROS image topic, with Ultralytics
# YOLO inference overlaid on each frame using a TensorRT engine.
# Designed to be launched via xinit:
#
#   xinit ~/Desktop/camera-preview-ros-yolo.sh -- :0 vt3
#
# Usage:
#   xinit ./camera-preview-ros-yolo.sh -- :0 vt3
#   ROS_TOPIC=/csi_cam_0/image_raw xinit ./camera-preview-ros-yolo.sh -- :0 vt3
#   MODEL_PATH=/path/to/yolo26m.engine xinit ./camera-preview-ros-yolo.sh -- :0 vt3
#
# Env vars (all optional):
#   ROS_TOPIC   ROS image topic           (default: /csi_cam_0/image_raw)
#   MODEL_PATH  path to .engine file      (default: ~/yolo26m.engine)
#   CONF        confidence threshold      (default: 0.25)
#   IMGSZ       inference image size      (default: 640)
#
# Requires: ROS, cv_bridge, ultralytics (pip install ultralytics),
# and a TensorRT engine exported from a YOLO model.
# Stop with Ctrl+C.

set -euo pipefail

# If not running under X, re-launch via xinit.
if [ -z "${DISPLAY:-}" ]; then
    exec xinit "$0" "$@" -- :0 vt3
fi

# Source ROS environment if not already active.
if [ -z "${ROS_DISTRO:-}" ]; then
    for setup in /opt/ros/*/setup.bash; do
        # shellcheck source=/dev/null
        source "${setup}" && break
    done
fi

roslaunch jetson_csi_cam jetson_csi_cam.launch &
LAUNCH_PID=$!

ROS_TOPIC="${ROS_TOPIC:-/csi_cam_0/image_raw}"
TRANSPORT="${TRANSPORT:-raw}"
MODEL_PATH="${MODEL_PATH:-$HOME/yolo26m.engine}"
CONF="${CONF:-0.25}"
IMGSZ="${IMGSZ:-640}"

echo "Starting ROS camera preview with YOLO"
echo "  topic:     ${ROS_TOPIC}"
echo "  transport: ${TRANSPORT}"
echo "  model:     ${MODEL_PATH}"
echo "  conf:      ${CONF}"
echo "  imgsz:     ${IMGSZ}"
echo "  DISPLAY:   ${DISPLAY:-<not set>}"
echo ""
echo "Press Ctrl+C to stop."
echo ""

VIEW_PID=""
WATCHDOG_PID=""

start_view() {
    local w h
    read -r w h < <(xdotool getdisplaygeometry 2>/dev/null || echo "1920 1080")
    export _PREVIEW_TOPIC="${ROS_TOPIC}" \
           _PREVIEW_W="${w}" \
           _PREVIEW_H="${h}" \
           _PREVIEW_MODEL="${MODEL_PATH}" \
           _PREVIEW_CONF="${CONF}" \
           _PREVIEW_IMGSZ="${IMGSZ}"

    # Python subscriber runs YOLO on each frame, scales the annotated result
    # to screen dimensions, and moves the window to (0,0) — reliable in bare
    # xinit with no window manager, unlike wmctrl/xdotool which only resize
    # the outer X11 frame, not OpenCV's canvas.
    python3 - <<'PYEOF' &
import os, time, threading
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

bridge = CvBridge()
topic = os.environ["_PREVIEW_TOPIC"]
W = int(os.environ.get("_PREVIEW_W", 1920))
H = int(os.environ.get("_PREVIEW_H", 1080))
MODEL_PATH = os.environ["_PREVIEW_MODEL"]
CONF = float(os.environ.get("_PREVIEW_CONF", 0.25))
IMGSZ = int(os.environ.get("_PREVIEW_IMGSZ", 640))

print(f"Loading YOLO engine: {MODEL_PATH}")
# task='detect' is required when loading a .engine since the export
# metadata may not include it; adjust if you exported a seg/pose model.
model = YOLO(MODEL_PATH, task="detect")
# Warm up so the first real frame isn't slow.
import numpy as np
_ = model.predict(np.zeros((IMGSZ, IMGSZ, 3), dtype=np.uint8),
                  imgsz=IMGSZ, half=True, conf=CONF, verbose=False)
print("YOLO engine ready")

# Single-slot frame buffer + busy flag: if inference is still running,
# drop incoming frames instead of queueing them.
_lock = threading.Lock()
_latest = {"img": None, "stamp": 0.0}
_busy = False

# Simple FPS counter
_fps_t = time.time()
_fps_n = 0
_fps = 0.0

def cb(msg):
    global _busy
    if _busy:
        return
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logwarn(f"cv_bridge error: {e}")
        return
    with _lock:
        _latest["img"] = img
        _latest["stamp"] = time.time()

rospy.init_node("fullscreen_preview_yolo", anonymous=True)
# queue_size=1 + buff_size large enough for a full HD frame so old frames
# get dropped at the transport layer rather than piling up.
rospy.Subscriber(topic, Image, cb, queue_size=1, buff_size=2**24)

cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
cv2.resizeWindow("preview", W, H)
cv2.moveWindow("preview", 0, 0)

rate = rospy.Rate(60)
while not rospy.is_shutdown():
    with _lock:
        img = _latest["img"]
        _latest["img"] = None
    if img is None:
        rate.sleep()
        continue

    _busy = True
    try:
        results = model.predict(img, imgsz=IMGSZ, half=True,
                                conf=CONF, verbose=False)
        annotated = results[0].plot()
    finally:
        _busy = False

    # FPS overlay
    _fps_n += 1
    now = time.time()
    if now - _fps_t >= 1.0:
        _fps = _fps_n / (now - _fps_t)
        _fps_n = 0
        _fps_t = now
    cv2.putText(annotated, f"{_fps:.1f} FPS",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("preview", cv2.resize(annotated, (W, H)))
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
PYEOF
    VIEW_PID=$!
}

find_hdmi_connector() {
    local p
    for p in /sys/class/drm/card*-HDMI-A-*/status; do
        [ -f "$p" ] && { echo "$p"; return 0; }
    done
    return 1
}

# Polls DRM sysfs; kills the viewer on HDMI reconnect so the main loop restarts it.
hdmi_watchdog() {
    local connector prev_status status
    if ! connector=$(find_hdmi_connector); then
        echo "WARN: No HDMI connector found in sysfs, watchdog disabled"
        return
    fi
    echo "HDMI watchdog monitoring: ${connector}"
    prev_status=$(cat "${connector}")
    while true; do
        sleep 2
        status=$(cat "${connector}" 2>/dev/null) || continue
        if [ "${prev_status}" = "disconnected" ] && [ "${status}" = "connected" ]; then
            echo "HDMI reconnected — restarting viewer"
            kill ${VIEW_PID:+"${VIEW_PID}"} 2>/dev/null || true
        fi
        prev_status="${status}"
    done
}

trap 'kill ${VIEW_PID:+"${VIEW_PID}"} ${WATCHDOG_PID:+"${WATCHDOG_PID}"} ${LAUNCH_PID:+"${LAUNCH_PID}"} 2>/dev/null; wait 2>/dev/null; exit 0' INT TERM

start_view
hdmi_watchdog &
WATCHDOG_PID=$!

while true; do
    wait "${VIEW_PID}" || true
    echo "Viewer stopped — refreshing display output and restarting"
    xrandr --auto 2>/dev/null || true
    sleep 1
    start_view
done