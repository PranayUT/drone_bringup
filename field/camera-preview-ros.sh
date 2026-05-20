#!/bin/bash
#
# camera-preview-ros.sh
# Fullscreen camera preview sourced from a ROS image topic.
# Designed to be launched via xinit:
#
#   xinit ~/Desktop/camera-preview-ros.sh -- :0 vt3
#
# Usage:
#   xinit ./camera-preview-ros.sh -- :0 vt3
#   ROS_TOPIC=/csi_cam_0/image_raw xinit ./camera-preview-ros.sh -- :0 vt3
#
# Requires: ROS with image_view package (sudo apt install ros-<distro>-image-view)
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

echo "Starting ROS camera preview"
echo "  topic:     ${ROS_TOPIC}"
echo "  transport: ${TRANSPORT}"
echo "  DISPLAY:   ${DISPLAY:-<not set>}"
echo ""
echo "Press Ctrl+C to stop."
echo ""

VIEW_PID=""
WATCHDOG_PID=""

start_view() {
    local w h
    read -r w h < <(xdotool getdisplaygeometry 2>/dev/null || echo "1920 1080")
    export _PREVIEW_TOPIC="${ROS_TOPIC}" _PREVIEW_W="${w}" _PREVIEW_H="${h}"

    # Python subscriber scales each frame to screen dimensions and moves the
    # window to (0,0) — reliable in bare xinit with no window manager, unlike
    # wmctrl/xdotool which only resize the outer X11 frame, not OpenCV's canvas.
    python3 - <<'PYEOF' &
import os, rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
topic = os.environ["_PREVIEW_TOPIC"]
W = int(os.environ.get("_PREVIEW_W", 1920))
H = int(os.environ.get("_PREVIEW_H", 1080))

def cb(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("preview", cv2.resize(img, (W, H)))
    cv2.waitKey(1)

rospy.init_node("fullscreen_preview", anonymous=True)
rospy.Subscriber(topic, Image, cb)
cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
cv2.resizeWindow("preview", W, H)
cv2.moveWindow("preview", 0, 0)
rospy.spin()
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
