#!/bin/bash
# =============================================================
# ZED 2i All-in-One Launcher (Capture + Record)
#
# Images -> /media/teal/ssd1tb/zed2_images/DDMMYYYY/rgb/
# Videos -> /media/teal/ssd1tb/zed2_videos/DDMMYYYY/
# =============================================================

# ── Config ───────────────────────────────────────────────────
RESOLUTION="HD720"
FPS=60
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CAPTURE_SCRIPT="$SCRIPT_DIR/zed_capture_node.py"

# ── Setup conda + ROS ───────────────────────────────────────
source ~/miniconda3/etc/profile.d/conda.sh
conda activate zed_ros
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash

DATE=$(date +%d%m%Y)

echo "============================================"
echo "  ZED 2i Capture + Record"
echo "  Resolution: $RESOLUTION @ ${FPS}fps"
echo "  Images:  /media/teal/ssd1tb/zed2_images/$DATE/rgb/"
echo "  Videos:  /media/teal/ssd1tb/zed2_videos/$DATE/"
echo "============================================"

# ── Launch ZED camera in background ─────────────────────────
echo "[1/2] Starting ZED camera..."
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i \
    general.grab_resolution:=$RESOLUTION \
    general.grab_frame_rate:=$FPS &
ZED_PID=$!

echo "Waiting for camera to start..."
sleep 8

# ── Launch capture + record node ────────────────────────────
echo "[2/2] Starting capture + record node..."
python3 "$CAPTURE_SCRIPT" --ros-args -p video_fps:=${FPS}.0 &
CAPTURE_PID=$!

echo ""
echo "============================================"
echo "  Controls:"
echo "  TRIANGLE (short) / X  = Single capture"
echo "  TRIANGLE (hold)  / A  = Toggle auto-capture"
echo "  R                     = Toggle video recording"
echo "  Q                     = Quit"
echo "============================================"

# ── Cleanup on exit ─────────────────────────────────────────
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $CAPTURE_PID 2>/dev/null
    kill $ZED_PID 2>/dev/null
    wait $CAPTURE_PID 2>/dev/null
    wait $ZED_PID 2>/dev/null
    echo "Done."
}

trap cleanup SIGINT SIGTERM

wait -n $ZED_PID $CAPTURE_PID
cleanup