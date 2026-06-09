#!/bin/bash
# =============================================================
# LIO-SAM Launcher
# Launches transform and run nodes
# =============================================================

# ── Setup ROS ───────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/Downloads/lio_ws/install/setup.bash

echo "============================================"
echo "  LIO-SAM Launcher"
echo "============================================"

# ── Launch transform node in background ─────────────────────
echo "[1/2] Starting LIO-SAM transform..."
ros2 launch lio_sam transform.launch.py &
TRANSFORM_PID=$!

sleep 3

# ── Launch run node ─────────────────────────────────────────
echo "[2/2] Starting LIO-SAM run..."
ros2 launch lio_sam run.launch.py &
RUN_PID=$!

echo ""
echo "============================================"
echo "  LIO-SAM running!"
echo "  Ctrl+C to stop everything"
echo "============================================"

# ── Cleanup on exit ─────────────────────────────────────────
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $RUN_PID 2>/dev/null
    kill $TRANSFORM_PID 2>/dev/null
    wait $RUN_PID 2>/dev/null
    wait $TRANSFORM_PID 2>/dev/null
    echo "Done."
}

trap cleanup SIGINT SIGTERM

wait -n $TRANSFORM_PID $RUN_PID
cleanup