#!/bin/bash
# Wrapper invoked by the XDG autostart .desktop entry.
# Sources ROS env and launches meerkat_autostart bringup.
mkdir -p /home/teal/.ros
exec >> /home/teal/.ros/meerkat-autostart.log 2>&1
echo "=== $(date '+%Y-%m-%d %H:%M:%S') boot ==="
source /opt/ros/humble/setup.bash
source /home/teal/Downloads/lio_ws/install/setup.bash
cd /home/teal/Downloads/lio_ws
exec ros2 launch meerkat_autostart bringup.launch.py
