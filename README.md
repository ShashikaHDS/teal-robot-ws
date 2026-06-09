# Teal Robot

ROS 2 Humble workspace for the **Teal / Meerkat** tracked robot — Jetson + Ouster
LiDAR + VectorNav IMU + ZED 2i + Oriental Motors + DS4 controller, running
LIO-SAM for mapping and Nav2 for autonomy.

This repository tracks the **user-authored** packages and scripts only.
Third-party dependencies (LIO-SAM, ouster-ros, vectornav, zed-ros2-wrapper,
ds4_driver, sick_scan_xd) are not redistributed here — clone them from upstream.
Robot-specific tweaks to LIO-SAM are preserved as drop-in files under
[`patches/`](patches/).

## Hardware geometry

| Frame | Offset from `base_link` (x, y, z, m) |
|---|---|
| `base_link` height above ground | 0.26 |
| `vectornav` (IMU) | (0.08, 0, 0) |
| `os_sensor` (3D LiDAR) | (0.145, 0, 0.31) |
| `camera_link` (ZED 2i) | (0.20, 0, 0.19) |

Defined in `src/meerkat_description/src/description/meerkat_description.urdf`
and the LIO-SAM patches.

## Packages in this workspace

| Package | Purpose |
|---|---|
| **`meerkat_autostart`** | Boot-time bringup + fullscreen Tk GUI + DS4 shortcuts for mode switching, map saving, and ZED capture. |
| `meerkat_description` | URDF / xacro for the robot. |
| `meerkat_motors` | Oriental Motors driver + DS4-to-`cmd_vel` bridge. |
| `meerkat_navigation` | Nav2 bringup, costmaps, AMCL params. |
| `meerkat_odometry` | Wheel odometry + EKF fusion config. |
| `pointcloud_colorize` | (Disabled via `COLCON_IGNORE`.) Colorize the LiDAR cloud using ZED RGB. |
| `teal_offline_coverage_bringup_ros2_humble` | Offline coverage-planning bringup. |
| `vlm_safety_msgs` / `vlm_safety_nodes` | Vision-LM safety messages + nodes. |
| `pcd_package` (zip only) | PCD utilities. |

Root-level helper scripts (kept for the manual workflow):

- `liosam.sh` — launch LIO-SAM (used by `meerkat_autostart` as a subprocess).
- `savemap.py` — keyboard map-saver, polls `D` to call `/lio_sam/save_map`.
- `zed_capture.sh` — start ZED wrapper + capture node under the `zed_ros` conda env.
- `zed_capture_node.py` — image/video capture node (DS4 + topic controlled).

## meerkat_autostart at a glance

A single Tkinter GUI plus a DS4 shortcut layer. Boot launches the foundation
(teleop + DS4 + ZED) and the GUI; mapping (LIO-SAM) and autonomy (Nav2) start
on demand.

**Modes** — Teleop / Mapping / Autonomous Navigation. Switching is exclusive
(the previous mode's subprocesses are SIGINT'd cleanly).

**DS4 shortcuts** (R1 is a modifier — hold R1 then press):

| Combo | Action |
|---|---|
| R1 + ✕ | Switch to Mapping |
| R1 + ◯ | Switch to Teleop |
| R1 + △ | Switch to Autonomous Navigation |
| R1 + SHARE | Save map (Mapping mode only) |
| R1 + OPTIONS | Toggle ZED video recording |

**GUI keys** — `F11`/`Esc` toggle fullscreen, `M` minimise to view RViz, `Q`
quits the entire bringup.

**ZED capture** — `/zed_capture/command` topic accepts:
`image | auto_on | auto_off | record_start | record_stop | interval=<sec>`.
The GUI publishes to this topic; the patched
[`zed_capture_node.py`](zed_capture_node.py) handles it.

## Build & run

```bash
# Install system deps
sudo apt install -y python3-tk

# Clone the third-party packages into src/ (one-time):
#   - https://github.com/TixiaoShan/LIO-SAM        → src/LIO-SAM
#   - https://github.com/ros-drivers/ds4_driver    → src/ds4_driver
#   - https://github.com/ouster-lidar/ouster-ros   → src/ouster-ros
#   - https://github.com/dawonn/vectornav          → src/vectornav
#   - https://github.com/stereolabs/zed-ros2-wrapper → src/zed-ros2-wrapper
#   - https://github.com/SICKAG/sick_scan_xd       → src/sick_scan_xd-develop
# Then apply the LIO-SAM patches:
cp patches/LIO-SAM/config/params.yaml         src/LIO-SAM/config/
cp patches/LIO-SAM/launch/transform.launch.py src/LIO-SAM/launch/
cp patches/LIO-SAM/launch/octomap.launch.py   src/LIO-SAM/launch/

# Build
colcon build --symlink-install
source install/setup.bash

# Launch the GUI bringup
ros2 launch meerkat_autostart bringup.launch.py
```

## Autostart at boot

```bash
mkdir -p ~/.config/autostart
cp src/meerkat_autostart/systemd/meerkat-autostart.desktop ~/.config/autostart/
```

Then enable GDM auto-login (edit `/etc/gdm3/custom.conf`, `[daemon]` section):

```
AutomaticLoginEnable=true
AutomaticLogin=teal
```

After reboot, the GUI comes up automatically (`DISPLAY=:0`, full screen).
Logs land in `~/.ros/meerkat-autostart.log`.
