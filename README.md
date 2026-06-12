# Teal Robot Workspace

ROS 2 Humble workspace for the **Teal / Meerkat** tracked robot — Jetson + Ouster
LiDAR + VectorNav IMU + ZED 2i + Oriental Motors, with a Tk GUI bringup and DS4
shortcuts that drive LIO-SAM mapping and Nav2 autonomy from the couch.

> **What's new** — `meerkat_autostart` adds a fullscreen Tk GUI bringup with DS4
> R1-modifier shortcuts; the robot now boots straight into mode-switchable
> Teleop / Mapping / Autonomous Nav. See [docs/AUTOSTART.md](docs/AUTOSTART.md).

## The robot

- **Compute** — NVIDIA Jetson, Ubuntu 22.04, Tegra kernel 5.15.
- **3D LiDAR** — Ouster (128 × 1024) via `ouster-ros`.
- **IMU** — VectorNav on `/dev/imu` (udev symlink).
- **Camera** — Stereolabs ZED 2i (HD720 @ 60 fps) via `zed-ros2-wrapper`.
- **Motors** — Oriental Motors, two RS-485 cables → `/dev/Motor_L` + `/dev/Motor_R`,
  driven by `meerkat_motors`.
- **Controller** — Sony DualShock 4 over Bluetooth via `ds4_driver`.
- **Storage** — 1 TB SSD mounted at `/media/teal/ssd1tb` (ext4, `nofail` in fstab);
  maps and ZED captures land here.

## Hardware geometry

Use these numbers — they're what the URDFs and LIO-SAM extrinsics ship with.

| Frame | Offset from `base_link` (x, y, z, m) |
|---|---|
| `base_link` height above ground | 0.26 |
| `vectornav` (IMU) | (0.08, 0, 0) |
| `os_sensor` (3D LiDAR) | (0.145, 0, 0.31) |
| `camera_link` (ZED 2i) | (0.20, 0, 0.19) |
| LIO-SAM `extrinsicTrans` (LiDAR ← IMU, base_link frame) | (0.065, 0.000, 0.310) |

Defined in [src/meerkat_description](src/meerkat_description) and mirrored in
[src/LIO-SAM/config/params.yaml](src/LIO-SAM/config/params.yaml) and
[src/LIO-SAM/launch/transform.launch.py](src/LIO-SAM/launch/transform.launch.py).

## What's tracked here vs third-party

**User-authored packages (tracked):**

| Package | Purpose |
|---|---|
| **`meerkat_autostart`** *(NEW)* | Tk GUI mode controller + DS4 shortcuts + subprocess manager + `bringup.launch.py` + `start_robot.sh` + `meerkat-autostart.desktop`. |
| `meerkat_description` | Robot URDFs. |
| `meerkat_motors` | Oriental Motors driver + DS4→`cmd_vel` bridge (publishes `/meerkat/cmd_vel`, `/odom`, `/ds4/trigger`, `/ds4/trigger2`). |
| `meerkat_navigation` | Nav2 bringup, costmaps, params. |
| `meerkat_odometry` | Wheel odometry + EKF fusion. |
| `pointcloud_colorize` | Colorize LiDAR cloud with ZED RGB. *Disabled via `COLCON_IGNORE`.* |
| `vlm_safety_msgs` / `vlm_safety_nodes` | VLM safety pipeline. |

Root-level scripts (also tracked): [liosam.sh](liosam.sh), [savemap.py](savemap.py),
[zed_capture.sh](zed_capture.sh), [zed_capture_node.py](zed_capture_node.py).

**Third-party packages tracked with our local modifications:**

| Package | Upstream | Why tracked |
|---|---|---|
| `LIO-SAM` | https://github.com/TixiaoShan/LIO-SAM | Extrinsics, Ouster 128×1024 tuning, `octomap.launch.py` + `transform.launch.py` added, `mapOptmization.cpp` / `imuPreintegration.cpp` / `imageProjection.cpp` local edits, save service tuning. |
| `vectornav` | https://github.com/dawonn/vectornav | Custom `vectornav.yaml`, `vectornav.cc` local edits, new `vectornav_liosam.launch.py`. |

You can clone teal-robot-ws and build directly — no upstream re-clone needed for
these two.

**Stock third-party packages — NOT in repo, clone into `src/` from upstream:**

| Package | Upstream |
|---|---|
| `ds4_driver` | https://github.com/naoki-mizuno/ds4_driver (ros2-humble branch) |
| `ouster-ros` | https://github.com/ouster-lidar/ouster-ros |
| `zed-ros2-wrapper` | https://github.com/stereolabs/zed-ros2-wrapper |
| `zed-ros2-examples` | https://github.com/stereolabs/zed-ros2-examples |
| `sick_scan_xd` | https://github.com/SICKAG/sick_scan_xd *(optional, if a SICK 2D LiDAR is present)* |

## Quick start

The full, do-it-once setup (apt packages, third-party clones, ZED conda env,
udev rules) is in **[docs/SETUP.md](docs/SETUP.md)**. The short version:

```bash
git clone https://github.com/ShashikaHDS/teal-robot-ws ~/Downloads/lio_ws
cd ~/Downloads/lio_ws
# follow docs/SETUP.md to clone the stock third-party deps into src/
colcon build --symlink-install
source install/setup.bash
ros2 launch meerkat_autostart bringup.launch.py
```

## Usage glance

Day-to-day driving, mapping, saving maps, ZED capture, and switching between
Teleop / Mapping / Autonomous Nav is in **[docs/USAGE.md](docs/USAGE.md)**. The
one-screen cheat sheet:

| Combo | Action |
|---|---|
| R1 + ✕ | Switch to Mapping |
| R1 + ◯ | Switch to Teleop |
| R1 + △ | Switch to Autonomous Nav |
| R1 + SHARE | Save map (Mapping mode) |
| R1 + OPTIONS | Toggle ZED recording |
| `F11` / `Esc` | GUI fullscreen toggle |
| `M` | Minimise GUI (view RViz map) |
| `Q` | Quit the entire bringup |

ZED command topic `/zed_capture/command` (`std_msgs/String`) accepts:
`image | auto_on | auto_off | record_start | record_stop | interval=<sec>`.

## Boot-time autostart

The GUI bringup comes up automatically via an XDG `.desktop` entry plus GDM
auto-login. Setup steps, log location (`~/.ros/meerkat-autostart.log`), and how
to disable it are in **[docs/AUTOSTART.md](docs/AUTOSTART.md)**.

## Troubleshooting

ZED `libgxf_isaac_optimizer.so` errors, IMU disappearing after swap, udev rules,
DS4 pairing, and other gotchas: see **[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)**.

## Repo

- GitHub: https://github.com/ShashikaHDS/teal-robot-ws
- Clone path on the robot: `~/Downloads/lio_ws`
- Branch: `main`
