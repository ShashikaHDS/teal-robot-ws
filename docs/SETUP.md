# Teal Robot — Setup Guide (Fresh Jetson)

This document walks through bringing up the **teal-robot-ws** stack on a *fresh* NVIDIA Jetson running Ubuntu 22.04. By the end you will have a Jetson that boots, auto-logs in, builds the workspace, recognises every USB device, and launches the full bringup with one command.

For an architectural overview of the packages see [../README.md](../README.md). For runtime issues consult [TROUBLESHOOTING.md](TROUBLESHOOTING.md), and for sensor placement/calibration see [HARDWARE.md](HARDWARE.md).

---

## 1. Prerequisites

You must start from a Jetson that has already been imaged with the standard NVIDIA L4T stack. Before beginning this guide make sure all of the following are true:

| Requirement | Expected value |
| --- | --- |
| Hardware | NVIDIA Jetson (compute module + carrier) |
| OS | Ubuntu 22.04 LTS (aarch64) |
| Kernel | Tegra 5.15 |
| CUDA | Pre-installed by the L4T BSP (do *not* reinstall) |
| ROS 2 | Humble Hawksbill, installed at `/opt/ros/humble` |
| User | `teal` with sudo rights |
| Network | Internet access for apt / git clones |

You can verify ROS 2 with:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
printenv ROS_DISTRO     # must print: humble
```

If ROS 2 Humble is *not* installed, follow the official Open Robotics instructions for Ubuntu 22.04 before continuing — this guide assumes it is already present.

The robot uses the following hardware, all of which is expected to be wired in:

- **3D LiDAR**: Ouster (driven by `ouster-ros`).
- **IMU**: VectorNav (driven by `vectornav`, exposed as `/dev/imu`).
- **Camera**: Stereolabs ZED 2i (driven by `zed-ros2-wrapper`).
- **Motors**: Oriental Motors over two RS-485 links → `/dev/Motor_L` (serial `AV0LXAGU`) and `/dev/Motor_R` (serial `AV0LZ81W`).
- **Controller**: Sony DualShock 4 over Bluetooth (driven by `ds4_driver`).
- **Storage**: A 1 TB SSD mounted at `/media/teal/ssd1tb` (ext4) for maps and recordings.

---

## 2. System apt dependencies

Install every system package the workspace depends on. `python3-tk` is **required** for the bringup GUI — do not skip it.

```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-nav2-bringup \
    ros-humble-octomap-server \
    ros-humble-pcl-ros \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    python3-tk \
    python3-colcon-common-extensions \
    build-essential \
    cmake
```

The ZED ROS 2 wrapper uses the NITROS / GXF path on Jetson. The Isaac GXF extensions ship as part of the L4T-tuned ROS 2 Humble image and are already on this machine:

```bash
dpkg -l | grep -E 'ros-humble-gxf-isaac' | head
```

If for any reason that list is empty, install them with:

```bash
sudo apt install -y 'ros-humble-gxf-isaac-*'
```

| Package | Why it is needed |
| --- | --- |
| `ros-humble-desktop` | Core ROS 2, RViz2, demo nodes |
| `ros-humble-nav2-bringup` | Nav2 stack used by `meerkat_navigation` |
| `ros-humble-octomap-server` | 3D occupancy grid generated from LIO-SAM |
| `ros-humble-pcl-ros` | Point cloud filters and helpers |
| `ros-humble-cv-bridge` | ROS ↔ OpenCV image conversion (ZED capture node) |
| `ros-humble-tf2-ros`, `ros-humble-tf2-tools` | TF tree publication & debugging |
| `python3-tk` | Tk GUI rendered by `mode_controller` |
| `python3-colcon-common-extensions` | `colcon build`, mixins, etc. |
| `build-essential`, `cmake` | C++ build chain for all packages |
| `ros-humble-gxf-isaac-*` | GXF runtime needed by ZED NITROS components |

---

## 3. Conda + ZED SDK environment

`zed_capture.sh` runs the ZED stack inside a dedicated **Miniconda** environment called `zed_ros`. This isolates the Stereolabs ZED SDK Python bindings from the system Python so the ROS 2 nodes still work.

### 3.1 Install Miniconda

Install Miniconda into the canonical location used by `zed_capture.sh`:

```bash
cd /tmp
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh -b -p $HOME/miniconda3
```

Initialise the shell so `conda` is on PATH for interactive sessions (the launch script sources `conda.sh` directly, so this step is mainly for you):

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda init bash
```

### 3.2 Create the `zed_ros` environment

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda create -n zed_ros python=3.10 -y
conda activate zed_ros
```

### 3.3 Install ZED SDK Python bindings

The Stereolabs ZED SDK must already be installed system-wide (it ships its own `.run` installer). With the SDK in place, install its Python bindings into the conda env:

```bash
# Inside (zed_ros)
python -m pip install --upgrade pip
python -m pip install cython numpy opencv-python
python /usr/local/zed/get_python_api.py
```

You also need the OpenCV ROS bridge and basic image utilities available to the conda Python so `zed_capture_node.py` can import them:

```bash
python -m pip install opencv-contrib-python
```

### 3.4 Verify the env and PYTHONNOUSERSITE

`zed_capture.sh` activates the env like this:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate zed_ros
export PYTHONNOUSERSITE=1
```

`PYTHONNOUSERSITE=1` blocks `~/.local/lib/pythonX.Y/site-packages` from leaking into the env, which is essential — otherwise user-site packages installed against the system Python override the conda copies.

Test it manually:

```bash
conda activate zed_ros
export PYTHONNOUSERSITE=1
python -c "import pyzed.sl as sl; print(sl.Camera().get_sdk_version())"
```

---

## 4. udev rules — stable device symlinks

Every USB-serial device on the robot is referenced by a stable `/dev/<name>` symlink created by udev. The mapping lives in `/etc/udev/rules.d/99-usb-serial.rules`.

### 4.1 Find your serials

Plug everything in, then run:

```bash
ls -l /dev/serial/by-id
```

You will see entries that include the FTDI / Silicon Labs serial number of each adapter. Note the serial for each device:

- the USB hub itself (currently `BG00PJ1W`),
- the left motor RS-485 cable (currently `AV0LXAGU`),
- the right motor RS-485 cable (currently `AV0LZ81W`),
- the VectorNav IMU (currently `AU04Q1UG`).

**These serials are per-device.** When you swap an FTDI cable or replace the IMU you must update the matching line below.

### 4.2 Write the rules file

Create the file:

```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

Paste the snapshot used on this robot (replace each serial with your own):

```udev
SUBSYSTEM=="tty", ATTRS{serial}=="BG00PJ1W", SYMLINK+="usb_hub"
SUBSYSTEM=="tty", ATTRS{serial}=="AV0LXAGU", SYMLINK+="Motor_L"
SUBSYSTEM=="tty", ATTRS{serial}=="AV0LZ81W", SYMLINK+="Motor_R"
SUBSYSTEM=="tty", ATTRS{serial}=="AU04Q1UG", SYMLINK+="imu"
```

This produces four predictable device paths:

| Symlink | Used by |
| --- | --- |
| `/dev/usb_hub` | book-keeping for the powered USB hub |
| `/dev/Motor_L` | `meerkat_motors` — left motor |
| `/dev/Motor_R` | `meerkat_motors` — right motor |
| `/dev/imu` | `vectornav` IMU driver |

### 4.3 Reload udev

After editing the rules file, ask udev to re-evaluate every tty device:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
```

If only the IMU is misbehaving you can target it specifically (replace `N` with the actual `ttyUSBN` it currently shows up as in `dmesg`):

```bash
sudo udevadm trigger --action=add --sysname-match=ttyUSBN
```

Verify:

```bash
ls -l /dev/Motor_L /dev/Motor_R /dev/imu
```

All three should be symlinks pointing at the underlying `ttyUSB*` nodes.

### 4.4 Group memberships

`teal` must be in `dialout` (to open `/dev/tty*`) and `bluetooth` (for `bluetoothctl` and `ds4_driver`):

```bash
sudo usermod -aG dialout teal
sudo usermod -aG bluetooth teal
```

Log out and back in (or reboot) for the new groups to take effect.

---

## 5. Bluetooth pairing for the DualShock 4

`ds4_driver` reads the controller over Bluetooth HID. The DS4 must be **paired _and_ trusted** so it auto-reconnects after every reboot — pairing alone is not enough.

### 5.1 Prepare the controller

1. Make sure the DS4 is fully charged or wired to a charger.
2. Hold **SHARE + PS** for ~3 seconds until the lightbar starts pulsing rapidly in white. This puts the pad into pairing mode.

### 5.2 Run bluetoothctl

```bash
bluetoothctl
```

Inside the interactive prompt:

```
power on
agent on
default-agent
scan on
```

Wait until you see a line like:

```
[NEW] Device A1:B2:C3:D4:E5:F6 Wireless Controller
```

Copy that MAC address, then run (substituting your MAC):

```
trust A1:B2:C3:D4:E5:F6
pair A1:B2:C3:D4:E5:F6
connect A1:B2:C3:D4:E5:F6
trust A1:B2:C3:D4:E5:F6
scan off
exit
```

The second `trust` after `connect` is intentional and important: it tells BlueZ to remember the device across boots so it reconnects automatically when the controller wakes up.

### 5.3 Verify

After the next reboot, press the **PS button** on the controller. The lightbar should turn solid blue within a few seconds without you needing to touch `bluetoothctl`. Confirm with:

```bash
ls /dev/input/js* 2>/dev/null
```

A `js0` (or similar) node means `ds4_driver` will find it.

---

## 6. Clone the workspace and third-party sources

The user-authored packages live in this repo. **LIO-SAM and vectornav are also tracked here directly** because they have substantial local modifications (extrinsics, Ouster tuning, custom launch files, C++ source edits). The remaining stock vendor drivers (ZED wrapper, Ouster, ds4_driver, optional SICK) are *not* tracked here and must be cloned alongside them into `src/`.

Copy-paste this whole block. It clones the workspace, then every stock third-party dependency at its upstream URL:

```bash
# 1. Workspace root
mkdir -p ~/Downloads
cd ~/Downloads
git clone https://github.com/ShashikaHDS/teal-robot-ws.git lio_ws
cd lio_ws/src

# 2. Stock third-party drivers (clone into src/)
git clone https://github.com/naoki-mizuno/ds4_driver.git
git clone https://github.com/ouster-lidar/ouster-ros.git
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-examples.git

# 3. OPTIONAL — only if a SICK 2D lidar is fitted
git clone https://github.com/SICKAG/sick_scan_xd.git
```

When checking out `ds4_driver`, make sure you are on the branch that supports **ROS 2 Humble** (check the project's README — `master` may default to ROS 1).

After cloning, your `src/` should contain:

- In-repo, user-authored: `meerkat_autostart`, `meerkat_description`, `meerkat_motors`, `meerkat_navigation`, `meerkat_odometry`, `pointcloud_colorize`, `vlm_safety_msgs`, `vlm_safety_nodes`
- In-repo, third-party tracked with local mods: `LIO-SAM`, `vectornav`
- Freshly cloned stock third-party: `ds4_driver`, `ouster-ros`, `zed-ros2-wrapper`, `zed-ros2-examples`, optional `sick_scan_xd`

Note: [src/pointcloud_colorize](../src/pointcloud_colorize) is intentionally disabled via a `COLCON_IGNORE` file — leave it in place, colcon will skip the package.

### What was changed in the tracked LIO-SAM and vectornav

You don't need to apply any patches — both are already pinned to the right state. For reference, here's the gist of the local modifications:

| File | What it sets |
| --- | --- |
| `src/LIO-SAM/config/params.yaml` | `extrinsicTrans: [0.065, 0.000, 0.310]`, IMU topic `/vectornav/imu`, Ouster sensor, `N_SCAN=128`, `H_SCAN=1024`, save service `/lio_sam/save_map`, `lidarMinRange = 0.6` |
| `src/LIO-SAM/launch/transform.launch.py` | Static TFs `base_link → os_sensor` `(0.145, 0, 0.31)` and `base_link → vectornav` `(0.08, 0, 0)` |
| `src/LIO-SAM/launch/octomap.launch.py` | `occupancy_min_z = -0.52`, `occupancy_max_z = 0.33` for the 0.26 m `base_link` height |
| `src/LIO-SAM/src/*.cpp`, `include/lio_sam/utility.hpp` | Local C++ tweaks — see git blame for specifics. |
| `src/vectornav/config/vectornav.yaml` | Serial port `/dev/imu`, baud rate, output rate matched to this robot's IMU. |
| `src/vectornav/src/vectornav.cc` | Local edits — see git blame for specifics. |
| `src/vectornav/launch/vectornav_liosam.launch.py` | New launch file that wires the IMU output to what LIO-SAM expects. |

---

## 7. Build the workspace

```bash
cd ~/Downloads/lio_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`--symlink-install` lets you edit Python scripts and launch files in-place without rebuilding.

After the first build completes, source the overlay (and add it to your shell startup if you want it permanent):

```bash
source ~/Downloads/lio_ws/install/setup.bash
echo 'source ~/Downloads/lio_ws/install/setup.bash' >> ~/.bashrc
```

If the build fails on a single package, rebuild just that one with:

```bash
colcon build --symlink-install --packages-select <package_name>
```

---

## 8. First-launch smoke test

Power-cycle the robot so udev sees every device freshly, then run the full bringup:

```bash
source /opt/ros/humble/setup.bash
source ~/Downloads/lio_ws/install/setup.bash
ros2 launch meerkat_autostart bringup.launch.py
```

This launch file:

1. Includes `meerkat_motors/meerkat_teleop_launch.py` to bring up motors + DS4 → `cmd_vel` bridge.
2. Runs [../zed_capture.sh](../zed_capture.sh) as an `ExecuteProcess` (so the `zed_ros` conda env is preserved).
3. Starts the `mode_controller` node, which renders the Tk GUI.

### What to verify

- A full-screen Tk window appears.
- Press **F11** / **Esc** to toggle full-screen; **M** minimises so you can see RViz; **Q** quits the whole bringup.
- Press the **PS** button on the DS4 — the lightbar should turn solid blue.
- Drive a tiny amount in **Teleop** mode (R1 + ◯) to confirm `/meerkat/cmd_vel` reaches the motors and `/odom` updates.
- A preview window from `zed_capture_node.py` shows the ZED RGB feed. Press **X** to capture a frame, **A** to toggle auto-capture, **R** to toggle recording, **Q** to quit the preview.
- `ros2 topic list` shows `/vectornav/imu`, `/ouster/points`, `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/depth/depth_registered`, `/odom`, `/ds4/trigger`, `/ds4/trigger2`, `/zed_capture/command`.

### DS4 shortcut summary (R1 modifier)

| Chord | Action |
| --- | --- |
| R1 + ✕ | Mapping (starts LIO-SAM subprocess) |
| R1 + ◯ | Teleop |
| R1 + △ | Autonomous Nav (launches `meerkat_navigation meerkat_bringup.launch.py`) |
| R1 + SHARE | Save map (calls `/lio_sam/save_map` with versioned directory) |
| R1 + OPTIONS | Toggle ZED recording |

Saved maps land under `/media/teal/ssd1tb/lio_sam_maps/<DDMMYY>/verN/`, matching the versioning used by [../savemap.py](../savemap.py).

---

## 9. Mount the 1 TB SSD at `/media/teal/ssd1tb`

All maps and ZED captures go to the SSD. Mount it persistently with `nofail` so a missing drive cannot block boot.

### 10.1 Identify the SSD

```bash
lsblk -o NAME,SIZE,FSTYPE,UUID,MOUNTPOINT
```

Find the ext4 partition that corresponds to the 1 TB drive and copy its **UUID**.

### 10.2 Create the mount point

```bash
sudo mkdir -p /media/teal/ssd1tb
sudo chown teal:teal /media/teal/ssd1tb
```

### 10.3 Add it to `/etc/fstab`

Edit fstab:

```bash
sudo nano /etc/fstab
```

Append a line of the form (substitute your UUID):

```fstab
UUID=xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx  /media/teal/ssd1tb  ext4  defaults,nofail,x-systemd.device-timeout=5s  0  2
```

The crucial option is `nofail`: if the SSD is ever absent, the boot still completes and `mode_controller` will simply fail to write captures rather than the whole machine refusing to come up.

### 10.4 Mount and verify

```bash
sudo systemctl daemon-reload
sudo mount -a
df -h /media/teal/ssd1tb
```

You should see the drive mounted with plenty of free space.

Pre-create the directories the runtime expects:

```bash
mkdir -p /media/teal/ssd1tb/lio_sam_maps
mkdir -p /media/teal/ssd1tb/zed_captures
```

---

## Next steps

- Set up boot-time autostart so `bringup.launch.py` launches automatically after auto-login — see the `~/.config/autostart/meerkat-autostart.desktop` entry and the `start_robot.sh` wrapper under [../src/meerkat_autostart](../src/meerkat_autostart). GDM auto-login is configured via `/etc/gdm3/custom.conf` (`AutomaticLoginEnable=true`, `AutomaticLogin=teal`).
- Review [../README.md](../README.md) for the full package tree and runtime topics.
- If something fails, check [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — common gotchas include the missing GXF `libgxf_isaac_optimizer.so` on ZED launch, a missing `/dev/imu` symlink after swapping IMUs, and a DS4 that was paired but not trusted.
