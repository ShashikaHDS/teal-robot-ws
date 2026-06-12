# Troubleshooting

Each section is symptom -> cause -> fix for the teal-robot-ws stack on the
Jetson (Ubuntu 22.04, ROS 2 Humble, LIO-SAM, ZED 2i, VectorNav, Ouster,
Oriental Motors, DualShock 4).

See also: [../README.md](../README.md),
[SETUP.md](SETUP.md),
[../src/LIO-SAM/](../src/LIO-SAM/),
[../src/meerkat_autostart/](../src/meerkat_autostart/).

---

## ZED node fails with `libgxf_isaac_optimizer.so: cannot open shared object file`

**Symptom.** `ros2 launch zed_wrapper zed_camera.launch.py` (or
[../zed_capture.sh](../zed_capture.sh)) aborts during component load with
`libgxf_isaac_optimizer.so: cannot open shared object file: No such file or directory`.

**Cause.** The ZED wrapper uses the NVIDIA Isaac NITROS path; its GXF
extensions live under `/opt/ros/humble/share/gxf_isaac_*/gxf/lib` and
`/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/*`. Those directories are not on
the default loader search path, so `dlopen` fails. The `ros-humble-gxf-isaac-*`
packages are installed on this machine - only `LD_LIBRARY_PATH` is wrong.

**Fix.** Export the GXF lib dirs before launching ZED:

```bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:\
$(ls -d /opt/ros/humble/share/gxf_isaac_*/gxf/lib 2>/dev/null | tr '\n' ':')\
$(ls -d /opt/ros/humble/share/isaac_ros_gxf/gxf/lib/* 2>/dev/null | tr '\n' ':')"
```

**Make it permanent.** Pick one:

1. Append the same export to `~/.bashrc` - applies to every interactive shell.
2. Add it to
   [../src/meerkat_autostart/scripts/start_robot.sh](../src/meerkat_autostart/scripts/start_robot.sh)
   right after the ROS source lines. This is the safer option: boot-time
   bringup uses GDM auto-login and never sources `~/.bashrc`, so option (1)
   alone will not help the autostart path.

---

## `/dev/imu` missing after IMU swap

**Symptom.** `ls /dev/imu` returns "No such file or directory", VectorNav
cannot open the port, `/vectornav/imu` never publishes.

**Cause.** `/dev/imu` is a udev symlink in
`/etc/udev/rules.d/99-usb-serial.rules`, keyed off the IMU's USB serial. A
swapped unit has a different serial, so no symlink is created. Current rule
snapshot:

```
SUBSYSTEM=="tty", ATTRS{serial}=="BG00PJ1W", SYMLINK+="usb_hub"
SUBSYSTEM=="tty", ATTRS{serial}=="AV0LXAGU", SYMLINK+="Motor_L"
SUBSYSTEM=="tty", ATTRS{serial}=="AV0LZ81W", SYMLINK+="Motor_R"
SUBSYSTEM=="tty", ATTRS{serial}=="AU04Q1UG", SYMLINK+="imu"
```

**Pre-check the new serial.** Plug the new IMU in and read it off
`/dev/serial/by-id/`:

```bash
ls -l /dev/serial/by-id/
```

The trailing token before `-if00` is the serial.

**Fix.** Swap the old serial (`AU04Q1UG`) for the new one and reload udev.
Replace `XXXXXXXX` with the actual new serial and `ttyUSBN` with the actual
device node (check `dmesg | tail` right after plugging it in):

```bash
sudo sed -i 's/AU04Q1UG/XXXXXXXX/' /etc/udev/rules.d/99-usb-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --sysname-match=ttyUSBN
ls -l /dev/imu
```

---

## VectorNav driver: `Unable to connect to device /dev/imu`

**Symptom.** The VectorNav node logs `Unable to connect to device /dev/imu`
and exits. LIO-SAM (configured against `/vectornav/imu` in
[../src/LIO-SAM/config/params.yaml](../src/LIO-SAM/config/params.yaml))
never initialises.

**Cause.** Same root cause as the previous section in most cases: the symlink
is not present. A second, independent cause is permissions - the user must be
in `dialout` to open the serial port even when the symlink exists.

**Fix.**

1. Check the symlink. If missing, follow
   [`/dev/imu` missing after IMU swap](#devimu-missing-after-imu-swap):

   ```bash
   ls -l /dev/imu
   ```

2. Check group membership - `dialout` must appear:

   ```bash
   groups
   sudo usermod -aG dialout $USER   # then log out / log back in
   ```

3. Confirm nothing else is holding the port:

   ```bash
   sudo fuser -v /dev/imu
   ```

---

## DS4 disconnects after boot

**Symptom.** The DualShock 4 is paired but never reconnects after a reboot.
The GUI still appears, but R1 chord shortcuts (R1+X Mapping, R1+O Teleop,
R1+Triangle Autonomy, R1+SHARE Save Map, R1+OPTIONS ZED record) do nothing.

**Cause.** BlueZ only auto-reconnects controllers that are both **paired**
**and trusted**. A pair without a trust does not survive reboot. `rfkill` may
also have soft-blocked the radio, or the autostart bringup may be racing
BlueZ at login.

**Fix.**

1. Make sure the radio is not blocked:

   ```bash
   rfkill list
   sudo rfkill unblock bluetooth
   ```

2. Pair *and* trust. Put the DS4 into pairing mode (hold SHARE + PS):

   ```bash
   bluetoothctl
   power on
   agent on
   default-agent
   scan on
   pair  AA:BB:CC:DD:EE:FF
   trust AA:BB:CC:DD:EE:FF
   connect AA:BB:CC:DD:EE:FF
   scan off
   exit
   ```

**Timing.** The DS4 must connect **before** `mode_controller` starts, since
both `mode_controller` and the meerkat_motors DS4 bridge attach to ds4_driver
topics (`/ds4/trigger`, `/ds4/trigger2`) at start-up. If the controller is
not up in time, the GUI will still come up (it does not depend on the DS4),
but R1 chords will be inert until the DS4 connects. Pressing PS on the DS4
after the GUI is up is usually enough to recover - no restart needed.

---

## RViz Color Transformer does not show RGB8 option for `/colored_points`

**Symptom.** In RViz, the PointCloud2 display for `/colored_points` only
offers Intensity / AxisColor - no RGB8 entry.

**Cause.** `/colored_points` is not being published, because
[../src/pointcloud_colorize/](../src/pointcloud_colorize/) is disabled via
`COLCON_IGNORE`. RViz only exposes an RGB8 transformer when a frame on the
topic actually carries an `rgb` field; with the package ignored, no publisher
exists.

**Fix.** Only re-enable if you actually need colorize. If you do:

1. Rebuild the package:

   ```bash
   rm src/pointcloud_colorize/COLCON_IGNORE
   colcon build --packages-select pointcloud_colorize
   source install/setup.bash
   ```

2. Verify ZED is publishing - without RGB samples the colorizer output stays
   empty:

   ```bash
   ros2 topic hz /zed/zed_node/rgb/image_rect_color
   ```

3. Confirm the TF chain reaches the ZED optical frame. The colorizer projects
   LiDAR points into the camera, so
   `os_sensor -> base_link -> camera_link -> zed_left_camera_optical_frame`
   must resolve:

   ```bash
   ros2 run tf2_ros tf2_echo os_sensor zed_left_camera_optical_frame
   ```

   If that errors, ZED itself is not running (it publishes the optical-frame
   TFs) or a static TF from
   [../src/LIO-SAM/launch/transform.launch.py](../src/LIO-SAM/launch/transform.launch.py)
   /
   [../src/meerkat_description/](../src/meerkat_description/) is missing.

---

## Mapping starts but no point cloud in RViz

**Symptom.** Mapping mode (R1+X) is entered, LIO-SAM comes up, but no cloud
ever appears in RViz.

**Cause.** Three usual suspects: LIO-SAM is still warming up on IMU; you are
looking at the wrong topic; or the static TFs from `transform.launch.py` are
not being published so RViz cannot resolve the cloud's frame.

**Fix.**

1. Wait a few seconds with the robot still. LIO-SAM is not instant.

2. Subscribe to the registered cloud, with Fixed Frame `map`:

   ```bash
   ros2 topic hz /lio_sam/mapping/cloud_registered
   ```

   Set the RViz PointCloud2 display topic to
   `/lio_sam/mapping/cloud_registered`.

3. Confirm the static TFs from
   [../src/LIO-SAM/launch/transform.launch.py](../src/LIO-SAM/launch/transform.launch.py)
   are up. They publish `base_link -> os_sensor` at `(0.145, 0, 0.31)` and
   `base_link -> vectornav` at `(0.08, 0, 0)`:

   ```bash
   ros2 run tf2_tools view_frames
   ```

   Both edges should appear in `frames.pdf`.

4. Confirm IMU samples are flowing:

   ```bash
   ros2 topic hz /vectornav/imu
   ```

   If silent, jump to
   [VectorNav driver: Unable to connect to device /dev/imu](#vectornav-driver-unable-to-connect-to-device-devimu).

Note: `lidarMinRange` in
[../src/LIO-SAM/config/params.yaml](../src/LIO-SAM/config/params.yaml)
is `0.6 m` (down from 1.0) so the robot body does not get clipped.

---

## GUI does not appear after boot

**Symptom.** The Jetson reaches the desktop but the Tk fullscreen GUI from
`meerkat_autostart` never shows up.

**Cause.** The autostart chain is GDM auto-login -> XDG autostart `.desktop`
-> `start_robot.sh` -> `ros2 launch meerkat_autostart bringup.launch.py` ->
`mode_controller`. Any broken link kills the GUI.

**Fix.** Walk the chain in order.

1. Check the autostart log - `start_robot.sh` writes everything here. Stack
   traces and missing packages will be obvious:

   ```bash
   tail -n 200 ~/.ros/meerkat-autostart.log
   ```

2. Confirm `DISPLAY=:0` is exported when the launch runs. Tk needs an X
   display; without it the window silently fails to map. Check the
   `Exec=` line in the `.desktop` file and the head of `start_robot.sh`.

3. Confirm the `.desktop` file is in `~/.config/autostart/`. If missing,
   copy it in:

   ```bash
   ls ~/.config/autostart/meerkat-autostart.desktop
   cp src/meerkat_autostart/systemd/meerkat-autostart.desktop \
      ~/.config/autostart/
   ```

4. Confirm GDM auto-login. `/etc/gdm3/custom.conf` `[daemon]` section must
   contain:

   ```
   AutomaticLoginEnable=true
   AutomaticLogin=teal
   ```

   Without it, no graphical session opens and the autostart entry never
   fires.

5. Confirm `python3-tk` is installed - the GUI will not import without it:

   ```bash
   dpkg -l python3-tk
   ```

---

## Save Map service unavailable

**Symptom.** R1+SHARE (or `savemap.py` pressing `D`) reports
`/lio_sam/save_map` is unavailable and no map is written under
`/media/teal/ssd1tb/lio_sam_maps/`.

**Cause.** `/lio_sam/save_map` is advertised only by LIO-SAM's
`mapOptimization` node, which only runs in **Mapping** mode. In Teleop or
Autonomous Nav mode, `mode_controller` has stopped the LIO-SAM subprocess and
the service is gone.

**Fix.**

1. Enter Mapping mode (R1+X, or the Mapping button on the Tk GUI).
2. Wait until `mapOptimization` is up - `/lio_sam/save_map` must appear
   before you press Save:

   ```bash
   ros2 node list    | grep lio_sam
   ros2 service list | grep save_map
   ```

3. Trigger the save. Output goes to
   `/media/teal/ssd1tb/lio_sam_maps/<DDMMYY>/verN/` at resolution 0.005. If
   the service exists but no file appears, confirm the SSD is mounted (it is
   in `/etc/fstab` with `nofail`):

   ```bash
   mount | grep ssd1tb
   ```

---

## ZED camera not connected

**Symptom.** [../zed_capture.sh](../zed_capture.sh) (or the ZED launch invoked
from `bringup.launch.py`) reports no ZED is connected, or the
`zed_capture_node.py` preview window never opens.

**Cause / Fix.** Three things to check, in order:

| Check | What to do |
|---|---|
| USB 3.0 port | ZED 2i needs full USB 3 bandwidth. Use a blue USB 3 port. `lsusb -t` should show the ZED at 5000M. |
| `zed_ros` conda env | `zed_capture.sh` activates `zed_ros` via `source ~/miniconda3/etc/profile.d/conda.sh && conda activate zed_ros && export PYTHONNOUSERSITE=1`. If the env is missing or broken the SDK Python bindings will not import. Verify with `conda env list`. |
| Cable seating | If ZED was working and suddenly is not, unplug and replug. The ZED is sensitive to flaky USB and sometimes only re-enumerates after a clean disconnect. |

After fixing, re-run `./zed_capture.sh`. Expect `zed_camera.launch.py` log
lines (`zed2i`, HD720@60fps) followed by the OpenCV preview from
`zed_capture_node.py`.

To drive the capture node remotely instead of via DS4 triggers, publish to
`/zed_capture/command` (`std_msgs/String`) - values: `image`, `auto_on`,
`auto_off`, `record_start`, `record_stop`, `interval=<sec>`.

---

## Robot drifts in autonomy

**Symptom.** The robot navigates but path-following is sloppy: it overshoots
goals, wanders off the planned route, or oscillates around the centreline.

**Cause.** Nav2 tuning. Not in scope for this guide - if mapping looks clean
and the TF tree resolves end-to-end, perception and odometry
(`meerkat_odometry`) are fine.

**Fix.** Tune Nav2. Start from
[../src/meerkat_navigation/config/nav2_params.yaml](../src/meerkat_navigation/config/nav2_params.yaml).
Controller frequency, inflation radius, and local-costmap update rate are the
usual suspects.
