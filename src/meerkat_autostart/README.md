# meerkat_autostart

GUI mode controller, DS4 shortcut layer, subprocess manager, bringup launch, and
boot-time autostart wiring for the Meerkat robot.

See also: [../../README.md](../../README.md) (workspace overview),
[../meerkat_motors/README.md](../meerkat_motors/README.md) (teleop bringup that
this package launches), and
[../meerkat_navigation/README.md](../meerkat_navigation/README.md) (Nav2
bringup that this package starts in autonomy mode).

## 1. Purpose

`meerkat_autostart` is the single entry point used at robot boot. It glues
together:

- A fullscreen **Tk GUI** with three operating modes (Teleop / Mapping /
  Autonomous Navigation), a Save-Map button, and ZED capture controls.
- **DS4 shortcuts** (R1 + face button) that drive the same state machine
  without the GUI.
- A **subprocess manager** that owns the LIO-SAM mapping process and the Nav2
  bringup process, switching between them when the mode changes.
- A **bringup launch** that brings up teleop, the ZED capture pipeline, and
  this controller in the right order.
- An **XDG autostart** `.desktop` entry plus a wrapper script so all of the
  above starts automatically on user login.

## 2. Node

A single `ament_python` console script is exported from
[setup.py](setup.py):

```python
entry_points={
    'console_scripts': [
        'mode_controller = meerkat_autostart.mode_controller:main',
    ],
},
```

The node is named `mode_controller` and is implemented in
[meerkat_autostart/mode_controller.py](meerkat_autostart/mode_controller.py).
It spins `rclpy` in a background thread and runs the Tk mainloop on the main
thread.

GUI keys: `F11` / `Esc` toggle fullscreen, `M` minimises (so you can see the
RViz map underneath), `Q` quits the whole bringup.

## 3. Topics & services

| Direction | Name | Type | Notes |
| --- | --- | --- | --- |
| Subscribes | `/status` | `ds4_driver_msgs/Status` | DS4 button state for the R1 shortcut layer. |
| Publishes | `/zed_capture/command` | `std_msgs/String` | Commands consumed by `zed_capture_node.py`: `image`, `auto_on`, `auto_off`, `record_start`, `record_stop`, `interval=<sec>`. |
| Client of | `/lio_sam/save_map` | `lio_sam/srv/SaveMap` | Called by Save Map. Versioned destination, see below. |

Save-Map versioning is ported from the standalone `savemap.py`: maps land in
`<map_base_dir>/<DDMMYY>/verN/`, where `N` is the next free integer in today's
date directory. The resolution sent in the request defaults to `0.005` m.

## 4. Subprocesses it owns

| Mode | Command | How it is stopped |
| --- | --- | --- |
| `mapping` | `bash <liosam_script>` (default `/home/teal/Downloads/lio_ws/liosam.sh`) | `SIGINT` to the process group, escalates to `SIGTERM` then `SIGKILL` after timeout. |
| `autonomy` | `ros2 launch <nav_package> <nav_launch>` (default `meerkat_navigation meerkat_bringup.launch.py`) | Same group-signal sequence. |
| `teleop` | (no child) | Teleop is already running from the bringup launch. |

Each child is started with `subprocess.Popen(..., preexec_fn=os.setsid)` so it
gets its own process group; on stop the controller calls `os.killpg(pgid,
SIGINT)` and waits, escalating to `SIGTERM` then `SIGKILL` if the child does
not exit. Switching modes always stops the previous child before starting the
new one, so LIO-SAM and Nav2 never run at the same time.

When the GUI `Quit` button (or `Q`) is pressed, the controller stops both
managed children and then sends `SIGINT` to its parent (`ros2 launch`) so the
whole bringup tears down cleanly.

## 5. Parameters

Every parameter is declared via `declare_parameter` in
[meerkat_autostart/mode_controller.py](meerkat_autostart/mode_controller.py).
Defaults match what the bringup launch passes in.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `liosam_script` | `/home/teal/Downloads/lio_ws/liosam.sh` | Script run in mapping mode. |
| `nav_package` | `meerkat_navigation` | Package for the autonomy launch. |
| `nav_launch` | `meerkat_bringup.launch.py` | Launch file for autonomy. |
| `map_base_dir` | `/media/teal/ssd1tb/lio_sam_maps` | Root of the versioned map output tree. |
| `map_resolution` | `0.005` | Resolution sent in the `SaveMap` request. |
| `status_topic` | `/status` | DS4 status topic. |
| `save_service` | `/lio_sam/save_map` | LIO-SAM save service name. |
| `zed_cmd_topic` | `/zed_capture/command` | Where GUI / DS4 publish ZED commands. |
| `modifier_button` | `button_r1` | Held button required for any DS4 shortcut. |
| `btn_mapping` | `button_cross` | R1 + X -> Mapping. |
| `btn_teleop` | `button_circle` | R1 + O -> Teleop. |
| `btn_autonomy` | `button_triangle` | R1 + Triangle -> Autonomy. |
| `btn_save` | `button_share` | R1 + Share -> Save Map. |
| `btn_record` | `button_options` | R1 + Options -> Toggle ZED record. |

Override at launch time the usual ROS 2 way:

```bash
ros2 launch meerkat_autostart bringup.launch.py
# or override an individual parameter when running the node directly:
ros2 run meerkat_autostart mode_controller \
    --ros-args -p map_resolution:=0.01 -p btn_save:=button_ps
```

## 6. Launch file

[launch/bringup.launch.py](launch/bringup.launch.py) wires up three things:

1. **Teleop** — `IncludeLaunchDescription` of
   `meerkat_motors/launch/meerkat_teleop_launch.py` (motors driver + DS4 ->
   `/meerkat/cmd_vel` bridge). Started immediately.
2. **ZED capture** — `ExecuteProcess` running
   `/home/teal/Downloads/lio_ws/zed_capture.sh`, wrapped in a `TimerAction`
   with a **3 s** stagger so the conda env activation and ZED wrapper start
   after teleop is up. `sigterm_timeout=5`, `sigkill_timeout=3`.
3. **Controller** — `Node` running `mode_controller`, wrapped in a
   `TimerAction` with a **2 s** stagger and prefilled with the parameters
   listed in section 5.

```python
return LaunchDescription([
    teleop,
    TimerAction(period=3.0, actions=[zed]),
    TimerAction(period=2.0, actions=[controller]),
])
```

## 7. Wrapper script

[scripts/start_robot.sh](scripts/start_robot.sh) is the entry point used by
the autostart `.desktop`. It:

1. Creates `~/.ros/` if needed and redirects all stdout/stderr to
   `~/.ros/meerkat-autostart.log`.
2. Sources `/opt/ros/humble/setup.bash` and
   `/home/teal/Downloads/lio_ws/install/setup.bash`.
3. `cd`s into the workspace.
4. `exec ros2 launch meerkat_autostart bringup.launch.py`.

Tail the log to debug a boot:

```bash
tail -f ~/.ros/meerkat-autostart.log
```

## 8. Autostart entry

The XDG autostart unit lives at
[systemd/meerkat-autostart.desktop](systemd/meerkat-autostart.desktop). To
enable it on the robot user, copy it into the user's autostart directory:

```bash
mkdir -p ~/.config/autostart
cp /home/teal/Downloads/lio_ws/src/meerkat_autostart/systemd/meerkat-autostart.desktop \
   ~/.config/autostart/
```

It runs:

```
Exec=/bin/bash -lc "/home/teal/Downloads/lio_ws/src/meerkat_autostart/scripts/start_robot.sh"
```

For unattended boot, GDM is configured for auto-login in
`/etc/gdm3/custom.conf`:

```
[daemon]
AutomaticLoginEnable=true
AutomaticLogin=teal
```

The DS4 controller must already be paired & trusted via `bluetoothctl` so it
reconnects on boot.

## 9. Dependencies

From [package.xml](package.xml):

| Dependency | Why |
| --- | --- |
| `rclpy` | ROS 2 Python client. |
| `std_msgs` | `String` for `/zed_capture/command`. |
| `ds4_driver_msgs` | `Status` message subscribed for DS4 shortcuts. |
| `lio_sam` | `SaveMap.srv` definition for the save-map client. |
| `meerkat_motors` | Provides the teleop launch included by bringup. |
| `meerkat_navigation` | Provides the Nav2 launch run in autonomy mode. |
| `python3-tk` | Required for the Tk GUI. |
| `ros2launch` | `ros2 launch` CLI used to start the Nav2 child process. |

`ds4_driver_msgs` and `lio_sam` come from third-party packages that are
cloned into `src/` (see the workspace README). `meerkat_motors` and
`meerkat_navigation` are sibling packages in this repo.

## 10. Build & run

From the workspace root:

```bash
cd ~/Downloads/lio_ws
colcon build --symlink-install --packages-select meerkat_autostart
source install/setup.bash
ros2 launch meerkat_autostart bringup.launch.py
```

To install boot autostart on a fresh machine:

```bash
# 1. Enable GDM auto-login (edit /etc/gdm3/custom.conf, [daemon] section).
# 2. Pair the DS4 over Bluetooth via bluetoothctl, mark it trusted.
# 3. Copy the autostart entry:
mkdir -p ~/.config/autostart
cp ~/Downloads/lio_ws/src/meerkat_autostart/systemd/meerkat-autostart.desktop \
   ~/.config/autostart/
# 4. Reboot. The Tk GUI should come up fullscreen.
```

Smoke test the DS4 layer without rebooting: launch the bringup, then with the
controller connected hold `R1` and tap `X` / `O` / `Triangle` — the radio
button in the GUI should follow your selection and you should see
`mode -> mapping` etc. in the controller log.
