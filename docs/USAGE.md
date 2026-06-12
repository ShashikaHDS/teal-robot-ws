# Operator Manual

This is the day-to-day operator guide for the Meerkat / Teal robot running the
[teal-robot-ws](https://github.com/ShashikaHDS/teal-robot-ws) stack. For one-time
setup (deps, udev, GDM, ZED conda env), see [SETUP.md](SETUP.md). For an overview
of the codebase, see [../README.md](../README.md).

---

## 1. Boot behaviour

The Jetson is configured to log in `teal` automatically (GDM `AutomaticLogin`)
and then run an XDG autostart entry. That entry launches the wrapper
[`src/meerkat_autostart/scripts/start_robot.sh`](../src/meerkat_autostart/scripts/start_robot.sh),
which sources ROS 2 and runs:

```bash
ros2 launch meerkat_autostart bringup.launch.py
```

All output is appended to `~/.ros/meerkat-autostart.log` — that's the first place
to look if something doesn't come up.

### What you see on screen

A fullscreen Tk GUI from `mode_controller` (the
[meerkat_autostart](../src/meerkat_autostart/) package). It shows:

- The currently-selected operating mode (radio buttons).
- A **Save Map** button.
- A **ZED capture** panel.
- A status bar at the bottom.
- A **Quit** button.

### What's running underneath the GUI

`bringup.launch.py` brings up three things in parallel:

1. **Teleop stack** — included from
   [`meerkat_motors/meerkat_teleop_launch.py`](../src/meerkat_motors/launch/meerkat_teleop_launch.py).
   This starts the `ds4_driver`, the motor drivers on `/dev/Motor_L` and
   `/dev/Motor_R`, and the DS4-to-`cmd_vel` bridge. It publishes
   `/meerkat/cmd_vel`, `/odom`, `/ds4/trigger`, `/ds4/trigger2`.
2. **ZED capture** — `zed_capture.sh` is launched as an `ExecuteProcess` so it
   keeps its `zed_ros` conda environment. It brings up `zed_wrapper`
   (`camera_model:=zed2i`, HD720 @ 60 fps) and `zed_capture_node.py`.
3. **mode_controller** — the GUI itself, which also owns the LIO-SAM and Nav2
   subprocesses (started/stopped on mode change).

The robot boots into **Teleop** mode. Nothing is mapping or navigating until you
ask for it.

---

## 2. Operating modes

The mode_controller exposes three radio-button modes. Switching modes cleanly
stops the previous mode's subprocesses before starting the new one.

| Mode | What it starts | Safe to do |
| --- | --- | --- |
| **Teleop** | Just the teleop stack (already up from boot). LIO-SAM and Nav2 are stopped. | Drive manually with the DS4. Nothing is recording the map. Safe default. |
| **Mapping** | Spawns a LIO-SAM subprocess (same launches as [`liosam.sh`](../liosam.sh) — `transform.launch.py` + `run.launch.py`). | Drive slowly and smoothly to build a map. Use **Save Map** at key moments. Do not jerk the robot. |
| **Autonomous Nav** | Spawns `ros2 launch meerkat_navigation meerkat_bringup.launch.py`. | Send Nav2 goals (RViz, scripted, etc.). Keep the L1 dead-man handy in case you need to override. |

You should always **return to Teleop** before powering off so the mapping /
navigation subprocesses are torn down cleanly.

---

## 3. DS4 (DualShock 4) shortcuts

The DS4 is paired over Bluetooth and reconnects automatically on boot (see
[SETUP.md](SETUP.md) if it doesn't).

### Dead-man (driving)

- **L1** is the dead-man trigger for driving. Hold L1 and use the sticks to
  move. Release L1 and the robot stops. **L1 is unrelated to R1 below.**

### R1 modifier shortcuts (GUI control)

Hold **R1** and tap one of the face / centre buttons to trigger an action in
the GUI. This is independent of the L1 dead-man.

| Combo | Action |
| --- | --- |
| **R1 + ✕ (Cross)** | Switch to **Mapping** mode |
| **R1 + ◯ (Circle)** | Switch to **Teleop** mode |
| **R1 + △ (Triangle)** | Switch to **Autonomous Nav** mode |
| **R1 + SHARE** | **Save Map** (calls `/lio_sam/save_map`) |
| **R1 + OPTIONS** | Toggle **ZED recording** on/off |

Tap-style — release R1 after the combo. You do not need to be holding L1 at the
same time; these are GUI shortcuts, not motion commands.

---

## 4. GUI elements

The Tk GUI exposes everything you normally need without having to drop to a
terminal.

- **Mode radio** — Teleop / Mapping / Autonomous Nav. Selecting one stops the
  previous mode's processes and starts the new one.
- **Save Map** — calls the `/lio_sam/save_map` service with
  `resolution=0.005` and a versioned destination directory. Only meaningful
  while Mapping mode is running (LIO-SAM must be alive to serve the call).
- **ZED capture panel:**
  - **Capture Image** — publish `image` on `/zed_capture/command` to grab a
    single RGB + depth pair.
  - **Auto-capture toggle** — publish `auto_on` / `auto_off` to start/stop
    periodic captures.
  - **Interval entry** — sets the auto-capture period; publish `interval=<sec>`
    to push the new value.
  - **Recording toggle** — publish `record_start` / `record_stop` to write
    `.svo` / video to disk. Mirrors the **R1 + OPTIONS** shortcut.
- **Status bar** — bottom of the GUI; shows current mode, last action, and
  errors (e.g. "save_map service unavailable").
- **Quit** — tears down LIO-SAM / Nav2 subprocesses (if running) and exits
  the entire bringup. Use this before powering off.

---

## 5. GUI keyboard shortcuts

| Key | Action |
| --- | --- |
| **F11** | Toggle fullscreen |
| **Esc** | Toggle fullscreen (exit fullscreen) |
| **M** | Minimise the GUI so you can see RViz / the map view behind it |
| **Q** | Quit the entire bringup (same as the **Quit** button) |

`M` is the one you'll use most during a mapping run — minimise to watch the
LIO-SAM map grow in RViz, hit **R1 + SHARE** to save, then restore the GUI.

---

## 6. Map saving

Saving a map is driven by `mode_controller`, which ports the versioning logic
from the standalone [`savemap.py`](../savemap.py).

### Where files land

```
/media/teal/ssd1tb/lio_sam_maps/<DDMMYY>/verN/
```

- `<DDMMYY>` is today's date (e.g. `120626` for 2026-06-12).
- `verN` increments per save within the same day — `ver1`, `ver2`, `ver3`, ...
- Each directory contains the standard LIO-SAM `save_map` output (`.pcd` files
  for the global map and trajectory, plus any auxiliary files the service
  writes).

### How to save

1. Make sure you're in **Mapping** mode and LIO-SAM is alive.
2. Click **Save Map** in the GUI, or press **R1 + SHARE** on the DS4.
3. Watch the status bar — it confirms the destination directory on success.

The save is a service call to `/lio_sam/save_map` with
`SaveMap.Request(resolution=0.005, destination=<versioned dir>)`.

### Loading a saved map later

The `.pcd` outputs in `verN/` are standard LIO-SAM artefacts. Feed them back to
LIO-SAM via its load-map parameters (see the LIO-SAM upstream docs and the
patched `params.yaml` in [`src/LIO-SAM/config/params.yaml`](../src/LIO-SAM/config/params.yaml)).
Octomap / Nav2 consumption goes via the existing
`octomap.launch.py` patch ([`src/LIO-SAM/launch/octomap.launch.py`](../src/LIO-SAM/launch/octomap.launch.py))
which sets `occupancy_min_z=-0.52` / `occupancy_max_z=0.33` for the current
`base_link` height of 0.26 m.

---

## 7. ZED capture details

The ZED 2i is brought up at boot (HD720 @ 60 fps) by `zed_capture.sh`. The
`zed_capture_node.py` subscribes:

- `/zed/zed_node/rgb/image_rect_color`
- `/zed/zed_node/depth/depth_registered`
- `/ds4/trigger`
- `/zed_capture/command` (added for GUI control)

### Three capture modes

| Mode | What it does |
| --- | --- |
| **Image** | One-shot capture of the current RGB + depth pair. Written as image files. |
| **Auto-capture** | Same as Image, repeated every `interval` seconds until toggled off. |
| **Recording** | Records continuous video from the ZED (SVO/video) until toggled off. |

These are independent — auto-capture and recording can be on at the same time.

### `/zed_capture/command` vocabulary

Publish `std_msgs/String` on `/zed_capture/command`:

| Command | Effect |
| --- | --- |
| `image` | Capture one image pair now |
| `auto_on` | Start auto-capture |
| `auto_off` | Stop auto-capture |
| `record_start` | Start video recording |
| `record_stop` | Stop video recording |
| `interval=<sec>` | Set auto-capture interval, e.g. `interval=2.5` |

From a shell:

```bash
ros2 topic pub --once /zed_capture/command std_msgs/String "data: 'image'"
ros2 topic pub --once /zed_capture/command std_msgs/String "data: 'auto_on'"
ros2 topic pub --once /zed_capture/command std_msgs/String "data: 'interval=5'"
ros2 topic pub --once /zed_capture/command std_msgs/String "data: 'record_start'"
```

The preview window from `zed_capture_node.py` also accepts keys directly:
**X** capture, **A** toggle auto, **R** toggle record, **Q** quit.

### Where files land

- Images: `/media/teal/ssd1tb/zed2_images/...`
- Videos: `/media/teal/ssd1tb/zed2_videos/...`

Both live on the SSD mounted at `/media/teal/ssd1tb` (see `/etc/fstab`, `nofail`
so a missing SSD won't block boot).

---

## 8. Manual workflow (4-terminal fallback)

If the GUI fails to come up (e.g. `python3-tk` missing, `mode_controller`
crashed), the original 4-terminal flow still works. Each block is a separate
terminal.

**Terminal 1 — teleop and motors:**

```bash
cd ~/Downloads/lio_ws
source install/setup.bash
ros2 launch meerkat_motors meerkat_teleop_launch.py
```

**Terminal 2 — LIO-SAM mapping:**

```bash
cd ~/Downloads/lio_ws
./liosam.sh
```

`liosam.sh` sources ROS, launches `lio_sam transform.launch.py` and
`run.launch.py`, and traps `SIGINT` so a single `Ctrl-C` cleans everything up.

**Terminal 3 — map saver:**

```bash
cd ~/Downloads/lio_ws
python3 savemap.py
```

This polls the keyboard; press **D** to save. The destination follows the same
`/media/teal/ssd1tb/lio_sam_maps/<DDMMYY>/verN/` versioning the GUI uses.

**Terminal 4 — ZED capture:**

```bash
cd ~/Downloads/lio_ws
./zed_capture.sh
```

This activates the `zed_ros` conda env, sources ROS, launches the ZED wrapper,
and starts `zed_capture_node.py`. Use the preview window keys (**X / A / R /
Q**) or publish to `/zed_capture/command` from another shell.

When you're done, **Ctrl-C** each terminal in reverse order (ZED, savemap,
LIO-SAM, teleop).

---

## 9. Recommended mapping routine

A typical mapping session, end-to-end:

1. **Power on** — wait for the GUI to come up. It boots into **Teleop**.
2. **Pair check** — hold **L1** and nudge the left stick. Confirm the robot
   responds. If not, see the DS4 / Bluetooth section in [SETUP.md](SETUP.md).
3. **Position the robot** at the start of your map (clear area, away from
   walls). Stay in Teleop for this.
4. **Switch to Mapping** — **R1 + ✕** on the DS4, or click the Mapping radio
   button. Wait a couple of seconds for LIO-SAM to come up; watch the status
   bar.
5. **Drive slowly and smoothly** with **L1** held and the sticks. Avoid sharp
   yaw spins — LIO-SAM tracks better at modest angular rates. If you want to
   watch the map grow, hit **M** to minimise the GUI behind RViz.
6. **Save at key moments** — **R1 + SHARE** (or the Save Map button) after each
   meaningful section (e.g. each room, each corridor). You get a fresh `verN/`
   directory each time, so you can roll back.
7. **Optionally capture ZED images** along the route — Capture Image for
   one-offs, or toggle auto-capture with a 2-5 second interval.
8. **Finish** — drive back near the start, hit **Save Map** one last time
   (`verN/` final), then **switch back to Teleop** (**R1 + ◯**) so LIO-SAM is
   torn down cleanly.
9. **Quit** — press **Q** in the GUI (or click **Quit**) before shutting the
   Jetson down. This stops every subprocess the bringup owns.

If a mapping run goes badly (drift, jumps), don't fight it — switch to Teleop
to kill LIO-SAM, reposition, then back to Mapping to start fresh. You'll get a
new `verN/` directory on the next save, the old ones are untouched.

---

## See also

- [../README.md](../README.md) — repo overview, packages, third-party deps.
- [SETUP.md](SETUP.md) — one-time install, udev, GDM, conda env.
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — ZED `gxf_isaac` errors, missing
  `/dev/imu`, DS4 not reconnecting, etc.
- [`src/LIO-SAM/`](../src/LIO-SAM/) — the LIO-SAM patches that match
  this robot's geometry.
