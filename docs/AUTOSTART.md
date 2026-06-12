# Boot-Time Autostart

This document describes how the Meerkat robot brings itself up automatically at
boot: from power-on, through GDM auto-login, to the Tk GUI and the full ROS 2
stack (LIO-SAM mapping / Nav2 autonomy / ZED capture, all selectable from the
DualShock 4).

See also: [../README.md](../README.md), [SETUP.md](SETUP.md),
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## 1. Architecture

The autostart path is intentionally simple and uses standard Ubuntu/GNOME
machinery rather than a bespoke systemd unit:

```
power on
  -> GDM (gnome-display-manager)
       AutomaticLogin=teal           (in /etc/gdm3/custom.conf)
  -> GNOME session for user `teal`
  -> XDG autostart scans ~/.config/autostart/
       meerkat-autostart.desktop -> Exec=start_robot.sh
  -> start_robot.sh
       source /opt/ros/humble/setup.bash
       source ~/Downloads/lio_ws/install/setup.bash
       ros2 launch meerkat_autostart bringup.launch.py
         -> meerkat_motors  (teleop + DS4 bridge)
         -> zed_capture.sh  (ExecuteProcess, conda env zed_ros)
         -> mode_controller (Tk GUI, LIO-SAM/Nav2 supervisor)
```

The wrapper script is [../src/meerkat_autostart/scripts/start_robot.sh](../src/meerkat_autostart/scripts/start_robot.sh)
and the desktop entry lives at
[../src/meerkat_autostart/systemd/meerkat-autostart.desktop](../src/meerkat_autostart/systemd/meerkat-autostart.desktop).
Output is captured to `~/.ros/meerkat-autostart.log`.

## 2. Why XDG autostart, not `systemd --user`

The bringup is not purely headless. It contains:

- a Tk `Toplevel` GUI (fullscreen mode controller) — needs `DISPLAY`,
- the ZED preview window from `zed_capture_node.py` (`cv2.imshow`) — also needs
  `DISPLAY` and a live X session,
- subprocess children (LIO-SAM, Nav2, ZED) that inherit that same environment.

| Approach | Gets `DISPLAY=:0`? | Survives logout? | Notes |
| --- | --- | --- | --- |
| XDG autostart `.desktop` in `~/.config/autostart/` | yes, runs inside the GNOME session | no (dies with session — fine, GDM auto-login keeps a session up) | what we use |
| `systemd --user` unit | no (no `DISPLAY` by default) | yes, with `loginctl enable-linger` | needs extra wiring for X; not worth it for a GUI app |
| `systemd` system unit | no | yes | wrong privilege boundary, breaks GUI completely |

GDM auto-login guarantees there is always a logged-in GNOME session for user
`teal`, so the XDG entry fires on every boot without anyone touching a
keyboard.

## 3. Install Steps

Perform these once per robot, as user `teal`.

### 3.1 Install the Tk runtime

The mode controller GUI is Tkinter-based.

```bash
sudo apt update
sudo apt install -y python3-tk
```

### 3.2 Drop the autostart entry into the user's autostart dir

```bash
mkdir -p ~/.config/autostart
cp ~/Downloads/lio_ws/src/meerkat_autostart/systemd/meerkat-autostart.desktop \
   ~/.config/autostart/
```

### 3.3 Make the wrapper executable

```bash
chmod +x ~/Downloads/lio_ws/src/meerkat_autostart/scripts/start_robot.sh
```

### 3.4 Enable GDM auto-login

Edit `/etc/gdm3/custom.conf` and ensure the `[daemon]` section contains:

```ini
[daemon]
AutomaticLoginEnable=true
AutomaticLogin=teal
```

Save and exit. No daemon reload is required — GDM reads this on the next boot.

## 4. Reboot Test

After a clean `sudo reboot`, you should see:

1. GDM briefly, then the GNOME desktop logging in as `teal` with no password
   prompt.
2. A fullscreen Tk window (the mode controller) within a few seconds.
3. The ZED preview window (`cv2.imshow`).

From a terminal (open one with the GNOME hotkey or SSH in):

```bash
# wrapper running?
pgrep -af start_robot.sh

# bringup launched?
pgrep -af "ros2 launch meerkat_autostart"

# ROS graph populated?
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list | grep -E "cmd_vel|ds4|zed"
```

Expected nodes include `mode_controller`, the `meerkat_motors` driver and DS4
bridge, and the ZED wrapper. Expected topics include `/meerkat/cmd_vel`,
`/odom`, `/ds4/trigger`, `/ds4/trigger2`, and `/zed_capture/command`.

Press R1 + Triangle on the DualShock 4 to launch autonomy, R1 + Cross for
mapping, R1 + Circle for teleop, R1 + Share to save the current LIO-SAM map,
and R1 + Options to toggle ZED recording.

## 5. Disabling Temporarily

Two equivalent ways — pick whichever is more convenient.

### 5.1 GNOME GUI

`Settings -> Applications -> Startup` (or `gnome-tweaks -> Startup
Applications` on older builds), then untick **Meerkat Autostart**. The entry
remains on disk and can be re-ticked later.

### 5.2 Rename the .desktop file

```bash
mv ~/.config/autostart/meerkat-autostart.desktop{,.disabled}
```

To re-enable:

```bash
mv ~/.config/autostart/meerkat-autostart.desktop{.disabled,}
```

Either way, GDM auto-login keeps working; only the bringup is skipped. To
also disable auto-login, set `AutomaticLoginEnable=false` in
`/etc/gdm3/custom.conf`.

## 6. Logs

`start_robot.sh` redirects stdout and stderr of the bringup to a single log
file under `~/.ros/`:

```bash
tail -f ~/.ros/meerkat-autostart.log
```

This captures the full `ros2 launch` output, including child processes
(motor driver, DS4 bridge, ZED wrapper, LIO-SAM, Nav2). It is the first place
to look if the GUI does not appear after a reboot.

Common quick checks against this log:

| Symptom | Search for | Likely cause |
| --- | --- | --- |
| GUI never appears | `no display name` / `TclError` | `DISPLAY` not set — wrong launch path (systemd?) |
| ZED fails to load | `libgxf_isaac_optimizer.so` | missing `LD_LIBRARY_PATH` — see [TROUBLESHOOTING.md](TROUBLESHOOTING.md) |
| Motors don't move | `/dev/Motor_L` / `/dev/Motor_R` errors | udev rule / cable swap — see [TROUBLESHOOTING.md](TROUBLESHOOTING.md) |
| DS4 inactive | `ds4_driver` reconnect loops | Bluetooth pairing — re-pair and trust in `bluetoothctl` |

## 7. Alternative: `systemd --user` (headless robots only)

If a future variant of this robot truly has no display attached (no monitor,
no ZED preview, no Tk GUI), the bringup can be moved under `systemd --user`:

```bash
loginctl enable-linger teal     # keep user manager alive without a login
systemctl --user daemon-reload
systemctl --user enable meerkat-autostart.service
```

This approach is **not recommended for the current robot** because:

- `mode_controller` opens a Tk `Toplevel` — without `DISPLAY` it raises
  `TclError` and the whole bringup exits.
- `zed_capture_node.py` calls `cv2.imshow`, which similarly requires an X
  server.
- Even when adding `Environment=DISPLAY=:0` to the unit, you still need an X
  server running, which on Ubuntu means a GNOME session, which brings you
  back to GDM auto-login + XDG autostart.

Use `systemd --user` only after stripping the GUI dependencies (Tk + cv2
preview) from `meerkat_autostart` and `zed_capture_node.py`. Until then,
the `.desktop` + GDM path documented above is the supported one.
