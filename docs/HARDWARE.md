# Hardware: Sensor Placement & Calibration

This document captures the physical geometry of the Teal Meerkat platform,
where those numbers live in the repo, and how to update them when a sensor
is moved or swapped.

See also: [../README.md](../README.md), [SETUP.md](SETUP.md).

---

## 1. Coordinate convention

All frames follow [ROS REP-103](https://www.ros.org/reps/rep-0103.html):

- Right-handed coordinate system.
- `x` forward, `y` left, `z` up.
- Roll about `x`, pitch about `y`, yaw about `z`.
- Units: metres and radians throughout.

`base_link` is the robot body frame. It sits **0.26 m above the ground**
(the ground plane is `odom` / `map` at `z = 0`). Every sensor frame is
defined as a static transform from `base_link`.

---

## 2. Sensor geometry table

All translations below are `base_link -> <sensor>` in metres, with
`rpy = (0, 0, 0)` (no rotation relative to `base_link`).

| Frame                  | x (m)  | y (m) | z (m) | roll | pitch | yaw | Notes                              |
|------------------------|--------|-------|-------|------|-------|-----|------------------------------------|
| `base_link` (vs ground)| 0.000  | 0.000 | 0.260 | 0    | 0     | 0   | Body frame, 0.26 m above floor     |
| `vectornav` (IMU)      | 0.080  | 0.000 | 0.000 | 0    | 0     | 0   | VectorNav, mounted on body         |
| `os_sensor` (LiDAR)    | 0.145  | 0.000 | 0.310 | 0    | 0     | 0   | Ouster, mounted forward and high   |
| `camera_link` (ZED 2i) | 0.200  | 0.000 | 0.190 | 0    | 0     | 0   | Stereolabs ZED 2i, forward         |

The LiDAR position **relative to the IMU** (used by LIO-SAM as
`extrinsicTrans`) follows directly:

```
extrinsicTrans = os_sensor - vectornav
               = (0.145 - 0.080, 0 - 0, 0.310 - 0)
               = [0.065, 0.000, 0.310]  # metres
```

---

## 3. Where these numbers live in the repo

These geometry constants are duplicated in two places: the URDF (used by
RViz, TF tree visualisation, Nav2 footprint reasoning) and the LIO-SAM
launch + params (used by SLAM). Both must agree.

### 3.1 URDF

File: [src/meerkat_description/src/description/meerkat_description.urdf](../src/meerkat_description/src/description/meerkat_description.urdf)

The relevant joints (link line numbers approximate; cite by joint name):

- `base_joint` — places `base_link` 0.26 m above the floor.
- `imu_joint` — `base_link -> vectornav` at `xyz = "0.08 0 0"`.
- `os_sensor_mount` — `base_link -> os_sensor` at `xyz = "0.145 0 0.31"`.
- `camera_joint` — `base_link -> camera_link` at `xyz = "0.20 0 0.19"`.

All four joints have `rpy = "0 0 0"`.

### 3.2 LIO-SAM static TFs

File: [src/LIO-SAM/launch/transform.launch.py](../src/LIO-SAM/launch/transform.launch.py)

This launch publishes the static TFs LIO-SAM consumes at runtime:

- `base_link -> os_sensor` at `(0.145, 0, 0.31)`.
- `base_link -> vectornav` at `(0.08, 0, 0)`.

These come from the geometry table above and must match the URDF.

### 3.3 LIO-SAM extrinsics

File: [src/LIO-SAM/config/params.yaml](../src/LIO-SAM/config/params.yaml)

Key entries:

- `extrinsicTrans: [0.065, 0.000, 0.310]` — **LiDAR position expressed in
  the IMU frame** (i.e. `os_sensor` minus `vectornav` in `base_link`).
- `extrinsicRot` — identity (see Section 7 below).
- `imuTopic: /vectornav/imu`
- LiDAR sensor: `ouster`, `N_SCAN: 128`, `Horizon_SCAN: 1024`.
- `lidarMinRange: 0.6` (see Section 5).
- Save service: `/lio_sam/save_map`.

---

## 4. Re-measuring and recalibrating

When you physically move or swap a sensor, update the numbers in the
following order, then rebuild.

### 4.1 Measure

1. Power down the robot.
2. Measure each translation from the `base_link` reference (the body
   frame origin sits 0.26 m above the ground, centred fore/aft on the
   chassis as defined by the URDF).
3. Record the new `(x, y, z)` for the moved sensor in metres.
4. If the sensor is rotated relative to `base_link`, also record
   `(roll, pitch, yaw)` in radians.

### 4.2 Update

For every change, update **both** of these files so the URDF and SLAM
agree:

- [src/meerkat_description/src/description/meerkat_description.urdf](../src/meerkat_description/src/description/meerkat_description.urdf)
  — edit the `xyz` (and `rpy` if non-zero) of the matching joint
  (`imu_joint`, `os_sensor_mount`, `camera_joint`, or `base_joint`).
- [src/LIO-SAM/launch/transform.launch.py](../src/LIO-SAM/launch/transform.launch.py)
  — edit the static TF arguments for `base_link -> os_sensor` and
  `base_link -> vectornav`.

If the LiDAR **or** the IMU moved, also update the LiDAR-in-IMU
translation:

- [src/LIO-SAM/config/params.yaml](../src/LIO-SAM/config/params.yaml)
  — recompute `extrinsicTrans = os_sensor - vectornav` (each component
  in `base_link`) and set the new `[x, y, z]`.

Finally, update Section 2 of this document so the table stays
authoritative.

### 4.3 Rebuild

```bash
cd ~/Downloads/lio_ws
colcon build --symlink-install
source install/setup.bash
```

If you changed the LIO-SAM patches, remember to re-apply them over the
upstream `LIO-SAM` clone before building (see
[SETUP.md](SETUP.md)).

---

## 5. `lidarMinRange = 0.6 m`

`lidarMinRange` in
[src/LIO-SAM/config/params.yaml](../src/LIO-SAM/config/params.yaml)
is set to **0.6 m** (down from the upstream default of 1.0 m).

Reason: the Ouster sits forward on the chassis at
`base_link -> os_sensor = (0.145, 0, 0.31)`. The robot body extends
behind and below it, so part of the chassis falls inside the
default 1.0 m exclusion ring. A 1.0 m cutoff was clipping useful returns
near the robot footprint, so the floor was lowered to 0.6 m to keep
chassis self-returns rejected while preserving close-in environment
points.

If you move the LiDAR (e.g. higher / further forward so the body no
longer sits within ~1 m), re-evaluate this value. If you change the
robot footprint substantially, tune it together with Nav2 costmap
inflation in [src/meerkat_navigation](../src/meerkat_navigation).

---

## 6. Visualising the URDF

To sanity-check that joint edits look right before rebuilding the whole
stack, use the standard `urdf_tutorial` viewer:

```bash
ros2 launch urdf_tutorial display.launch.py \
  model:=$HOME/Downloads/lio_ws/src/meerkat_description/src/description/meerkat_description.urdf
```

RViz tip: set **Fixed Frame** to `base_link` (or `odom` if you also want
to see the ground offset), then add the **RobotModel** and **TF**
displays. The TF display will show each sensor frame at its configured
offset; eyeball the arrows against the table in Section 2.

---

## 7. IMU axis convention expected by LIO-SAM

LIO-SAM uses `extrinsicRot` (in
[src/LIO-SAM/config/params.yaml](../src/LIO-SAM/config/params.yaml))
to rotate IMU samples into the LiDAR frame's axis convention. In this
repo `extrinsicRot` is the **identity matrix**, which means:

> The VectorNav IMU is mounted so that its `+x / +y / +z` axes already
> align with the Ouster `os_sensor` axes (and therefore with `base_link`
> — see Section 2, where both have `rpy = 0`).

Operational implications:

- If you **swap** the VectorNav for the same model, mounted the same
  way, you do not need to touch `extrinsicRot`.
- If you swap in a **different IMU** (e.g. different brand, or the
  VectorNav re-mounted on its side), the new sensor's body axes will
  almost certainly **not** match the LiDAR's axes. In that case:
  1. Determine the rotation `R` that maps the new IMU's body frame into
     the LiDAR frame.
  2. Update `extrinsicRot` (and `extrinsicRPY`, if the upstream params
     expose it) accordingly.
  3. Refer to the upstream
     [LIO-SAM README](https://github.com/TixiaoShan/LIO-SAM) for the
     matrix format and conventions — it documents how
     `extrinsicRot` / `extrinsicRPY` are applied to incoming IMU
     samples.
- Symptoms of an IMU axis mismatch include: SLAM diverging immediately
  on startup, gravity pointing the wrong way in initialisation, or yaw
  drifting in the opposite direction from real motion.

---

## 8. Summary checklist when a sensor moves

1. Measure new `(x, y, z)` (and `rpy` if rotated) from `base_link`.
2. Update the corresponding joint in
   [meerkat_description.urdf](../src/meerkat_description/src/description/meerkat_description.urdf).
3. Update the static TF in
   [transform.launch.py](../src/LIO-SAM/launch/transform.launch.py).
4. If the LiDAR or IMU moved, recompute `extrinsicTrans` and update
   [params.yaml](../src/LIO-SAM/config/params.yaml).
5. If a non-identical IMU was installed, update `extrinsicRot`.
6. Update the table in Section 2 of this file.
7. `colcon build --symlink-install` and re-source.
8. Sanity-check with `urdf_tutorial display.launch.py` and, on the live
   robot, `ros2 run tf2_tools view_frames` to confirm the TF tree.
