# LIO-SAM patches

Drop-in files preserving the robot-specific tweaks made to upstream
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM). After cloning LIO-SAM into
`src/LIO-SAM/`, copy these over the upstream files:

```bash
cp patches/LIO-SAM/config/params.yaml         src/LIO-SAM/config/
cp patches/LIO-SAM/launch/transform.launch.py src/LIO-SAM/launch/
cp patches/LIO-SAM/launch/octomap.launch.py   src/LIO-SAM/launch/
```

## What changed

- **`config/params.yaml`** — `extrinsicTrans` updated to match the current
  IMU/LiDAR placement: `[0.065, 0, 0.310]` m (LiDAR − IMU in `base_link`).
- **`launch/transform.launch.py`** — static TFs `base_link → os_sensor` set to
  `(0.145, 0, 0.31)` and `base_link → vectornav` set to `(0.08, 0, 0)`.
- **`launch/octomap.launch.py`** — comments and `occupancy_min_z`/
  `occupancy_max_z` updated for `base_link` at `0.26 m` above ground and
  `base_link → os_lidar` at `0.31 m`.
