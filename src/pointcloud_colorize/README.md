# pointcloud_colorize

Colorize the Ouster LiDAR cloud by sampling pixel colors from the ZED 2i
left rectified RGB image. Produces an RGB-encoded
`sensor_msgs/PointCloud2` for visualization and downstream fusion. See
the workspace overview in [../../README.md](../../README.md).

---

## 1. Purpose

`pointcloud_colorize` takes the Ouster LiDAR scan and the ZED 2i left
rectified color image, projects each LiDAR point into the camera, samples
the corresponding pixel, and republishes the cloud with a packed `rgb`
field. The result is a colored 3D cloud useful as a sanity check on
extrinsics and for richer scene visualization in RViz.

The ZED 2i sits on `base_link` at `(0.20, 0, 0.19)` and the Ouster at
`(0.145, 0, 0.31)` (see [meerkat_description](../meerkat_description)).

---

## 2. DISABLED by default

This package is **excluded from `colcon build`** by default. A
[COLCON_IGNORE](COLCON_IGNORE) marker in this directory tells `colcon`
to skip the package entirely.

To enable it:

```bash
# from the workspace root
rm src/pointcloud_colorize/COLCON_IGNORE
colcon build --packages-select pointcloud_colorize
source install/setup.bash
```

It ships disabled because the rest of the stack (LIO-SAM, Nav2, ZED
capture, autostart bringup) does not depend on a colorized cloud.

---

## 3. Node

There is one node: `colorize_node`, registered as a console entry point
in [setup.py](setup.py).

### Subscriptions

| Topic | Type | Notes |
|-------|------|-------|
| `/ouster/points` | `sensor_msgs/PointCloud2` | Raw LiDAR cloud from `ouster-ros` |
| `/zed/zed_node/left/image_rect_color` | `sensor_msgs/Image` | ZED left rectified color image |
| `/zed/zed_node/left/camera_info` | `sensor_msgs/CameraInfo` | ZED left intrinsics (K matrix) |

### Publications

| Topic | Type | Notes |
|-------|------|-------|
| `/colored_points` | `sensor_msgs/PointCloud2` | Same XYZ as input, with packed `rgb` field |

---

## 4. Algorithm summary

Vectorised numpy pipeline per cloud message:

1. Look up TF from the LiDAR frame (cloud `header.frame_id`) to the ZED
   left optical frame.
2. Transform the cloud into the camera optical frame in one matmul.
3. Project to the image plane using `K` from `/zed/zed_node/left/camera_info`.
4. Clip to points in front of the camera and inside image bounds.
5. Sample the pixel from the latest rectified image.
6. Pack `(r, g, b)` into a `float32` field and publish `/colored_points`.

Points outside the ZED frustum stay gray (or are dropped if
`drop_uncolored` is set — see Launch).

---

## 5. Launch

The package ships one launch file: [launch/colorize.launch.py](launch/colorize.launch.py).

```bash
ros2 launch pointcloud_colorize colorize.launch.py
```

### Configurable arguments

| Arg | Default | Meaning |
|-----|---------|---------|
| `cloud_topic` | `/ouster/points` | Input LiDAR cloud |
| `image_topic` | `/zed/zed_node/left/image_rect_color` | Input ZED color image |
| `camera_info_topic` | `/zed/zed_node/left/camera_info` | ZED intrinsics |
| `output_topic` | `/colored_points` | Colored cloud output |
| `drop_uncolored` | (see launch file) | Drop points outside the ZED FOV instead of coloring them gray |

Example overriding topics:

```bash
ros2 launch pointcloud_colorize colorize.launch.py \
    cloud_topic:=/ouster/points \
    output_topic:=/colored_points \
    drop_uncolored:=true
```

---

## 6. RViz visualization

In RViz:

1. Add a `PointCloud2` display.
2. Set **Topic** to `/colored_points`.
3. Set **Color Transformer** to `RGB8`.
4. Set **Channel Name** to `rgb`.

If the `RGB8` transformer or the `rgb` channel does not appear, the
incoming cloud has no color field — usually because the ZED node is not
running. See [Caveats](#7-caveats).

---

## 7. Caveats

- **Limited FOV.** Only points inside the ZED's image frustum get
  colored; the rest stay gray (or are dropped with
  `drop_uncolored:=true`). The Ouster covers 360°; the ZED covers a
  narrow forward cone, so most points will be uncolored.
- **TF chain required.** A valid TF from the LiDAR frame to the ZED left
  optical frame must exist, provided by [meerkat_description](../meerkat_description)
  static transforms plus the ZED wrapper.
- **ZED must be running.** No ZED image means no color, and RViz will
  not list `RGB8` as a Color Transformer.
- **Disabled by default.** Remove `COLCON_IGNORE` before building, or
  `colcon` will silently skip the package.

---

## 8. Build

After removing the ignore marker:

```bash
cd ~/Downloads/lio_ws
rm src/pointcloud_colorize/COLCON_IGNORE
colcon build --packages-select pointcloud_colorize
source install/setup.bash
```

To disable again, recreate an empty `COLCON_IGNORE` and rebuild.
