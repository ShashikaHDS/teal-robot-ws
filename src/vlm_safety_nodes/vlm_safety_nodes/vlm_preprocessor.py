# #!/usr/bin/env python3
# import math
# import time
# import hashlib
# from typing import Dict, Optional, Tuple

# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# from rclpy.duration import Duration

# from std_msgs.msg import Float32
# from geometry_msgs.msg import Point
# from sensor_msgs.msg import CompressedImage
# from tf2_ros import Buffer, TransformListener

# # -------------------------

# # -------------------------
# # Example:
# # from vlm_safety_msgs.msg import HazardObjectArray, VLMQuery
# from vlm_safety_msgs.msg import HazardObjectArray, VLMQuery


# # -------------------------
# # Class config
# # -------------------------
# CONSIDERED = {"vehicle", "machinery", "person", "human", "worker", "staircase"}

# BASE_PRIORITY = {
#     "person": 3, "human": 3, "worker": 3,
#     "vehicle": 3,
#     "machinery": 2,
#     "staircase": 2,
# }

# def normalize_label(label: str) -> str:
#     """Map your label string into canonical class names."""
#     s = (label or "").strip().lower()
#     if s in ["human", "worker", "person", "people", "man", "woman"]:
#         return "person"
#     if s in ["car", "truck", "forklift", "vehicle"]:
#         return "vehicle"
#     if s in ["machinery", "machine", "excavator", "crane"]:
#         return "machinery"
#     if s in ["stair", "stairs", "staircase"]:
#         return "staircase"
#     return s

# def is_moving_from_action_state(action_state: int) -> bool:
#     """
#     ZED zed_msgs/Object:
#       action_state: 0 -> IDLE, 2 -> MOVING
#     """
#     return int(action_state) != 0 

# def dynamic_boost(cls: str, moving: bool) -> int:
#     if cls == "machinery" and moving:
#         return 2
#     if cls == "vehicle" and moving:
#         return 1
#     if cls in ["person", "human", "worker"] and moving:
#         return 1
#     return 0

# def compute_risk_band(d: float, p_eff: int) -> int:
#     # Band 3
#     if d <= 1.0:
#         return 3
#     if p_eff >= 4 and d <= 2.0:
#         return 3

#     # Band 2
#     if 1.0 < d <= 2.5:
#         return 2
#     if p_eff >= 3 and 2.5 < d <= 4.0:
#         return 2

#     # Band 1 (your rule)
#     if 2.5 < d <= 5.0 and p_eff < 3:
#         return 1

#     # Conservative fallback for uncovered cases with d <= 5
#     if d <= 5.0:
#         return 1

#     return 0

# def band_to_velocity_scale(band: int) -> float:
#     if band == 3:
#         return 0.0
#     if band == 2:
#         return 0.5
#     return 1.0

# def stable_int32_from_str(s: str) -> int:
#     """
#     Stable surrogate id: md5 -> 32-bit positive int.
#     """
#     digest = hashlib.md5(s.encode("utf-8")).digest()
#     val = int.from_bytes(digest[:4], byteorder="big", signed=False)
#     return int(val & 0x7FFFFFFF)


# class VLMRiskGateNode(Node):
#     def __init__(self):
#         super().__init__("vlm_preprocessor")

#         # Topics / frames
#         self.declare_parameter("hazard_topic", "/hazard_objects")
#         self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color/compressed")
#         self.declare_parameter("target_frame", "map")
#         self.declare_parameter("base_frame", "base_link")

#         # Tracking / throttling
#         self.declare_parameter("track_forget_s", 10.0)
#         self.declare_parameter("surrogate_cell_size_m", 0.50)  # grid size for track_id fallback

#         self.hazard_topic = self.get_parameter("hazard_topic").value
#         self.image_topic = self.get_parameter("image_topic").value
#         self.target_frame = self.get_parameter("target_frame").value
#         self.base_frame = self.get_parameter("base_frame").value

#         self.track_forget_s = float(self.get_parameter("track_forget_s").value)
#         self.cell = float(self.get_parameter("surrogate_cell_size_m").value)

#         # TF for robot pose
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Cache latest image
#         self.latest_img: Optional[CompressedImage] = None
#         self.create_subscription(CompressedImage, self.image_topic, self.on_image, 10)

#         # Main input
#         self.create_subscription(HazardObjectArray, self.hazard_topic, self.on_hazards, 10)

#         # Outputs
#         self.pub_query = self.create_publisher(VLMQuery, "/vlm/query", 10)
#         self.pub_scale = self.create_publisher(Float32, "/velocity_scale", 10)

#         # Trigger memory (track_id -> last_band_triggered)
#         self.last_band: Dict[int, int] = {}
#         self.last_seen: Dict[int, float] = {}
#         self.create_timer(1.0, self.cleanup_tracks)

#         self.get_logger().info("Node C started: /hazard_objects -> risk bands -> /vlm/query + /velocity_scale")

#     def on_image(self, msg: CompressedImage):
#         self.latest_img = msg

#     def cleanup_tracks(self):
#         now = time.time()
#         dead = [tid for tid, t in self.last_seen.items() if (now - t) > self.track_forget_s]
#         for tid in dead:
#             self.last_seen.pop(tid, None)
#             self.last_band.pop(tid, None)

#     def get_robot_xy(self) -> Optional[Tuple[float, float]]:
#         """
#         Get robot position (x,y) in map using TF map->base_link.
#         """
#         try:
#             tf = self.tf_buffer.lookup_transform(
#                 self.target_frame,
#                 self.base_frame,
#                 Time(),
#                 timeout=Duration(seconds=0.2),
#             )
#             rx = tf.transform.translation.x
#             ry = tf.transform.translation.y
#             return (rx, ry)
#         except Exception as e:
#             self.get_logger().warn(f"TF lookup failed {self.target_frame}->{self.base_frame}: {e}")
#             return None

#     def make_track_id(self, obj) -> int:
#         """
#         Use real track_id if available; else create surrogate from label_id + position_map_2d grid cell.
#         """
#         tid = int(obj.track_id)
#         if tid >= 0:
#             return tid

#         # surrogate key based on coarse map location
#         x = float(obj.position_map_2d.x)
#         y = float(obj.position_map_2d.y)
#         cx = int(math.floor(x / self.cell))
#         cy = int(math.floor(y / self.cell))
#         key = f"{int(obj.label_id)}:{cx}:{cy}"
#         return stable_int32_from_str(key)

#     def on_hazards(self, msg: HazardObjectArray):
#         robot_xy = self.get_robot_xy()
#         if robot_xy is None:
#             return
#         rx, ry = robot_xy

#         min_scale = 1.0
#         now = time.time()

#         for obj in msg.objects:
#             # 1) class filtering
#             cls = normalize_label(obj.label)
#             if cls not in CONSIDERED:
#                 continue

#             # 2) moving state from ZED action_state
#             moving = is_moving_from_action_state(obj.action_state)

#             # staircase always static
#             if cls == "staircase":
#                 moving = False

#             # 3) P_eff = base + boost
#             p_base = BASE_PRIORITY.get(cls, 0)
#             p_eff = p_base + dynamic_boost(cls, moving)

#             # 4) distance d from robot to object (planar) using position_map_2d
#             ox = float(obj.position_map_2d.x)
#             oy = float(obj.position_map_2d.y)
#             dx = ox - rx
#             dy = oy - ry
#             d = math.sqrt(dx * dx + dy * dy)

#             # 5) risk band
#             band = compute_risk_band(d, p_eff)

#             # 6) velocity scale -> global min
#             scale = band_to_velocity_scale(band)
#             min_scale = min(min_scale, scale)

#             # 7) tracking key
#             track_id = self.make_track_id(obj)
#             self.last_seen[track_id] = now

#             # 8) Trigger rules: band in {1,2,3}, only on escalation
#             if band == 0:
#                 continue

#             prev_band = self.last_band.get(track_id, -1)
#             should_trigger = (prev_band < 0) or (band > prev_band)
#             if not should_trigger:
#                 continue

#             # Need image payload
#             if self.latest_img is None:
#                 continue

#             # 9) Build /vlm/query (no prompt)
#             q = VLMQuery()
#             q.header.stamp = msg.header.stamp
#             q.header.frame_id = "map"

#             q.class_name = cls
#             q.track_id = track_id
#             q.distance_m = float(d)
#             q.risk_band = int(band)
#             q.p_eff = int(p_eff)
#             q.is_moving = bool(moving)

#             q.object_pos_map.x = float(obj.position_map.x)
#             q.object_pos_map.y = float(obj.position_map.y)
#             q.object_pos_map.z = float(obj.position_map.z)

#             q.image = self.latest_img

#             self.pub_query.publish(q)
#             self.last_band[track_id] = band

#         # publish velocity scale (always)
#         out = Float32()
#         out.data = float(min_scale)
#         self.pub_scale.publish(out)


# def main():
#     rclpy.init()
#     node = VLMRiskGateNode()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
"""
Node C: VLM Risk Gate / Preprocessor (no prompt)
- Subscribes:  /hazard_objects   (HazardObjectArray)   [from Node B]
              /<image_topic>    (sensor_msgs/CompressedImage)  [latest frame cache]
- Publishes:   /vlm/query       (VLMQuery)  [metadata + compressed image]
              /velocity_scale  (std_msgs/Float32)  [min scale across hazards]

Fix included:
- Robust handling for NaN positions (prevents crash in surrogate track-id logic).
- If track_id is not available yet (track_id < 0), uses surrogate id from label_id + quantized map cell.
- Moving rule: action_state != 0 => moving
- Trigger rule (current version): retrigger only on risk-band escalation per track_id.
  (If you want "only once ever per track_id", I can switch this in 2 lines.)
"""

import math
import time
import hashlib
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage

from tf2_ros import Buffer, TransformListener

# -------------------------
# TODO: Change these imports to your actual message package names
# Your messages:
#   HazardObject3D  (not needed directly here)
#   HazardObjectArray with fields: header, objects[]
# -------------------------
from vlm_safety_msgs.msg import HazardObjectArray, VLMQuery


# -------------------------
# Class config
# -------------------------
CONSIDERED = {"vehicle", "machinery", "person", "human", "worker", "staircase"}

BASE_PRIORITY = {
    "person": 3,
    "human": 3,
    "worker": 3,
    "vehicle": 3,
    "machinery": 2,
    "staircase": 2,
}


def normalize_label(label: str) -> str:
    """Map raw labels to canonical class names used by the rule tables."""
    s = (label or "").strip().lower()
    if s in ["human", "worker", "person", "people", "man", "woman"]:
        return "person"
    if s in ["car", "truck", "forklift", "vehicle"]:
        return "vehicle"
    if s in ["machinery", "machine", "excavator", "crane"]:
        return "machinery"
    if s in ["stair", "stairs", "staircase"]:
        return "staircase"
    return s


def is_moving_from_action_state(action_state: int) -> bool:
    """
    Your rule: if action_state == 1 or 2 consider as moving.
    """
    return (int(action_state) == 1 or int(action_state) == 2)


def dynamic_boost(cls: str, moving: bool) -> int:
    """
    Dynamic boost rules:
      - Machinery moving: +2
      - Vehicle moving: +1
      - Human moving: +1
      - Staircase: +0
    """
    if cls == "machinery" and moving:
        return 2
    if cls == "vehicle" and moving:
        return 1
    if cls in ["person", "human", "worker"] and moving:
        return 1
    return 0


def compute_risk_band(d: float, p_eff: int) -> int:
    """
    Risk Band rules (order matters):

    Band 3 (Critical):
      - d <= 1.0
      - OR (p_eff >= 4 AND d <= 2.0)

    Band 2 (High):
      - 1.0 < d <= 2.5
      - OR (p_eff >= 3 AND 2.5 < d <= 4.0)

    Band 1 (Monitor):
      - 2.5 < d <= 5.0 AND p_eff < 3

    Band 0 (Ignore):
      - d > 5.0

    Note: your spec leaves a gap for p_eff>=3 and 4.0<d<=5.0.
    Conservative fallback: treat any d<=5.0 as Band 1 if not matched above.
    """
    # Band 3
    if d <= 1.0:
        return 3
    if p_eff >= 4 and d <= 2.0:
        return 3

    # Band 2
    if 1.0 < d <= 2.5:
        return 2
    if p_eff >= 3 and 2.5 < d <= 4.0:
        return 2

    # Band 1
    if 2.5 < d <= 5.0 and p_eff < 3:
        return 1

    # Conservative fallback for uncovered cases with d <= 5
    if d <= 5.0:
        return 1

    # Band 0
    return 0


def band_to_velocity_scale(band: int) -> float:
    """Velocity scaling rules."""
    if band == 3:
        return 0.0
    if band == 2:
        return 0.5
    return 1.0


def stable_int32_from_str(s: str) -> int:
    """Create a stable positive int32 from a string (for surrogate track IDs)."""
    digest = hashlib.md5(s.encode("utf-8")).digest()
    val = int.from_bytes(digest[:4], byteorder="big", signed=False)
    return int(val & 0x7FFFFFFF)


def is_finite_xy(x: float, y: float) -> bool:
    """True if both coordinates are finite (not NaN/inf)."""
    return math.isfinite(x) and math.isfinite(y)


def get_best_xy(obj) -> Optional[Tuple[float, float]]:
    """
    Prefer position_map_2d (already grounded z=0), else fall back to position_map.
    Return None if both are invalid.
    """
    x2 = float(obj.position_map_2d.x)
    y2 = float(obj.position_map_2d.y)
    if is_finite_xy(x2, y2):
        return (x2, y2)

    xm = float(obj.position_map.x)
    ym = float(obj.position_map.y)
    if is_finite_xy(xm, ym):
        return (xm, ym)

    return None


class VLMRiskGateNode(Node):
    def __init__(self):
        super().__init__("vlm_risk_gate_node")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("hazard_topic", "/hazard_objects")
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color/compressed")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("min_vlm_band", 2)
        self.min_vlm_band = int(self.get_parameter("min_vlm_band").value)
        # --- Cooldown parameters ---
        self.declare_parameter("min_retrigger_s", 8.0)
        self.min_retrigger_s = float(self.get_parameter("min_retrigger_s").value)

        # track_id -> last VLM query time
        self.last_query_time = {}


        self.declare_parameter("track_forget_s", 10.0)
        self.declare_parameter("surrogate_cell_size_m", 0.50)  # for surrogate IDs

        self.hazard_topic = self.get_parameter("hazard_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.track_forget_s = float(self.get_parameter("track_forget_s").value)
        self.cell = float(self.get_parameter("surrogate_cell_size_m").value)

        # -------------------------
        # TF (robot pose) + latest image cache
        # -------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_img: Optional[CompressedImage] = None
        self.create_subscription(CompressedImage, self.image_topic, self.on_image, 10)

        # -------------------------
        # Input hazards
        # -------------------------
        self.create_subscription(HazardObjectArray, self.hazard_topic, self.on_hazards, 10)

        # -------------------------
        # Outputs
        # -------------------------
        self.pub_query = self.create_publisher(VLMQuery, "/vlm/query", 10)
        self.pub_scale = self.create_publisher(Float32, "/velocity_scale", 10)

        # -------------------------
        # Trigger memory: track_id -> last triggered band
        # (so it only retriggers on escalation)
        # -------------------------
        self.last_band: Dict[int, int] = {}
        self.last_seen: Dict[int, float] = {}

        self.create_timer(1.0, self.cleanup_tracks)

        self.get_logger().info("Node C started (risk gate). NaN-safe surrogate tracking enabled.")

    def on_image(self, msg: CompressedImage):
        """Cache latest compressed frame."""
        self.latest_img = msg

    def cleanup_tracks(self):
        """Forget old tracks so dicts don't grow forever."""
        now = time.time()
        dead = [tid for tid, t in self.last_seen.items() if (now - t) > self.track_forget_s]
        for tid in dead:
            self.last_seen.pop(tid, None)
            self.last_band.pop(tid, None)

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        """Get robot (x,y) in map from TF map->base_link."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {self.target_frame}->{self.base_frame}: {e}")
            return None

    def make_track_id(self, obj) -> int:
        """
        If obj.track_id >= 0, use it.
        Else create surrogate ID based on label_id + quantized map position.

        NaN fix: if we cannot get a finite (x,y), return -1 (caller will skip).
        """
        tid = int(obj.track_id)
        if tid >= 0:
            return tid

        xy = get_best_xy(obj)
        if xy is None:
            return -1

        x, y = xy
        cx = int(math.floor(x / self.cell))
        cy = int(math.floor(y / self.cell))
        key = f"{int(obj.label_id)}:{cx}:{cy}"
        return stable_int32_from_str(key)

    def on_hazards(self, msg: HazardObjectArray):
        """Main callback: compute bands/scales, publish velocity_scale + VLMQuery (no prompt)."""
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return
        rx, ry = robot_xy

        min_scale = 1.0
        now = time.time()

        for obj in msg.objects:
            # 1) Normalize class & filter
            cls = normalize_label(obj.label)
            if cls not in CONSIDERED:
                continue

            # 2) Moving state
            moving = is_moving_from_action_state(obj.action_state)
            if cls == "staircase":
                moving = False  # forced static

            # 3) Effective priority
            p_base = BASE_PRIORITY.get(cls, 0)
            p_eff = p_base + dynamic_boost(cls, moving)

            # 4) Need valid object XY in map for distance + surrogate tracking
            xy = get_best_xy(obj)
            if xy is None:
                # Don't crash: just skip this detection instance
                self.get_logger().warn(
                    f"Skipping object with invalid (NaN) map position: label={obj.label} label_id={obj.label_id}"
                )
                continue
            ox, oy = xy

            # 5) Distance d (planar)
            dx = ox - rx
            dy = oy - ry
            d = math.sqrt(dx * dx + dy * dy)

            # 6) Risk band
            band = compute_risk_band(d, p_eff)

            # 7) Velocity scale (global min)
            scale = band_to_velocity_scale(band)
            min_scale = min(min_scale, scale)

            # 8) Track id (real or surrogate)
            track_id = self.make_track_id(obj)
            if track_id < 0:
                # Still cannot create stable ID (no valid XY earlier would have continued anyway)
                continue

            self.last_seen[track_id] = now

            # 9) Trigger rules: only band when band >= min_vlm_band, and only on escalation
            # (a) Band gating (Option 2)
            if band < self.min_vlm_band:
                continue

            prev_band = self.last_band.get(track_id, -1)
            self.last_band[track_id] = band

            # (b) Escalation check
            if not (prev_band < 0 or band > prev_band):
                continue

            # (c) Cooldown check
            last_q = self.last_query_time.get(track_id, 0.0)
            if (now - last_q) < self.min_retrigger_s:
                continue

            # Need image payload
            if self.latest_img is None:
                continue

            # 10) Build /vlm/query message (NO PROMPT)
            q = VLMQuery()
            q.header.stamp = msg.header.stamp
            q.header.frame_id = "map"

            q.class_name = cls
            q.track_id = track_id
            q.distance_m = float(d)
            q.risk_band = int(band)
            q.p_eff = int(p_eff)
            q.is_moving = bool(moving)

            # Use your 3D position_map for logging/debug
            q.object_pos_map.x = float(obj.position_map.x)
            q.object_pos_map.y = float(obj.position_map.y)
            q.object_pos_map.z = float(obj.position_map.z)

            q.image = self.latest_img

            # self.pub_query.publish(q)
            # self.last_band[track_id] = band
            self.pub_query.publish(q)
            self.last_band[track_id] = band
            self.last_query_time[track_id] = now

        # Publish velocity scale once per hazard message
        out = Float32()
        out.data = float(min_scale)
        self.pub_scale.publish(out)


def main():
    rclpy.init()
    node = VLMRiskGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
