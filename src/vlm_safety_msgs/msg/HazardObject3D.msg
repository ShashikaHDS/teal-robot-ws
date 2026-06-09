# Output header in MAP frame (frame_id="map")
std_msgs/Header header

# --- Identity (required) ---
string label          # ZED: obj.label  (class name)
int16 label_id        # ZED: obj.label_id
float32 confidence    # ZED: obj.confidence
int32 track_id        # keep for later (your ZED msg doesn't expose it yet → publish -1 for now)

# --- Motion state (required) ---
int8 action_state     # ZED: obj.action_state (used as static/dynamic):contentReference[oaicite:1]{index=1}

# --- Source bookkeeping (required) ---
builtin_interfaces/Time source_stamp   # copy of input header.stamp (time sync):contentReference[oaicite:2]{index=2}
string source_frame                   # copy of input header.frame_id (source frame):contentReference[oaicite:3]{index=3}

# --- Centroid (required) ---
geometry_msgs/Point position_src      # centroid in source_frame (from obj.position)
geometry_msgs/Point position_map      # centroid transformed to map
geometry_msgs/Point position_map_2d   # same as position_map but z=0 (grounded)

# --- 3D bounding box (required) ---
zed_msgs/BoundingBox3D bbox3d_src     # ZED: obj.bounding_box_3d (camera/source frame):contentReference[oaicite:4]{index=4}
zed_msgs/BoundingBox3D bbox3d_map     # bbox3d_src corners transformed to map

# ==========================================================
# Your required 2D outputs
# ==========================================================

# (1) Image 2D bbox in pixels (from ZED)
zed_msgs/BoundingBox2Di bbox2d_img_px # ZED: obj.bounding_box_2d (pixel corners):contentReference[oaicite:5]{index=5}

# (2) Map 2D bbox on ground plane (computed by this node)
# Use float corners so kp=[x,y] are in map meters
zed_msgs/BoundingBox2Df bbox2d_map
