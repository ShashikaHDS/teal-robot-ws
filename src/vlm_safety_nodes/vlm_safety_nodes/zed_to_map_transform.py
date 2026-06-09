#!/usr/bin/env python3
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from zed_msgs.msg import ObjectsStamped, BoundingBox3D, BoundingBox2Df
from vlm_safety_msgs.msg import HazardObject3D, HazardObjectArray

def point_from_xyz(x: float, y: float, z: float) -> Point:
    # Helper: make geometry_msgs/Point from xyz.
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p

def point_from_float3(arr3) -> Point:
    # Helper: convert ZED float32[3] -> geometry_msgs/Point.
    return point_from_xyz(arr3[0], arr3[1], arr3[2])

class ZedTOMapFrame(Node):
    # Node: zed_to_map_frame
    # Input : zed_msgs/ObjectsStamped
    # Output: vlm_safety_msgs/HazardObjectArray on /hazard_objects

    # TF:
    #   target_frame (map) <- source_frame (msg.header.frame_id)
    def __init__(self):
        super().__init__("zed_to_map_frame")

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('input_topic', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('output_topic', '/hazard_objects')
        self.declare_parameter('tf_timeout_sec', 0.2)

        self.target_frame = self.get_parameter('target_frame').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #/ Subscribers and Publishers
        self.obj_sub = self.create_subscription(ObjectsStamped,
                                                        self.input_topic, self.cb_objects, qos_profile_sensor_data)
        
        self.pub_haz = self.create_publisher(HazardObjectArray, self.output_topic, 10)

        self.get_logger().info(f"Subscribed to {self.input_topic}, publishing to {self.output_topic}, transforming to frame '{self.target_frame}'")

    def lookup_tf(self, source_frame: str, stamp):
        # Lookup transform from source_frame to target_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout)
            )
            return transform
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed from '{source_frame}' to '{self.target_frame}': {e}")
            return None     
        
    def tf_point(self, tf, source_frame: str, stamp, p_src: Point) -> Point:
        # Transform point p_src from source_frame to target_frame using tf
        # Apply TF to a point.
        # Implements: p_map = R * p_src + t (done inside TF).
        ps = PointStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = source_frame
        ps.point = p_src
        ps_map = do_transform_point(ps, tf)
        return ps_map.point
    
    def transform_bbox3d(self, tf, source_frame: str, stamp, bbox_src: BoundingBox3D) -> BoundingBox3D:
        # Transform ZED BoundingBox3D from source_frame to target_frame using tf
        bbox_map = BoundingBox3D()
        for i in range(8):
            kp = bbox_src.corners[i].kp
            p_src = point_from_xyz(kp[0], kp[1], kp[2])
            p_map = self.tf_point(tf, source_frame, stamp, p_src)
            bbox_map.corners[i].kp = [float(p_map.x), float(p_map.y), float(p_map.z)]
        return bbox_map
    
    def compute_bbox2d_map(self, bbox3d_map: BoundingBox3D) -> BoundingBox2Df:
        # Compute 2D bounding box in map frame from 3D bounding box in map frame
        xs = [float(bbox3d_map.corners[i].kp[0]) for i in range(8)]
        ys = [float(bbox3d_map.corners[i].kp[1]) for i in range(8)] 

        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        bbox2d_map = BoundingBox2Df()

        bbox2d_map.corners[0].kp = [minx, miny]  # top-left
        bbox2d_map.corners[1].kp = [maxx, miny]  # top-right
        bbox2d_map.corners[2].kp = [maxx, maxy]  # bottom-right
        bbox2d_map.corners[3].kp = [minx, maxy]  # bottom-left

        return bbox2d_map
    
    def cb_objects(self, msg: ObjectsStamped):

        # Main callback.
        # 1) get source_frame and stamp
        # 2) lookup TF once
        # 3) for each object build HazardObject3D
        # 4) publish HazardObjectArray
        source_frame = msg.header.frame_id
        stamp = msg.header.stamp

        if not source_frame:
            self.get_logger().warn("Received ObjectsStamped with empty frame_id, ZED msg.header.frame_id is empty.")
            return
        
        try:
            tf = self.lookup_tf(source_frame, stamp)
            if tf is None:
                return

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {self.target_frame} <- {source_frame}: {e}")
            return

        out_arr = HazardObjectArray()
        out_arr.header = msg.header
        out_arr.header.frame_id = self.target_frame
        for obj in msg.objects:
            ho = HazardObject3D()
            # Output header in MAP
            ho.header.stamp = stamp
            ho.header.frame_id = self.target_frame

            # -------- Identity --------
            ho.label = str(obj.label)
            ho.label_id = int(obj.label_id)
            ho.confidence = float(obj.confidence)
            ho.track_id = -1  # placeholder until tracking id is available

            # -------- Motion state --------
            ho.action_state = int(obj.action_state)
            # 0: Satic and 2: Moving

            # -------- Source bookkeeping --------
            ho.source_stamp = stamp
            ho.source_frame = str(source_frame)

            # -------- Centroid --------
            ho.position_src = point_from_float3(obj.position)
            ho.position_map = self.tf_point(tf, source_frame, stamp, ho.position_src)

            # 2D grounded centroid
            ho.position_map_2d.x = float(ho.position_map.x)
            ho.position_map_2d.y = float(ho.position_map.y)
            ho.position_map_2d.z = 0.0

            # -------- 2D bbox pixels (direct from ZED) --------
            ho.bbox2d_img_px = obj.bounding_box_2d

            # -------- 3D bbox --------
            ho.bbox3d_src = obj.bounding_box_3d
            ho.bbox3d_map = self.transform_bbox3d(tf, source_frame, stamp, ho.bbox3d_src)

            # -------- 2D bbox in map ground --------
            ho.bbox2d_map = self.compute_bbox2d_map(ho.bbox3d_map)

            out_arr.objects.append(ho)

        self.pub_haz.publish(out_arr)


def main():
    rclpy.init()
    node = ZedTOMapFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()