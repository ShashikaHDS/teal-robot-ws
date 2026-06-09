import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


class Colorizer(Node):
    def __init__(self):
        super().__init__('pointcloud_colorize')

        self.declare_parameter('cloud_topic', '/ouster/points')
        self.declare_parameter('image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/left/camera_info')
        self.declare_parameter('output_topic', '/colored_points')
        self.declare_parameter('drop_uncolored', False)
        self.declare_parameter('default_rgb', [128, 128, 128])
        self.declare_parameter('min_z_in_cam', 0.05)

        cloud_topic = self.get_parameter('cloud_topic').value
        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.drop_uncolored = bool(self.get_parameter('drop_uncolored').value)
        self.default_rgb = np.array(
            self.get_parameter('default_rgb').value, dtype=np.uint8)
        self.min_z = float(self.get_parameter('min_z_in_cam').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.image = None
        self.K = None
        self.W = self.H = 0
        self.cam_frame = None

        self.create_subscription(Image, image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._info_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'colorize: {cloud_topic} + {image_topic} -> {out_topic}')

    def _image_cb(self, msg: Image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')

    def _info_cb(self, msg: CameraInfo):
        self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.W, self.H = int(msg.width), int(msg.height)
        self.cam_frame = msg.header.frame_id

    def _cloud_cb(self, msg: PointCloud2):
        if self.image is None or self.K is None or not self.cam_frame:
            return
        try:
            tfmsg = self.tf_buffer.lookup_transform(
                self.cam_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'TF {msg.header.frame_id}->{self.cam_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return

        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.asarray(pts.tolist(), dtype=np.float32) if not isinstance(pts, np.ndarray) \
            else np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float32)
        if pts.size == 0:
            return

        T = self._tf_to_matrix(tfmsg.transform)
        cam = pts @ T[:3, :3].T + T[:3, 3]

        z = cam[:, 2]
        valid = z > self.min_z
        u = np.zeros_like(z)
        v = np.zeros_like(z)
        u[valid] = self.K[0, 0] * cam[valid, 0] / z[valid] + self.K[0, 2]
        v[valid] = self.K[1, 1] * cam[valid, 1] / z[valid] + self.K[1, 2]
        ui = u.astype(np.int32)
        vi = v.astype(np.int32)
        in_img = valid & (ui >= 0) & (ui < self.W) & (vi >= 0) & (vi < self.H)

        rgb = np.tile(self.default_rgb, (pts.shape[0], 1))
        rgb[in_img] = self.image[vi[in_img], ui[in_img]]

        if self.drop_uncolored:
            pts = pts[in_img]
            rgb = rgb[in_img]
        if pts.shape[0] == 0:
            return

        rgb_packed = (rgb[:, 0].astype(np.uint32) << 16) | \
                     (rgb[:, 1].astype(np.uint32) << 8) | \
                     (rgb[:, 2].astype(np.uint32))

        record = np.empty(pts.shape[0], dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        record['x'] = pts[:, 0]
        record['y'] = pts[:, 1]
        record['z'] = pts[:, 2]
        record['rgb'] = rgb_packed.view(np.float32)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = record.shape[0]
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.data = record.tobytes()
        out.is_dense = True
        self.pub.publish(out)

    @staticmethod
    def _tf_to_matrix(t):
        x, y, z = t.translation.x, t.translation.y, t.translation.z
        qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        n = qx * qx + qy * qy + qz * qz + qw * qw
        s = 2.0 / n if n > 0 else 0.0
        xx, yy, zz = qx * qx * s, qy * qy * s, qz * qz * s
        xy, xz, yz = qx * qy * s, qx * qz * s, qy * qz * s
        wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
        T = np.eye(4)
        T[:3, :3] = [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
        T[:3, 3] = [x, y, z]
        return T


def main():
    rclpy.init()
    node = Colorizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
