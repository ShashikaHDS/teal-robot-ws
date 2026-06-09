#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

class YawAccumulator(Node):
    def __init__(self):
        super().__init__("yaw_accumulator")
        self.sub = self.create_subscription(Odometry, "/odom", self.cb, 10)
        self.prev_yaw = None
        self.total_yaw = 0.0
        self.started = False

    def cb(self, msg):
        o = msg.pose.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)

        if self.prev_yaw is None:
            self.prev_yaw = yaw
            return

        # unwrap difference
        dyaw = yaw - self.prev_yaw
        while dyaw > math.pi:
            dyaw -= 2*math.pi
        while dyaw < -math.pi:
            dyaw += 2*math.pi

        if self.started:
            self.total_yaw += dyaw

        self.prev_yaw = yaw

def main():
    rclpy.init()
    node = YawAccumulator()

    print("Waiting for odom...")
    while rclpy.ok() and node.prev_yaw is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    input("Press Enter to START accumulating yaw...")
    node.started = True

    input("Rotate robot physically 360°, then press Enter to STOP...")
    node.started = False

    total_rad = node.total_yaw
    total_deg = total_rad * 180.0 / math.pi

    print("\n===== RESULT =====")
    print(f"Total yaw (rad): {total_rad:.4f}")
    print(f"Total yaw (deg): {total_deg:.2f}")

    print("\nExpected for 360°:")
    print("Radians ≈ 6.2832")
    print("Degrees ≈ 360.00")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

