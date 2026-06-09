#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomMeasure(Node):
    def __init__(self):
        super().__init__("odom_measure")
        self.sub = self.create_subscription(Odometry, "/odom", self.cb, 10)
        self.last = None

    def cb(self, msg):
        self.last = msg

    def get_xy(self):
        if self.last is None:
            return None
        p = self.last.pose.pose.position
        return (p.x, p.y)

def main():
    rclpy.init()
    node = OdomMeasure()

    # Spin until we get at least one odom message
    while rclpy.ok() and node.get_xy() is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    input("Press Enter to CAPTURE START odom (robot at start line)...")
    start = node.get_xy()
    print("START:", start)

    input("Now drive the robot to 2.5m mark. Press Enter to CAPTURE END odom...")
    # make sure we have the latest odom
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    end = node.get_xy()
    print("END:", end)

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    d_odom = math.sqrt(dx*dx + dy*dy)

    print(f"\nOdom distance = {d_odom:.4f} m")
    print("Real distance = 2.5000 m")
    if d_odom > 1e-6:
        k = 2.5 / d_odom
        print(f"Scale factor k = real/odom = {k:.6f}")
        print("Meaning: multiply your meters-per-rev (or wheel radius / meters-per-tick) by k")
    else:
        print("Odom distance too small/zero. Check odom is updating.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

