#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

import meerkat_motors.oriental_driver as oriental_driver

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np
import math


def quaternion_from_yaw(yaw: float):
    """2D yaw-only quaternion (qx, qy, qz, qw)."""
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class OrientalRosWrapper(Node):
    def __init__(self):
        super().__init__('oriental_ros')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.gear_ratio = 30.0

        # Tracked robot: treat as diff drive
        self.track_width = 0.38          # distance between track centerlines (m)
        self.sprocket_radius = 0.0506 * 0.766382 * 1.024335

        # -----------------------------
        # Motor init
        # -----------------------------
        # RIGHT
        port_r = '/dev/Motor_R'
        addr_r = 1
        driver_r = oriental_driver.OrienDriver(port_r)
        self.motor_r = driver_r.initialize(addr_r)

        # LEFT
        port_l = '/dev/Motor_L'
        addr_l = 2
        driver_l = oriental_driver.OrienDriver(port_l)
        self.motor_l = driver_l.initialize(addr_l)

        # -----------------------------
        # Target speeds (set by cmd_vel callback, applied by timer)
        # This decouples cmd_vel processing from blocking Modbus I/O
        # -----------------------------
        self.target_rpm_r = 0.0
        self.target_rpm_l = 0.0
        self.last_written_rpm_r = None  # track what we last sent to avoid redundant writes
        self.last_written_rpm_l = None

        # -----------------------------
        # Publishers (motor feedback)
        # -----------------------------
        self.speed_pub_r = self.create_publisher(Float64, 'oriental/motor1/speed', 10)
        self.voltage_pub_r = self.create_publisher(Float64, 'oriental/motor1/voltage', 10)

        self.speed_pub_l = self.create_publisher(Float64, 'oriental/motor2/speed', 10)
        self.voltage_pub_l = self.create_publisher(Float64, 'oriental/motor2/voltage', 10)

        # -----------------------------
        # Odometry pub + TF
        # -----------------------------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        #self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # -----------------------------
        # Subscriber (cmd_vel)
        # -----------------------------
        self.cmd_sub = self.create_subscription(
            Twist, 'meerkat/cmd_vel', self.listener_callback, 10
        )

        # -----------------------------
        # Timer — reduced to 30 Hz (was 100 Hz)
        # 8 Modbus reads at ~3-5ms each = ~30ms.  At 100Hz (10ms period)
        # the timer was ALWAYS overrunning, blocking the cmd_vel callback.
        # 30 Hz (33ms period) gives the Modbus reads enough room and
        # keeps the executor responsive.
        # -----------------------------
        self.timer_period = 0.033  # ~30 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("oriental_ros started (publishing /odom and TF odom->base_link)")

        # Watchdog — continuous stop (not one-shot)
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.25  # seconds
        self.cmd_timed_out = False

    def listener_callback(self, msg: Twist):
        """
        Convert cmd_vel -> target motor RPM.
        NO Modbus I/O here — just math and storing two floats.
        This ensures cmd_vel is processed instantly, never blocked.
        """
        v = msg.linear.x
        wz = msg.angular.z

        L = self.track_width
        r = self.sprocket_radius

        # sprocket angular speeds (rad/s)
        w_r = (v / r) + ((L / (2.0 * r)) * wz)
        w_l = (v / r) - ((L / (2.0 * r)) * wz)

        self.last_cmd_time = self.get_clock().now()
        self.cmd_timed_out = False

        # rad/s -> sprocket RPM
        rpm_r_sprocket = w_r * 60.0 / (2.0 * np.pi)
        rpm_l_sprocket = w_l * 60.0 / (2.0 * np.pi)

        # sprocket RPM -> motor shaft RPM (gear ratio)
        rpm_r_motor = rpm_r_sprocket * self.gear_ratio
        rpm_l_motor = rpm_l_sprocket * self.gear_ratio

        # Right motor direction inverted
        rpm_r_motor *= -1.0

        # clamp to limits
        rpm_r_motor = max(min(rpm_r_motor, 3150.0), -3150.0)
        rpm_l_motor = max(min(rpm_l_motor, 3150.0), -3150.0)

        # Store target — the timer will write them to Modbus
        self.target_rpm_r = rpm_r_motor
        self.target_rpm_l = rpm_l_motor

    def timer_callback(self):
        """
        Runs at ~30 Hz. Does ALL Modbus I/O:
        1. Watchdog check — if no cmd_vel for 0.25s, force target to 0
        2. Write target speeds to motors
        3. Read feedback (speed + voltage)
        4. Compute and publish odom
        """

        # --------- Watchdog: continuous stop if cmd_vel dies ----------
        now = self.get_clock().now()
        dt_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9

        if dt_cmd > self.cmd_timeout:
            # Force targets to zero — continuously, not just once
            self.target_rpm_r = 0.0
            self.target_rpm_l = 0.0
            if not self.cmd_timed_out:
                self.cmd_timed_out = True
                self.get_logger().warn("cmd_vel timeout: stopping motors")

        # --------- Write target speeds to motors ----------
        try:
            self.motor_r.writeSpeed(self.target_rpm_r)
        except Exception as e:
            self.get_logger().error(f'Motor R write error: {e}')

        try:
            self.motor_l.writeSpeed(self.target_rpm_l)
        except Exception as e:
            self.get_logger().error(f'Motor L write error: {e}')

        # --------- Read feedback RPM (motor shaft) ----------
        try:
            rpm_r = float(self.motor_r.readSpeed()) * (-1.0)
        except Exception:
            rpm_r = 0.0

        try:
            rpm_l = float(self.motor_l.readSpeed())
        except Exception:
            rpm_l = 0.0

        # Publish speed topics
        msg = Float64()
        msg.data = rpm_r if (-65000.0 <= rpm_r <= 65000.0) else 0.0
        self.speed_pub_r.publish(msg)

        try:
            msg = Float64()
            msg.data = float(self.motor_r.readVoltage()) / 10.0
            self.voltage_pub_r.publish(msg)
        except Exception:
            pass

        msg = Float64()
        msg.data = rpm_l if (-65000.0 <= rpm_l <= 65000.0) else 0.0
        self.speed_pub_l.publish(msg)

        try:
            msg = Float64()
            msg.data = float(self.motor_l.readVoltage()) / 10.0
            self.voltage_pub_l.publish(msg)
        except Exception:
            pass

        # --------- Odom time step ----------
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9

        if dt <= 0.0:
            self.last_time = now
            dt = self.timer_period
        else:
            self.last_time = now

        # --------- Convert RPM -> track velocities ----------
        w_r = (rpm_r / self.gear_ratio) * (2.0 * math.pi / 60.0)
        w_l = (rpm_l / self.gear_ratio) * (2.0 * math.pi / 60.0)

        v_r = self.sprocket_radius * w_r
        v_l = self.sprocket_radius * w_l

        v = 0.5 * (v_r + v_l)
        omega = (v_r - v_l) / self.track_width

        # --------- Integrate pose ----------
        self.yaw += omega * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # --------- Publish /odom ----------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # --------- TF: odom -> base_link ----------
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        #self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OrientalRosWrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
