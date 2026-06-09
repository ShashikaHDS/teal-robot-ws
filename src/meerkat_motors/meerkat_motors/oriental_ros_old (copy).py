#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import meerkat_motors.oriental_driver as oriental_driver
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np

##### For odm calculation

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster



import math

def quaternion_from_yaw(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)

#######


# RIGHT
port1 = '/dev/Motor_R'
address1 = 1
motor_driver1 = oriental_driver.OrienDriver(port1)
motor1 = motor_driver1.initialize(address1)

# LEFT
port2 = '/dev/Motor_L'
address2 = 2
motor_driver2 = oriental_driver.OrienDriver(port2)
motor2 = motor_driver2.initialize(address2)
linear_x = 0.0
angular_z = 0.0

class OrientalRosWrapper(Node):

    def __init__(self):
        super().__init__('oriental_ros')
        self.gear_ratio = 30
        self.speed_publisher_motor1 = self.create_publisher(Float64, 'oriental/motor1/speed', 10)
        self.voltage_publisher_motor1 = self.create_publisher(Float64, 'oriental/motor1/voltage', 10)

        self.speed_publisher_motor2 = self.create_publisher(Float64, 'oriental/motor2/speed', 10)
        self.voltage_publisher_motor2 = self.create_publisher(Float64, 'oriental/motor2/voltage', 10)
        
        self.speed_subscription = self.create_subscription(Twist,'meerkat/cmd_vel', self.listener_callback, 10)
        self.speed_subscription  # prevent unused variable warning

        timer_period = 0.01  # seconds
        self.i = 0

        self.timer = self.create_timer(timer_period, self.timer_callback)        


    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        wheel_base = 0.38
        wheel_radius = 0.099
        
        right_speed = (linear_x / wheel_radius) + ((wheel_base/wheel_radius) * angular_z)
        left_speed = (linear_x / wheel_radius) - ((wheel_base/wheel_radius) * angular_z)

        right_speed = right_speed * self.gear_ratio * 60 / (2 * np.pi)*(-1)
        left_speed = (left_speed * self.gear_ratio * 60 / (2 * np.pi))

        #self.get_logger().info(f'Speed M1: {right_speed}')
        #self.get_logger().info(f'Speed M2: {left_speed}')
        
        if right_speed > 3150:
            right_speed = 3150

        if right_speed < -3150:
            right_speed = -3150

        if left_speed < -3150:
            left_speed = -3150

        if left_speed > 3150:
            left_speed = 3150

        motor1.writeSpeed(right_speed)
        motor2.writeSpeed(left_speed)


    def timer_callback(self):
        speed_msg = Float64()
        speed_msg.data = float(motor1.readSpeed()) * (-1)
        if speed_msg.data > 65000.0 or speed_msg.data < -65000.0:
            speed_msg.data = 0.0
        self.speed_publisher_motor1.publish(speed_msg)
        #self.get_logger().info(f'Publishing Speed Motor1: {speed_msg.data}')
        self.i += 1

        voltage_msg = Float64()
        voltage_msg.data = float(motor1.readVoltage())/10.0
        self.voltage_publisher_motor1.publish(voltage_msg)
        # self.get_logger().info(f'Publishing Voltage Motor1: {voltage_msg.data}')

        speed_msg = Float64()
        speed_msg.data = float(motor2.readSpeed())
        if speed_msg.data > 65000.0 or speed_msg.data < -65000.0:
            speed_msg.data = 0.0
        self.speed_publisher_motor2.publish(speed_msg)
        #self.get_logger().info(f'Publishing Speed Motor2: {speed_msg.data}')
        self.i += 1

        voltage_msg = Float64()
        voltage_msg.data = float(motor2.readVoltage())/10.0
        self.voltage_publisher_motor2.publish(voltage_msg)
        # self.get_logger().info(f'Publishing Voltage Motor2: {voltage_msg.data}')

def main(args=None):
    rclpy.init(args=args)

    oriental = OrientalRosWrapper()

    rclpy.spin(oriental)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oriental.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
