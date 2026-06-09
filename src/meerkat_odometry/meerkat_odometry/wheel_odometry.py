#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
import rclpy.clock
from rclpy.node import Node
import meerkat_odometry.oriental_driver as oriental_driver
from ds4_driver_msgs.msg import Status
from rclpy.duration import Duration
import rclpy.time
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np
from std_msgs.msg import Header


class MeerkatOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')
        self.theta = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.reset = False
        self.step = 0.0

        self.initial_time = self.get_clock().now()
        self.current_time = self.get_clock().now()
        self.gear_ratio = 30     #To be added in ros para
        self.wheel_base = 0.38  #To be added in ros para
        self.wheel_radius = 0.07    #To be added in ros para
    
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

        self.pre_theta = 0.0
        self.imu_yaw = 0.0
        self.i = 0
        
        # Publishers
        self.odometry_publisher = self.create_publisher(Odometry, 'wheel/odometry', 10)
        
        # TF Broadcaster for publishing transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.left_motor_speed_subscription = self.create_subscription(Float64,'oriental/motor1/speed', self.left_motor_callback, 10)
        self.left_motor_speed_subscription  # prevent unused variable warning

        self.right_motor_speed_subscription = self.create_subscription(Float64,'oriental/motor2/speed', self.right_motor_callback, 10)
        self.right_motor_speed_subscription  # prevent unused variable warning

        self.dual_shock_subscription = self.create_subscription(Status,'/status',self.reset_odom, 10)
        self.dual_shock_subscription  # prevent unused variable warning
        
        self.ventor_nav_imu = self.create_subscription(Imu,'/vectornav/imu', self.convert_to_RPY, 10)
        self.ventor_nav_imu  # prevent unused variable warning

        self.timer_period = 0.01  # seconds
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback) 
        
          
    

    def convert_to_RPY(self, msg):
        quaternion_array = [msg.orientation.x, 
                            msg.orientation.y,
                            msg.orientation.z,
                            msg.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_array)
        self.imu_yaw = float(yaw) 
        if self.i==0 and self.imu_yaw != 0.0:
            self.pre_theta = self.imu_yaw
            self.i += 1

    def reset_odom(self, msg):
        self.reset = msg.button_circle

    def change_in_time(self):
        self.current_time = self.get_clock().now()
        elasped_time = self.current_time - self.initial_time
        self.initial_time = self.current_time
        #self.pre_theta = self.imu_yaw
        return float(elasped_time.nanoseconds/1e9)

    def left_motor_callback(self, msg):
        speed = msg.data
        self.left_speed = (speed*2*np.pi)/(self.gear_ratio*60)
        #elasped_time = self.change_in_time()
        #self.get_logger().info(f'Publishing Speed Motor1: {float(speed)}')
    
    def right_motor_callback(self, msg):
        speed = msg.data
        self.right_speed = (speed*2*np.pi)/(self.gear_ratio*60)
        #elasped_time = self.change_in_time()
        #self.get_logger().info(f'Publishing Speed Motor2: {float(speed)}')

    def timer_callback(self):
        #self.get_logger().info(f'Left Speed: {self.left_speed}, Right Speed: {self.right_speed}')

        self.vx = self.wheel_radius*(self.right_speed + self.left_speed)/2.0
        self.vy = 0.0
        self.vth = self.wheel_radius*(self.right_speed - self.left_speed)/(self.wheel_base)
        
        dt = self.change_in_time()
        #self.get_logger().info(f'dt: {dt}')
        #dt = self.timer_period
        delta_x = (self.vx * np.cos(self.theta) - self.vy * np.cos(self.theta)) * dt
        delta_y = (self.vx * np.sin(self.theta) + self.vy * np.sin(self.theta)) * dt
        #delta_theta = self.pre_theta - self.imu_yaw
        #vth = delta_theta/dt

        delta_theta = self.vth * dt
        #delta_theta = self.vth * dt
        self.theta -= delta_theta
        
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        self.x += delta_x
        self.y += delta_y
        #self.theta  = delta_theta
        #self.theta = self.imu_yaw
        #self.pre_theta = self.theta


       # if self.reset:
        #    self.x = 0.0
         #   self.y = 0.0
          #  self.theta = 0.0
           # self.i = 0
            
        
        
        # Create and publish odometry message
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'  # FIXED: Changed from 'base_footprint' to 'base_link'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y 
        msg.pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0,0,self.theta)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.vth

        msg.pose.covariance = [0.0] * 36
        msg.twist.covariance = [0.0] * 36
        
        # Publish odometry
        self.odometry_publisher.publish(msg)
        
        # ADDED: Broadcast the transform from odom to base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        
        # self.get_logger().info(f'\nPublishing x: {msg.pose.pose.position.x}\nPublishing y: {msg.pose.pose.position.y}\nPublishing yaw: {self.theta}')
                            # Time Period: {dt}\nDelta X: {delta_x}\nDelta Y: {delta_y}\nMotor1 Speed: {self.left_speed}\nMotor2 Speed: {self.right_speed}\nVx: {self.vx}\nVy: {self.vy}')
        



def main(args=None):
    rclpy.init(args=args)

    odometry = MeerkatOdometry()

    rclpy.spin(odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
