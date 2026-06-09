import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from ds4_driver_msgs.msg import Status
from geometry_msgs.msg import Twist

from time import time

class DualShockSubscriber(Node):

    def __init__(self):
        super().__init__('dualshock_ros')

        self.linear_speed = 0
        self.angular_speed1 = 0
        self.angular_speed2 = 0
        self.trigger = 0

        self.dual_shock_publisher = self.create_publisher(Twist, 'meerkat/cmd_vel', 10)

        self.button_publisher = self.create_publisher(Bool, 'ds4/trigger', 10)
        self.button_publisher_2 = self.create_publisher(Bool, 'ds4/trigger2', 10)

        self.dual_shock_subscription = self.create_subscription(Status,'/status',self.listener_callback,10)
        self.dual_shock_subscription  # prevent unused variable warning

        timer_period = 0.01  # seconds
        self.i = 0

        self.timer = self.create_timer(timer_period, self.timer_callback)  

        self.change_control = True

        self.linear_scale = 0.0
        self.angular_scale = 0.0
        self.trigger_controls = 0
        self.increase_linear_speed = False
        self.decrease_linear_speed = False
        self.increase_angular_speed = False
        self.decrease_angular_speed = False
        self.button_trigger = False
        self.button_trigger_2 = False

        self.read_data = False
        self.read_data_2 = False
        self.trigger_button_press_time = 0.0

        self.control_trigger_time = 0.0
        self.increase_linear_speed_time = 0.0
        self.decrease_linear_speed_time = 0.0
        self.increase_angular_speed_time = 0.0
        self.decrease_angular_speed_time = 0.0

    def listener_callback(self, msg):
        self.linear_speed = msg.axis_left_y 
        self.angular_speed1 = msg.axis_right_x 
        self.angular_speed2 = msg.axis_left_x
        self.trigger = msg.button_l1
        self.increase_linear_speed = msg.button_dpad_up
        self.decrease_linear_speed = msg.button_dpad_down
        self.increase_angular_speed = msg.button_dpad_right
        self.decrease_angular_speed = msg.button_dpad_left
        self.trigger_controls = msg.button_trackpad
        self.button_trigger = msg.button_triangle
        self.button_trigger_2 = msg.button_square

    def timer_callback(self):
        msg = Twist()
        trigger_msg = Bool()
        current_time = time()

        
        if self.trigger_controls and (current_time - self.control_trigger_time > 0.5):
            self.control_trigger_time = current_time
            self.get_logger().info(f'Changed Controls')
            self.change_control = not self.change_control
            
        if self.increase_linear_speed and self.linear_scale < 1.0 and (current_time - self.increase_linear_speed_time > 0.5):
            self.increase_linear_speed_time = current_time
            self.get_logger().info(f'Linear Speed: {self.linear_scale}')
            self.linear_scale += 0.05
        
        if self.decrease_linear_speed and self.linear_scale > 0.0 and (current_time - self.decrease_linear_speed_time > 0.5):
            self.decrease_linear_speed_time = current_time
            self.get_logger().info(f'Linear Speed: {self.linear_scale}')
            self.linear_scale -= 0.05
        
        if self.increase_angular_speed and self.angular_scale < 1.0 and (current_time - self.increase_angular_speed_time > 0.5):
            self.increase_angular_speed_time = current_time
            self.get_logger().info(f'Angular Speed: {self.angular_scale}')
            self.angular_scale += 0.05
        
        if self.decrease_angular_speed and self.angular_scale > 0.0 and (current_time - self.decrease_angular_speed_time > 0.5):
            self.decrease_angular_speed_time = current_time
            self.get_logger().info(f'Angular Speed: {self.angular_scale}')
            self.angular_scale -= 0.05
        
        if self.button_trigger and (current_time - self.trigger_button_press_time > 0.5):
            self.trigger_button_press_time = current_time
            if self.read_data:
                self.read_data = False
            else:
                self.read_data = True
            self.get_logger().info(f'BOOL: {self.read_data}')
            
        if self.button_trigger_2 and (current_time - self.trigger_button_press_time > 0.5):
            self.trigger_button_press_time = current_time
            if self.read_data_2:
                self.read_data_2 = False
            else:
                self.read_data_2 = True
            self.get_logger().info(f'BOOL: {self.read_data_2}')
        
       
      
        if self.trigger:
            msg.linear.x = float(self.linear_speed) * (self.linear_scale)
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            if self.change_control:
                msg.angular.z = float(self.angular_speed1)*(1.0) * self.angular_scale
            else:
                msg.angular.z = float(self.angular_speed2)*(1.0) * self.angular_scale
        else:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0


        self.dual_shock_publisher.publish(msg)

        trigger_msg.data = self.read_data
        self.button_publisher.publish(trigger_msg)

        trigger_msg.data = self.read_data_2
        self.button_publisher_2.publish(trigger_msg)

        #self.get_logger().info(f'Publishing Speed Motor1: {msg.linear.x} and {msg.angular.z}')
        #self.get_logger().info(f'Publishing Speed Motor1: {self.change_control}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    dualshock_subscriber = DualShockSubscriber()

    rclpy.spin(dualshock_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dualshock_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
