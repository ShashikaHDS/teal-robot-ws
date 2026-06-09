import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from ds4_driver_msgs.msg import Status
from geometry_msgs.msg import Twist

from time import time


class DualShockSubscriber(Node):

    def __init__(self):
        super().__init__('dualshock_ros')

        self.linear_speed = 0.0
        self.angular_speed1 = 0.0
        self.angular_speed2 = 0.0
        self.trigger = 0

        self.dual_shock_publisher = self.create_publisher(Twist, 'meerkat/cmd_vel', 10)

        self.button_publisher = self.create_publisher(Bool, 'ds4/trigger', 10)
        self.button_publisher_2 = self.create_publisher(Bool, 'ds4/trigger2', 10)

        self.dual_shock_subscription = self.create_subscription(
            Status, '/status', self.listener_callback, 10
        )

        timer_period = 0.05  # 20 Hz
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

        # --- Watchdog / Dead-man's switch ---
        self.last_status_time = 0.0
        self.watchdog_timeout = 0.3  # seconds — if no /status for this long, assume lost
        self.controller_alive = False  # True only while /status is arriving

        self.i = 0

    def listener_callback(self, msg: Status):
        """Called each time a /status message arrives from ds4_driver.
        Publishes cmd_vel IMMEDIATELY for low-latency control."""
        self.last_status_time = time()
        self.controller_alive = True

        # Read controller inputs
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

        # Publish cmd_vel immediately (low-latency path)
        if self.trigger:
            deadzone = 0.05
            lin_in = float(self.linear_speed)
            lin = lin_in if abs(lin_in) > deadzone else 0.0

            ang_raw = (float(self.angular_speed1) if self.change_control
                       else float(self.angular_speed2))
            ang = ang_raw if abs(ang_raw) > deadzone else 0.0

            self._publish_cmd_vel(lin * self.linear_scale, ang * self.angular_scale)
        else:
            self._publish_cmd_vel(0.0, 0.0)

    def _publish_cmd_vel(self, lin_x: float, ang_z: float):
        """Helper to publish a Twist."""
        cmd = Twist()
        cmd.linear.x = lin_x
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = ang_z
        self.dual_shock_publisher.publish(cmd)

    def timer_callback(self):
        """
        Runs at 20 Hz. Safety net ONLY.

        Normal driving is handled in listener_callback (immediate, low-latency).
        This timer ONLY kicks in when the controller goes silent (Bluetooth drop)
        and continuously publishes STOP until /status resumes.
        """
        current_time = time()
        trigger_msg = Bool()

        # ---- Check watchdog: is controller still sending /status? ----
        if self.last_status_time > 0.0:
            dt = current_time - self.last_status_time
            if dt > self.watchdog_timeout:
                if self.controller_alive:
                    self.get_logger().warn(
                        f'DualShock watchdog timeout ({dt:.2f}s): '
                        'no /status -- sending continuous STOP'
                    )
                self.controller_alive = False

        # ---- Safety stop: continuously publish stop when controller is lost ----
        if not self.controller_alive and self.last_status_time > 0.0:
            self._publish_cmd_vel(0.0, 0.0)

        # ---- D-pad speed adjustments (debounced) ----
        if self.trigger_controls and (current_time - self.control_trigger_time > 0.5):
            self.control_trigger_time = current_time
            self.change_control = not self.change_control
            self.get_logger().info('Changed Controls')

        if (self.increase_linear_speed and self.linear_scale < 1.0
                and (current_time - self.increase_linear_speed_time > 0.5)):
            self.increase_linear_speed_time = current_time
            self.linear_scale += 0.05
            self.get_logger().info(f'Linear Speed: {self.linear_scale:.2f}')

        if (self.decrease_linear_speed and self.linear_scale > 0.0
                and (current_time - self.decrease_linear_speed_time > 0.5)):
            self.decrease_linear_speed_time = current_time
            self.linear_scale -= 0.05
            self.get_logger().info(f'Linear Speed: {self.linear_scale:.2f}')

        if (self.increase_angular_speed and self.angular_scale < 1.0
                and (current_time - self.increase_angular_speed_time > 0.5)):
            self.increase_angular_speed_time = current_time
            self.angular_scale += 0.05
            self.get_logger().info(f'Angular Speed: {self.angular_scale:.2f}')

        if (self.decrease_angular_speed and self.angular_scale > 0.0
                and (current_time - self.decrease_angular_speed_time > 0.5)):
            self.decrease_angular_speed_time = current_time
            self.angular_scale -= 0.05
            self.get_logger().info(f'Angular Speed: {self.angular_scale:.2f}')

        # ---- Button triggers ----
        if self.button_trigger and (current_time - self.trigger_button_press_time > 0.5):
            self.trigger_button_press_time = current_time
            self.read_data = not self.read_data
            self.get_logger().info(f'BOOL: {self.read_data}')

        if self.button_trigger_2 and (current_time - self.trigger_button_press_time > 0.5):
            self.trigger_button_press_time = current_time
            self.read_data_2 = not self.read_data_2
            self.get_logger().info(f'BOOL: {self.read_data_2}')

        trigger_msg.data = self.read_data
        self.button_publisher.publish(trigger_msg)

        trigger_msg.data = self.read_data_2
        self.button_publisher_2.publish(trigger_msg)

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    dualshock_subscriber = DualShockSubscriber()
    rclpy.spin(dualshock_subscriber)
    dualshock_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
