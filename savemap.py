#!/usr/bin/env python3
"""
LIO-SAM Map Saver — Press D to save map with auto-versioning.
No cv2/numpy dependency — runs in base Python environment.

Saves to: /media/teal/ssd1tb/lio_sam_maps/<DDMMYY>/ver<N>/
Each D press increments the version number.

Usage:
  python3 lio_sam_save_map.py
  python3 lio_sam_save_map.py --ros-args -p base_dir:=/custom/path -p resolution:=0.1
"""

import os
import sys
import tty
import termios
import select
from datetime import datetime

import rclpy
from rclpy.node import Node
from lio_sam.srv import SaveMap


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')

        self.declare_parameter('base_dir', '/media/teal/ssd1tb/lio_sam_maps')
        #self.declare_parameter('base_dir', '/media/teal/ssd1tb/lio_sam_maps')

       
        self.declare_parameter('resolution', 0.005)

        self.base_dir = self.get_parameter('base_dir').value
        self.resolution = self.get_parameter('resolution').value

        # Auto-generate date folder
        self.date_str = datetime.now().strftime('%d%m%y')
        self.date_dir = os.path.join(self.base_dir, self.date_str)

        # Find the next version number
        self.version = self._find_next_version()

        # Service client
        self.client = self.create_client(SaveMap, '/lio_sam/save_map')

        # State
        self.saving = False

        # Terminal raw mode for keypress detection
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Timer to poll keyboard
        self.key_timer = self.create_timer(0.05, self.check_keyboard)

        self._print_banner()

    def _find_next_version(self):
        """Scan existing folders to find the next version number."""
        if not os.path.exists(self.date_dir):
            return 1
        existing = []
        for name in os.listdir(self.date_dir):
            if name.startswith('ver') and os.path.isdir(os.path.join(self.date_dir, name)):
                try:
                    num = int(name[3:])
                    existing.append(num)
                except ValueError:
                    pass
        return max(existing, default=0) + 1

    def _print_banner(self):
        print('\n' + '=' * 50)
        print('  LIO-SAM Map Saver')
        print('=' * 50)
        print(f'  Date:        {self.date_str}')
        print(f'  Save dir:    {self.date_dir}')
        print(f'  Next save:   ver{self.version}')
        print(f'  Resolution:  {self.resolution}')
        print('-' * 50)
        print('  [D] = Save map   [Q] = Quit')
        print('=' * 50 + '\n')

    def check_keyboard(self):
        """Poll for keypress without blocking."""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            if key == 'd':
                self.save_map()
            elif key == 'q':
                self.get_logger().info('Quit key pressed — shutting down.')
                rclpy.shutdown()

    def save_map(self):
        """Call the LIO-SAM save_map service."""
        if self.saving:
            self.get_logger().warn('Already saving — please wait.')
            return

        # Create versioned directory
        ver_dir = os.path.join(self.date_dir, f'ver{self.version}')
        os.makedirs(ver_dir, exist_ok=True)

        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('LIO-SAM save_map service not available!')
            return

        self.saving = True
        request = SaveMap.Request()
        request.resolution = self.resolution
        request.destination = ver_dir + '/'

        self.get_logger().info(f'Saving map ver{self.version} -> {ver_dir}')

        future = self.client.call_async(request)
        future.add_done_callback(self._save_done)

    def _save_done(self, future):
        """Callback when save completes."""
        try:
            result = future.result()
            self.get_logger().info(
                f'Map saved! ver{self.version} -> {self.date_dir}/ver{self.version}/')
            self.version += 1
            self.get_logger().info(f'Next version: ver{self.version}')
        except Exception as e:
            self.get_logger().error(f'Save failed: {e}')
        finally:
            self.saving = False

    def destroy_node(self):
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()