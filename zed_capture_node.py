#!/usr/bin/env python3
"""
ZED 2i All-in-One: Image Capture + Video Recording

Controls:
  TRIANGLE (short) / X key  = Single image capture
  TRIANGLE (hold 1.5s) / A  = Toggle auto-capture (every 1s)
  R key                      = Toggle video recording
  Q key                      = Quit (auto-stops recording)

Saves to:
  Images: /media/teal/ssd1tb/zed2_images/<DDMMYYYY>/rgb/
  Videos: /media/teal/ssd1tb/zed2_videos/<DDMMYYYY>/recording_<NNN>.mp4

Requirements:
  - zed-ros2-wrapper
  - cv_bridge
  - DS4 controller (optional)

Usage:
  python3 zed_capture_node.py
"""

import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2


class ZedCaptureNode(Node):
    """All-in-one ZED 2i image capture and video recording node."""

    def __init__(self):
        super().__init__('zed_capture_node')

        # ── Parameters ──────────────────────────────────────────────────
        today = datetime.now().strftime('%d%m%Y')

        self.declare_parameter('image_base_dir', '/media/teal/ssd1tb/zed2_images')
        self.declare_parameter('video_base_dir', '/media/teal/ssd1tb/zed2_videos')
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('trigger_topic', '/ds4/trigger')
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('save_depth', False)
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('debounce_sec', 0.5)
        self.declare_parameter('preview_width', 1280)
        self.declare_parameter('preview_height', 720)
        self.declare_parameter('auto_interval_sec', 1.0)
        self.declare_parameter('long_press_sec', 1.5)
        self.declare_parameter('video_fps', 30.0)
        self.declare_parameter('video_codec', 'mp4v')

        self.image_base_dir = self.get_parameter('image_base_dir').value
        self.video_base_dir = self.get_parameter('video_base_dir').value
        self.image_topic = self.get_parameter('image_topic').value
        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.image_format = self.get_parameter('image_format').value
        self.save_depth = self.get_parameter('save_depth').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.debounce_sec = self.get_parameter('debounce_sec').value
        self.preview_w = self.get_parameter('preview_width').value
        self.preview_h = self.get_parameter('preview_height').value
        self.auto_interval = self.get_parameter('auto_interval_sec').value
        self.long_press_sec = self.get_parameter('long_press_sec').value
        self.video_fps = self.get_parameter('video_fps').value
        self.video_codec = self.get_parameter('video_codec').value

        # ── Directories ─────────────────────────────────────────────────
        # Images
        self.image_dir = os.path.join(self.image_base_dir, today)
        self.rgb_dir = os.path.join(self.image_dir, 'rgb')
        self.depth_dir = os.path.join(self.image_dir, 'depth')
        os.makedirs(self.rgb_dir, exist_ok=True)
        if self.save_depth:
            os.makedirs(self.depth_dir, exist_ok=True)

        # Videos
        self.video_dir = os.path.join(self.video_base_dir, today)
        os.makedirs(self.video_dir, exist_ok=True)
        self.video_num = self._find_next_video()

        # ── State ───────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_rgb_cv = None
        self.latest_depth = None

        # Image capture state
        self.capture_count = 0
        self.last_capture_time = 0.0
        self.flash_until = 0.0
        self.flash_text = ''
        self.FLASH_DURATION = 0.5

        # Auto-capture state
        self.auto_capture = False
        self.last_auto_capture_time = 0.0

        # DS4 trigger state
        self.prev_button_state = False
        self.trigger_press_start = 0.0
        self.trigger_held = False
        self.long_press_triggered = False

        # Video recording state
        self.recording = False
        self.video_writer = None
        self.video_path = ''
        self.video_frame_count = 0
        self.video_start_time = 0.0

        # ── OpenCV Preview Window ─────────────────────────────────────
        self.window_name = 'ZED 2i Capture + Record'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.preview_w, self.preview_h)

        # ── QoS ─────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ─────────────────────────────────────────────────
        self.rgb_sub = self.create_subscription(
            Image, self.image_topic, self.rgb_callback, sensor_qos
        )

        if self.save_depth:
            self.depth_sub = self.create_subscription(
                Image, self.depth_topic, self.depth_callback, sensor_qos
            )

        self.trigger_sub = self.create_subscription(
            Bool, self.trigger_topic, self.trigger_callback, 10
        )

        self.cmd_sub = self.create_subscription(
            String, '/zed_capture/command', self._on_command, 10
        )

        # ── Timer for preview update (~30 FPS) ─────────────────────────
        self.display_timer = self.create_timer(1.0 / 30.0, self.update_preview)

        # ── Startup info ────────────────────────────────────────────────
        self.get_logger().info('=' * 55)
        self.get_logger().info('  ZED 2i Capture + Record')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'  Images -> {self.rgb_dir}')
        self.get_logger().info(f'  Videos -> {self.video_dir}')
        self.get_logger().info('-' * 55)
        self.get_logger().info('  TRIANGLE (short) / X  = Single capture')
        self.get_logger().info('  TRIANGLE (hold)  / A  = Toggle auto-capture')
        self.get_logger().info('  R                     = Toggle video recording')
        self.get_logger().info('  Q                     = Quit')
        self.get_logger().info('=' * 55)

    # ── Helpers ─────────────────────────────────────────────────────────

    def _find_next_video(self):
        """Find the next video recording number."""
        existing = []
        if os.path.exists(self.video_dir):
            for name in os.listdir(self.video_dir):
                if name.startswith('recording_') and name.endswith('.mp4'):
                    try:
                        num = int(name.replace('recording_', '').replace('.mp4', ''))
                        existing.append(num)
                    except ValueError:
                        pass
        return max(existing, default=0) + 1

    # ── Callbacks ───────────────────────────────────────────────────────

    def rgb_callback(self, msg: Image):
        """Cache the latest RGB frame and write to video if recording."""
        try:
            self.latest_rgb_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}', throttle_duration_sec=5.0)
            return

        # Write to video at full camera rate
        if self.recording and self.video_writer:
            self.video_writer.write(self.latest_rgb_cv)
            self.video_frame_count += 1

    def depth_callback(self, msg: Image):
        """Cache the latest depth frame."""
        self.latest_depth = msg

    def _on_command(self, msg: String):
        """Handle commands from /zed_capture/command (from GUI / orchestrator)."""
        cmd = msg.data.strip().lower()
        if cmd == 'image':
            self.capture_image()
        elif cmd == 'auto_on' and not self.auto_capture:
            self._toggle_auto_capture()
        elif cmd == 'auto_off' and self.auto_capture:
            self._toggle_auto_capture()
        elif cmd == 'record_start' and not self.recording:
            self.toggle_recording()
        elif cmd == 'record_stop' and self.recording:
            self.toggle_recording()
        elif cmd.startswith('interval='):
            try:
                self.auto_interval = float(cmd.split('=', 1)[1])
                self.get_logger().info(
                    f'auto_interval -> {self.auto_interval}s')
            except ValueError:
                self.get_logger().warn(f'bad interval command: {cmd}')
        else:
            self.get_logger().warn(f'unknown zed command: {cmd}')

    def trigger_callback(self, msg: Bool):
        """Handle DS4 trigger — short press = capture, long press = toggle auto."""
        current_state = msg.data
        now = time.time()

        if current_state and not self.prev_button_state:
            self.trigger_press_start = now
            self.trigger_held = True
            self.long_press_triggered = False

        elif current_state and self.prev_button_state:
            if self.trigger_held and not self.long_press_triggered:
                if (now - self.trigger_press_start) >= self.long_press_sec:
                    self._toggle_auto_capture()
                    self.long_press_triggered = True

        elif not current_state and self.prev_button_state:
            if self.trigger_held and not self.long_press_triggered:
                if (now - self.last_capture_time) >= self.debounce_sec:
                    self.capture_image()
                    self.last_capture_time = now
            self.trigger_held = False

        self.prev_button_state = current_state

    # ── Auto-Capture ────────────────────────────────────────────────────

    def _toggle_auto_capture(self):
        self.auto_capture = not self.auto_capture
        if self.auto_capture:
            self.last_auto_capture_time = time.time()
            self.get_logger().info(f'AUTO CAPTURE ON — every {self.auto_interval}s')
        else:
            self.get_logger().info('AUTO CAPTURE OFF')

    # ── Video Recording ─────────────────────────────────────────────────

    def toggle_recording(self):
        """Start or stop video recording."""
        if not self.recording:
            if self.latest_rgb_cv is None:
                self.get_logger().warn('No camera feed — cannot record.')
                return
            self.video_path = os.path.join(
                self.video_dir, f'recording_{self.video_num:03d}.mp4')
            h, w = self.latest_rgb_cv.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
            self.video_writer = cv2.VideoWriter(
                self.video_path, fourcc, self.video_fps, (w, h))
            self.video_frame_count = 0
            self.video_start_time = time.time()
            self.recording = True
            self.get_logger().info(f'REC STARTED -> {self.video_path}')
        else:
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            duration = time.time() - self.video_start_time
            self.get_logger().info(
                f'REC STOPPED — {self.video_frame_count} frames, '
                f'{duration:.1f}s -> {self.video_path}')
            self.recording = False
            self.video_num += 1

    # ── Live Preview ────────────────────────────────────────────────────

    def update_preview(self):
        """Update preview window at ~30 FPS."""
        now = time.time()

        # Auto-capture tick
        if self.auto_capture and self.latest_rgb_cv is not None:
            if (now - self.last_auto_capture_time) >= self.auto_interval:
                self.capture_image()
                self.last_auto_capture_time = now

        if self.latest_rgb_cv is not None:
            display = self.latest_rgb_cv.copy()

            # ── Flash overlay ──
            if now < self.flash_until:
                overlay = display.copy()
                cv2.rectangle(overlay, (0, 0),
                              (display.shape[1], display.shape[0]),
                              (0, 255, 0), -1)
                alpha = 0.3 * ((self.flash_until - now) / self.FLASH_DURATION)
                cv2.addWeighted(overlay, alpha, display, 1 - alpha, 0, display)
                text = self.flash_text
                font = cv2.FONT_HERSHEY_SIMPLEX
                (tw, th), _ = cv2.getTextSize(text, font, 1.5, 3)
                cx = (display.shape[1] - tw) // 2
                cy = (display.shape[0] + th) // 2
                cv2.putText(display, text, (cx, cy), font, 1.5,
                            (255, 255, 255), 3, cv2.LINE_AA)

            # ── Top HUD: capture info ──
            mode = 'AUTO' if self.auto_capture else 'MANUAL'
            mode_color = (0, 0, 255) if self.auto_capture else (0, 255, 0)
            hud = f'[{mode}] Photos: {self.capture_count}  |  X=Snap  A=Auto  R=Rec  Q=Quit'
            cv2.putText(display, hud, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 2, cv2.LINE_AA)

            # Auto-capture pulsing dot
            if self.auto_capture:
                pulse = int(127 + 128 * np.sin(now * 4))
                cv2.circle(display, (display.shape[1] - 30, 30), 12, (0, 0, pulse), -1)
                cv2.putText(display, f'Auto: {self.auto_interval}s',
                            (display.shape[1] - 200, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            # ── Bottom HUD: video recording info ──
            if self.recording:
                duration = now - self.video_start_time
                mins = int(duration // 60)
                secs = int(duration % 60)
                # Pulsing red REC dot
                pulse = int(127 + 128 * np.sin(now * 4))
                h_img = display.shape[0]
                cv2.circle(display, (20, h_img - 25), 10, (0, 0, pulse), -1)
                rec_text = f'REC {mins:02d}:{secs:02d}  |  Frames: {self.video_frame_count}  |  recording_{self.video_num:03d}.mp4'
                cv2.putText(display, rec_text, (40, h_img - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                h_img = display.shape[0]
                rec_text = f'REC OFF  |  Next: recording_{self.video_num:03d}.mp4'
                cv2.putText(display, rec_text, (10, h_img - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1, cv2.LINE_AA)

            cv2.imshow(self.window_name, display)
        else:
            waiting = np.zeros((self.preview_h, self.preview_w, 3), dtype=np.uint8)
            text = 'Waiting for ZED camera feed...'
            font = cv2.FONT_HERSHEY_SIMPLEX
            (tw, th), _ = cv2.getTextSize(text, font, 1.0, 2)
            cx = (self.preview_w - tw) // 2
            cy = (self.preview_h + th) // 2
            cv2.putText(waiting, text, (cx, cy), font, 1.0,
                        (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow(self.window_name, waiting)

        # ── Keyboard input ──
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            if self.recording:
                self.toggle_recording()
            self.get_logger().info('Quit key pressed — shutting down.')
            rclpy.shutdown()
        elif key == ord('x'):
            if (time.time() - self.last_capture_time) >= self.debounce_sec:
                self.capture_image()
                self.last_capture_time = time.time()
        elif key == ord('a'):
            self._toggle_auto_capture()
        elif key == ord('r'):
            self.toggle_recording()

    # ── Image Capture ───────────────────────────────────────────────────

    def capture_image(self):
        """Save a single RGB (and optionally depth) image."""
        if self.latest_rgb_cv is None:
            self.get_logger().warn('No RGB image received yet — cannot capture.')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.capture_count += 1

        try:
            rgb_filename = f'capture_{self.capture_count:04d}_{timestamp}.{self.image_format}'
            rgb_path = os.path.join(self.rgb_dir, rgb_filename)
            cv2.imwrite(rgb_path, self.latest_rgb_cv)
            self.get_logger().info(f'[{self.capture_count}] Saved RGB -> {rgb_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save RGB: {e}')
            return

        if self.save_depth and self.latest_depth is not None:
            try:
                cv_depth = self.bridge.imgmsg_to_cv2(
                    self.latest_depth, desired_encoding='passthrough')
                depth_filename = f'depth_{self.capture_count:04d}_{timestamp}.png'
                depth_path = os.path.join(self.depth_dir, depth_filename)
                if cv_depth.dtype.name == 'float32':
                    depth_mm = (cv_depth * 1000.0).clip(0, 65535).astype(np.uint16)
                    cv2.imwrite(depth_path, depth_mm)
                else:
                    cv2.imwrite(depth_path, cv_depth)
                self.get_logger().info(f'[{self.capture_count}] Saved Depth -> {depth_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to save depth: {e}')

        self.flash_until = time.time() + self.FLASH_DURATION
        self.flash_text = f'CAPTURED #{self.capture_count}'

    # ── Cleanup ─────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info(f'Video saved: {self.video_path}')
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.recording:
            node.toggle_recording()
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()