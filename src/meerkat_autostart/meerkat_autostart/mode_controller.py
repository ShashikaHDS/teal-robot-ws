"""Meerkat mode controller: Tkinter GUI + DS4 shortcuts + subprocess manager."""
import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import rclpy
from ds4_driver_msgs.msg import Status
from lio_sam.srv import SaveMap
from rclpy.node import Node
from std_msgs.msg import String

MODE_TELEOP = 'teleop'
MODE_MAPPING = 'mapping'
MODE_AUTONOMY = 'autonomy'
MODES = (MODE_TELEOP, MODE_MAPPING, MODE_AUTONOMY)


class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')
        p = lambda k, d: self.declare_parameter(k, d).value

        self.liosam_script = p('liosam_script',
                               '/home/teal/Downloads/lio_ws/liosam.sh')
        self.nav_pkg = p('nav_package', 'meerkat_navigation')
        self.nav_launch = p('nav_launch', 'meerkat_bringup.launch.py')
        self.map_base_dir = p('map_base_dir',
                              '/media/teal/ssd1tb/lio_sam_maps')
        self.map_resolution = float(p('map_resolution', 0.005))
        self.status_topic = p('status_topic', '/status')
        self.save_service = p('save_service', '/lio_sam/save_map')
        self.zed_cmd_topic = p('zed_cmd_topic', '/zed_capture/command')

        self.mod_btn = p('modifier_button', 'button_r1')
        self.btn_mapping = p('btn_mapping', 'button_cross')
        self.btn_teleop = p('btn_teleop', 'button_circle')
        self.btn_autonomy = p('btn_autonomy', 'button_triangle')
        self.btn_save = p('btn_save', 'button_share')
        self.btn_record = p('btn_record', 'button_options')

        self.mode = MODE_TELEOP
        self.lio_proc = None
        self.nav_proc = None
        self._prev_btn = {}
        self._last_press = {}
        self._debounce = 0.25
        self.last_status = 'ready.'
        self._recording_guess = False

        self.create_subscription(Status, self.status_topic,
                                 self._on_ds4, 10)
        self.save_client = self.create_client(SaveMap, self.save_service)
        self.zed_cmd_pub = self.create_publisher(String, self.zed_cmd_topic, 10)

        self.get_logger().info(
            f'mode_controller ready (mod={self.mod_btn} '
            f'mapping={self.btn_mapping} teleop={self.btn_teleop} '
            f'autonomy={self.btn_autonomy} save={self.btn_save} '
            f'record={self.btn_record})')

    # ---------- subprocess helpers ----------
    @staticmethod
    def _start(cmd):
        return subprocess.Popen(cmd, preexec_fn=os.setsid)

    def _stop_pg(self, proc, label, timeout=15):
        if proc is None or proc.poll() is not None:
            return None
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return None
        self.get_logger().info(f'SIGINT to {label} pgid={pgid}')
        try:
            os.killpg(pgid, signal.SIGINT)
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{label} did not exit on SIGINT, escalating')
            try:
                os.killpg(pgid, signal.SIGTERM)
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return None

    # ---------- mode lifecycle ----------
    def set_mode(self, new_mode):
        if new_mode not in MODES:
            self.get_logger().warn(f'unknown mode: {new_mode}')
            return
        if new_mode == self.mode:
            return
        self.get_logger().info(f'mode {self.mode} -> {new_mode}')
        if self.lio_proc is not None:
            self.lio_proc = self._stop_pg(self.lio_proc, 'liosam')
        if self.nav_proc is not None:
            self.nav_proc = self._stop_pg(self.nav_proc, 'nav2')
        if new_mode == MODE_MAPPING:
            self.lio_proc = self._start(['bash', self.liosam_script])
        elif new_mode == MODE_AUTONOMY:
            self.nav_proc = self._start(
                ['ros2', 'launch', self.nav_pkg, self.nav_launch])
        self.mode = new_mode
        self.last_status = f'mode -> {new_mode}'

    # ---------- save map (versioning ported from savemap.py) ----------
    def _next_ver_dir(self):
        date_dir = os.path.join(self.map_base_dir,
                                datetime.now().strftime('%d%m%y'))
        ver = 1
        if os.path.isdir(date_dir):
            existing = []
            for n in os.listdir(date_dir):
                if (n.startswith('ver') and n[3:].isdigit()
                        and os.path.isdir(os.path.join(date_dir, n))):
                    existing.append(int(n[3:]))
            ver = max(existing, default=0) + 1
        out = os.path.join(date_dir, f'ver{ver}')
        os.makedirs(out, exist_ok=True)
        return out + '/'

    def save_map(self):
        if self.mode != MODE_MAPPING:
            self.last_status = 'save_map ignored (not in mapping mode)'
            self.get_logger().warn(self.last_status)
            return
        if not self.save_client.wait_for_service(timeout_sec=2.0):
            self.last_status = 'save_map: service unavailable'
            self.get_logger().error(self.last_status)
            return
        req = SaveMap.Request()
        req.resolution = self.map_resolution
        req.destination = self._next_ver_dir()
        self.last_status = f'saving -> {req.destination}'
        self.get_logger().info(self.last_status)
        fut = self.save_client.call_async(req)
        fut.add_done_callback(self._on_save_done)

    def _on_save_done(self, fut):
        try:
            res = fut.result()
            self.last_status = f'save_map: success={res.success}'
        except Exception as e:
            self.last_status = f'save_map error: {e}'
        self.get_logger().info(self.last_status)

    # ---------- ZED capture ----------
    def zed_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.zed_cmd_pub.publish(msg)
        self.last_status = f'zed: {cmd}'
        self.get_logger().info(self.last_status)

    # ---------- DS4 ----------
    def _rising(self, name, val):
        prev = self._prev_btn.get(name, 0)
        self._prev_btn[name] = val
        if val and not prev:
            now = time.monotonic()
            if now - self._last_press.get(name, 0.0) > self._debounce:
                self._last_press[name] = now
                return True
        return False

    def _on_ds4(self, msg: Status):
        watched = (self.btn_mapping, self.btn_teleop, self.btn_autonomy,
                   self.btn_save, self.btn_record)
        held = bool(getattr(msg, self.mod_btn))
        if not held:
            for b in watched:
                self._prev_btn[b] = getattr(msg, b)
            return

        if self._rising(self.btn_mapping, getattr(msg, self.btn_mapping)):
            self.set_mode(MODE_MAPPING)
        if self._rising(self.btn_teleop, getattr(msg, self.btn_teleop)):
            self.set_mode(MODE_TELEOP)
        if self._rising(self.btn_autonomy, getattr(msg, self.btn_autonomy)):
            self.set_mode(MODE_AUTONOMY)
        if self._rising(self.btn_save, getattr(msg, self.btn_save)):
            self.save_map()
        if self._rising(self.btn_record, getattr(msg, self.btn_record)):
            self._recording_guess = not self._recording_guess
            self.zed_cmd('record_start' if self._recording_guess
                         else 'record_stop')

    # ---------- shutdown ----------
    def shutdown(self):
        if self.lio_proc is not None:
            self.lio_proc = self._stop_pg(self.lio_proc, 'liosam')
        if self.nav_proc is not None:
            self.nav_proc = self._stop_pg(self.nav_proc, 'nav2')


# -------------------- GUI --------------------
HEADER_FONT = ('TkDefaultFont', 16, 'bold')
SECTION_FONT = ('TkDefaultFont', 14, 'bold')
BTN_FONT = ('TkDefaultFont', 13)
RADIO_FONT = ('TkDefaultFont', 14)
STATUS_FONT = ('TkDefaultFont', 12)


class ModeGUI:
    def __init__(self, ctrl: ModeController):
        self.ctrl = ctrl
        self.root = tk.Tk()
        self.root.title('Meerkat Control')
        self.root.attributes('-fullscreen', True)
        self._fullscreen = True

        style = ttk.Style()
        style.configure('Big.TButton', font=BTN_FONT, padding=10)
        style.configure('Big.TRadiobutton', font=RADIO_FONT, padding=4)

        # Header
        header = ttk.Frame(self.root)
        header.pack(fill='x', padx=20, pady=(20, 10))
        ttk.Label(header, text='Meerkat Control', font=HEADER_FONT
                  ).pack(side='left')
        ttk.Label(header,
                  text='F11/Esc: toggle fullscreen   •   M: minimise (view map)   •   Q: quit',
                  font=STATUS_FONT, foreground='#666'
                  ).pack(side='right')

        # Main grid: left = mode + map, right = capture
        body = ttk.Frame(self.root)
        body.pack(fill='both', expand=True, padx=20, pady=10)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky='nsew', padx=(10, 0))

        # ----- LEFT: Operating Mode + Map -----
        ttk.Label(left, text='Operating Mode', font=SECTION_FONT
                  ).pack(anchor='w', pady=(0, 8))
        self.mode_var = tk.StringVar(value=ctrl.mode)
        for label, val in [('Teleop', MODE_TELEOP),
                           ('Mapping', MODE_MAPPING),
                           ('Autonomous Navigation', MODE_AUTONOMY)]:
            ttk.Radiobutton(left, text=label, value=val,
                            variable=self.mode_var,
                            style='Big.TRadiobutton',
                            command=lambda v=val: ctrl.set_mode(v)
                            ).pack(anchor='w', pady=4)

        ttk.Separator(left).pack(fill='x', pady=14)
        ttk.Label(left, text='Map', font=SECTION_FONT
                  ).pack(anchor='w', pady=(0, 8))
        self.btn_save = ttk.Button(left, text='Save Map',
                                   style='Big.TButton',
                                   command=ctrl.save_map)
        self.btn_save.pack(fill='x', pady=4)
        ttk.Button(left, text='Minimise (view map)',
                   style='Big.TButton',
                   command=self._minimise
                   ).pack(fill='x', pady=4)

        # ----- RIGHT: ZED Capture -----
        ttk.Label(right, text='ZED Capture', font=SECTION_FONT
                  ).pack(anchor='w', pady=(0, 8))
        ttk.Button(right, text='Capture Image',
                   style='Big.TButton',
                   command=lambda: ctrl.zed_cmd('image')
                   ).pack(fill='x', pady=4)

        self.auto_on = False
        self.btn_auto = ttk.Button(right, text='Auto-capture: OFF',
                                   style='Big.TButton',
                                   command=self._toggle_auto)
        self.btn_auto.pack(fill='x', pady=4)

        row = ttk.Frame(right)
        row.pack(fill='x', pady=4)
        ttk.Label(row, text='Interval (s):', font=BTN_FONT
                  ).pack(side='left', padx=(0, 6))
        self.interval_var = tk.StringVar(value='1.0')
        ttk.Entry(row, textvariable=self.interval_var, width=8,
                  font=BTN_FONT).pack(side='left', padx=4)
        ttk.Button(row, text='Set', style='Big.TButton',
                   command=lambda: ctrl.zed_cmd(
                       f'interval={self.interval_var.get()}')
                   ).pack(side='left', padx=4)

        self.rec_on = False
        self.btn_rec = ttk.Button(right, text='Recording: OFF',
                                  style='Big.TButton',
                                  command=self._toggle_rec)
        self.btn_rec.pack(fill='x', pady=4)

        # ----- BOTTOM: Status + Quit -----
        bottom = ttk.Frame(self.root)
        bottom.pack(fill='x', padx=20, pady=(10, 20))
        ttk.Separator(bottom).pack(fill='x', pady=(0, 8))
        ttk.Label(bottom, text='Status', font=SECTION_FONT
                  ).pack(anchor='w')
        self.status_var = tk.StringVar(value='ready.')
        ttk.Label(bottom, textvariable=self.status_var,
                  wraplength=1400, justify='left',
                  font=STATUS_FONT
                  ).pack(fill='x', pady=4)
        ttk.Button(bottom, text='Quit (Q)', style='Big.TButton',
                   command=self._quit).pack(anchor='e', pady=(8, 0))

        # Key bindings
        self.root.bind('<KeyPress-q>', lambda e: self._quit())
        self.root.bind('<KeyPress-Q>', lambda e: self._quit())
        self.root.bind('<F11>', lambda e: self._toggle_fullscreen())
        self.root.bind('<Escape>', lambda e: self._toggle_fullscreen())
        self.root.bind('<KeyPress-m>', lambda e: self._minimise())
        self.root.bind('<KeyPress-M>', lambda e: self._minimise())
        self.root.protocol('WM_DELETE_WINDOW', self._quit)
        self._tick()

    def _toggle_auto(self):
        self.auto_on = not self.auto_on
        self.btn_auto.config(
            text=f'Auto-capture: {"ON" if self.auto_on else "OFF"}')
        self.ctrl.zed_cmd('auto_on' if self.auto_on else 'auto_off')

    def _toggle_rec(self):
        self.rec_on = not self.rec_on
        self.btn_rec.config(
            text=f'Recording: {"ON" if self.rec_on else "OFF"}')
        self.ctrl._recording_guess = self.rec_on
        self.ctrl.zed_cmd('record_start' if self.rec_on else 'record_stop')

    def _toggle_fullscreen(self):
        self._fullscreen = not self._fullscreen
        self.root.attributes('-fullscreen', self._fullscreen)

    def _minimise(self):
        # leave fullscreen so iconify works on most WMs, then iconify
        if self._fullscreen:
            self._fullscreen = False
            self.root.attributes('-fullscreen', False)
        self.root.iconify()

    def _quit(self):
        """Stop managed subprocesses and SIGINT the parent ros2 launch."""
        self.ctrl.get_logger().info('Q pressed — shutting down bringup')
        self.ctrl.last_status = 'shutting down…'
        self.ctrl.shutdown()
        try:
            os.kill(os.getppid(), signal.SIGINT)
        except (ProcessLookupError, PermissionError) as e:
            self.ctrl.get_logger().warn(f'parent signal failed: {e}')
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def _tick(self):
        if self.mode_var.get() != self.ctrl.mode:
            self.mode_var.set(self.ctrl.mode)
        save_state = ('!disabled',) if self.ctrl.mode == MODE_MAPPING else ('disabled',)
        self.btn_save.state(save_state)

        # Reflect external recording state (DS4 shortcut)
        if self.rec_on != self.ctrl._recording_guess:
            self.rec_on = self.ctrl._recording_guess
            self.btn_rec.config(
                text=f'Recording: {"ON" if self.rec_on else "OFF"}')

        lp = ('running' if (self.ctrl.lio_proc and
                            self.ctrl.lio_proc.poll() is None) else '–')
        np_ = ('running' if (self.ctrl.nav_proc and
                             self.ctrl.nav_proc.poll() is None) else '–')
        self.status_var.set(
            f'Mode: {self.ctrl.mode}  •  LIO-SAM: {lp}  •  Nav2: {np_}\n'
            f'Last: {self.ctrl.last_status}')
        self.root.after(500, self._tick)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    ctrl = ModeController()
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(ctrl), daemon=True)
    spin_thread.start()
    try:
        ModeGUI(ctrl).run()
    finally:
        ctrl.shutdown()
        ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
