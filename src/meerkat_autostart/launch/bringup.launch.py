"""Bringup: foundation (teleop + ZED) + GUI mode controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('meerkat_motors'),
            'launch', 'meerkat_teleop_launch.py')))

    zed = ExecuteProcess(
        cmd=['/bin/bash', '/home/teal/Downloads/lio_ws/zed_capture.sh'],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='3',
    )

    controller = Node(
        package='meerkat_autostart',
        executable='mode_controller',
        name='mode_controller',
        output='screen',
        parameters=[{
            'liosam_script': '/home/teal/Downloads/lio_ws/liosam.sh',
            'nav_package': 'meerkat_navigation',
            'nav_launch': 'meerkat_bringup.launch.py',
            'map_base_dir': '/media/teal/ssd1tb/lio_sam_maps',
            'map_resolution': 0.005,
            'status_topic': '/status',
            'save_service': '/lio_sam/save_map',
            'zed_cmd_topic': '/zed_capture/command',
            'modifier_button': 'button_r1',
            'btn_mapping': 'button_cross',
            'btn_teleop': 'button_circle',
            'btn_autonomy': 'button_triangle',
            'btn_save': 'button_share',
            'btn_record': 'button_options',
        }],
    )

    return LaunchDescription([
        teleop,
        TimerAction(period=3.0, actions=[zed]),
        TimerAction(period=2.0, actions=[controller]),
    ])
