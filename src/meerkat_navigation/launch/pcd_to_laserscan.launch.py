from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
            remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
            parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 2.0, 'cloud_size': 500}],
            name='cloud_publisher'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'map', '--child-frame-id', 'cloud'
        #     ]
        # ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/cloud']),
            #             ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            remappings=[
        # INPUT: your real point cloud topic
                ('cloud_in', '/ouster/points'),
                # OUTPUT: where you want the 2D scan to go
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'os_sensor',
                'transform_tolerance': 0.01,
                'min_height': 0.72,
                'max_height': 0.78,
                'angle_min': -2.3561, #-1.5708,  # -M_PI/2
                'angle_max': 2.3561,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])