import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Paths to map and parameter file
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('meerkat_navigation'),
            'maps',
            'mit.yaml'),
            description="Full path to the yaml map file"
            )

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('meerkat_navigation'),
            'config',
            'nav2_params.yaml'))

    rviz_config_dir = os.path.join(
        get_package_share_directory('meerkat_navigation'),
        'rviz',
        'urdf_config.rviz')

    nav2_bringup_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    # Include sensor/driver bringup if needed
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/media/meerkat/Meerkat/aryan_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py'
        ])
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/media/meerkat/Meerkat/aryan_ws/src/meerkat_odometry/launch/odometry_data_launch.py'
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        lidar_launch,
        odometry_launch,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav2_bringup_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
