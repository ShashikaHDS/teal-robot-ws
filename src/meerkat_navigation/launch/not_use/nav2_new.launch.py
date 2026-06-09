import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from glob import glob

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gui = LaunchConfiguration('gui', default='false')

    pkg_share = get_package_share_directory('meerkat_navigation')
    model_path = os.path.join(get_package_share_directory('meerkat_description'), 'src/description/meerkat_description.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_share, 'maps', 'fab_lab.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))

    nav2_bringup_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Include driver bringups
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/media/meerkat/Meerkat/aryan_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py']
        )
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/media/meerkat/Meerkat/aryan_ws/src/meerkat_odometry/launch/odometry_data_launch.py']
        )
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]), 'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[model_path],
        parameters=[{'robot_description': Command(['xacro ', model_path])}],
        condition=UnlessCondition(gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui)
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('model', default_value=model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument('map', default_value=map_dir, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=param_dir, description='Full path to param file to load'),

        lidar_launch,
        odometry_launch,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/bringup_launch.py']),
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
            arguments=['-d', '/home/meerkat/.rviz2/meerkat_amcl.rviz'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
