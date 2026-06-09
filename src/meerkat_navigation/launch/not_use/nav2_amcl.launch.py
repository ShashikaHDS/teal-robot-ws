from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
import os
import launch_ros
import launch

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='meerkat_description').find('meerkat_description')
    default_model_path = os.path.join(pkg_share, 'src/description/meerkat_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    lidar2d_launch_file = "/media/meerkat/Meerkat/aryan_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py"
    odometry_launch_file = '/media/meerkat/Meerkat/aryan_ws/src/meerkat_odometry/launch/odometry_data_launch.py'

    model_arg = DeclareLaunchArgument('model', default_value=default_model_path)
    gui_arg = DeclareLaunchArgument('gui', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('meerkat_navigation'), 'maps', 'fab_lab.yaml']),
        description='Full path to map file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('meerkat_navigation'), 'config', 'nav2_params.yaml']),
        description='Full path to nav2_params.yaml'
    )

    lidar2d_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(lidar2d_launch_file)
    )

    odometry_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(odometry_launch_file)
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[default_model_path],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )

    nav2_bringup_dir = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch')

    nav2_stack = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/meerkat/.rviz2/meerkat_amcl.rviz']
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        use_sim_time_arg,
        map_file_arg,
        params_file_arg,
        lidar2d_launch,
        odometry_launch,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        robot_localization_node,
        nav2_stack,
        rviz_node
    ])

