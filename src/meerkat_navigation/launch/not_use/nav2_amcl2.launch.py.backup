from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
import launch_ros
import launch

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='meerkat_description').find('meerkat_description')
    default_model_path = os.path.join(pkg_share, 'src/description/meerkat_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    lidar2d_launch_file = "/media/meerkat/Meerkat/aryan_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py"
    odometry_launch_file = '/media/meerkat/Meerkat/aryan_ws/src/meerkat_odometry/launch/odometry_data_launch.py'

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')

    lidar2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar2d_launch_file)
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odometry_launch_file)
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[default_model_path],
        condition=UnlessCondition(gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui)
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[params_file, {'yaml_filename': map_file}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[params_file]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/meerkat/cmd_vel')]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[params_file]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'bt_navigator',
                'waypoint_follower']}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('model', default_value=default_model_path),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_file', default_value=PathJoinSubstitution([FindPackageShare('meerkat_navigation'), 'maps', 'mit.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([FindPackageShare('meerkat_navigation'), 'config', 'nav2_params.yaml'])),

        lidar2d_launch,
        odometry_launch,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        robot_localization_node,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        rviz_node
    ])

