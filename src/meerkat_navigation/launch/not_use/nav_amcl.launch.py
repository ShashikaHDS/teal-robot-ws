from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_ros
import launch
import os
from launch.actions import TimerAction


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='meerkat_description').find('meerkat_description')
    default_model_path = os.path.join(pkg_share, 'src/description/meerkat_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    lidar2d_launch_file = "/media/meerkat/Meerkat/aryan_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py"

    #teleop_launch_file = '/media/meerkat/Meerkat/meerkat_ws/src/meerkat_motors/launch/meerkat_teleop_launch.py'

    odometry_launch_file = '/media/meerkat/Meerkat/aryan_ws/src/meerkat_odometry/launch/odometry_data_launch.py'
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )



    # Include the XML launch file
    lidar2d_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(lidar2d_launch_file)
    )
    
    #teleop_launch = IncludeLaunchDescription(
    #    AnyLaunchDescriptionSource(teleop_launch_file)
   # )
    
    odometry_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(odometry_launch_file)
    )

    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")

    # ************************** Change the map name in the below section ************************************ #
    
    # 1. Declare the LaunchConfiguration variable
    map_file = LaunchConfiguration("map_file")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("meerkat_navigation"), "maps", "skunworkslab_map.yaml"]),
        description="/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/maps/skunworkslab_map.yaml",
    )


    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("meerkat_navigation"), "config", "amcl.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{
            'yaml_filename': map_file,
            'topic_qos_overrides': {
            '/map': {
                'durability': 'transient_local',
                'reliability': 'reliable'
            }
            }
        }],
)

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_params_file],
        remappings=[('/odom', '/odometry/filtered')]

    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server',
        parameters=[LaunchConfiguration('nav2_params_file')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server',
        parameters=[LaunchConfiguration('nav2_params_file')],
        output='screen'
    )

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server',
        parameters=[LaunchConfiguration('nav2_params_file')],
        output='screen'
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator',
        parameters=[LaunchConfiguration('nav2_params_file')],
        output='screen'
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[LaunchConfiguration('nav2_params_file')],
        output='screen'
    )

    # Single lifecycle manager for *all* Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    # Optional: Delay lifecycle manager until map & amcl warm up
    lifecycle_timer = TimerAction(period=5.0, actions=[lifecycle_manager])
    return LaunchDescription(
        [
            #teleop_launch,
            odometry_launch,
            launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                                description='Flag to enable joint_state_publisher_gui'),
            launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                                description='Absolute path to robot urdf file'),
            launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                                description='Absolute path to rviz config file'),
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            robot_localization_node,
            lidar2d_launch,
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', '/home/meerkat/.rviz2/meerkat_amcl.rviz'],
            ),
            Node(
                package='meerkat_motors',
                executable='oriental_ros',
                name='oriental_ros',
                output='screen'
            ),
            amcl_params_file_arg,
            map_file_arg,
            map_server_node,
            nav_manager,
            amcl_node
        ]
    )

