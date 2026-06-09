from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import launch
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='meerkat_description').find('meerkat_description')
    default_model_path = os.path.join(pkg_share, 'src/description/meerkat_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # Hardware launch files
    lidar2d_launch_file = "/media/meerkat/Meerkat/aryan_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py"
    teleop_launch_file = '/media/meerkat/Meerkat/aryan_ws/src/meerkat_motors/launch/meerkat_teleop_launch.py'
    odometry_launch_file = '/media/meerkat/Meerkat/aryan_ws/src/meerkat_odometry/launch/odometry_data_launch.py'
    
    # Nav2 configuration files
    nav2_params_file = LaunchConfiguration('params_file')
    nav2_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare("meerkat_navigation"), "config", "nav2_params.yaml"]),
        description='Full path to the ROS2 parameters file to use for Nav2'
    )
    
    # Map configuration
    map_file = LaunchConfiguration("map_file")
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("meerkat_navigation"), "maps", "mit.yaml"]),
        description="Full path to the yaml map file",
    )
    
    # Launch configuration arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Use composed nodes if true'
    )
    
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes'
    )
    
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Enable manual teleop control (for testing/emergency)'
    )

    # Robot state and joint state publishers
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Robot localization (EKF)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )

    # Map server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': map_file}
        ]
    )

    # AMCL
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params_file]
    )

    # Nav2 Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/cmd_vel', '/meerkat/cmd_vel')  # Remap to your robot's cmd_vel topic
        ]
    )

    # Nav2 Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Nav2 Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Nav2 BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Nav2 Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Nav2 Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/meerkat/cmd_vel')
        ]
    )

    # Nav2 Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Lifecycle manager for localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'smoother_server'
            ]}
        ]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/meerkat/.rviz2/meerkat_amcl.rviz'],
    )

    # Include hardware launch files
    lidar2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar2d_launch_file)
    )
    
    # Optional teleop launch (only if enabled)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_file),
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_teleop'))
    )
    
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odometry_launch_file)
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(name='gui', default_value='True',
                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                            description='Absolute path to rviz config file'),
        use_sim_time_arg,
        autostart_arg,
        use_composition_arg,
        use_respawn_arg,
        enable_teleop_arg,
        nav2_params_file_arg,
        map_file_arg,
        
        # Hardware launches
        teleop_launch,
        odometry_launch,
        lidar2d_launch,
        
        # Robot state
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        robot_localization_node,
        
        # Navigation nodes
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        smoother_server_node,
        
        # Lifecycle managers
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        
        # Visualization
        rviz_node
    ])
