from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_ros
import launch
import os

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='meerkat_description').find('meerkat_description')
    default_model_path = os.path.join(pkg_share, 'src/description/meerkat_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    lidar2d_launch_file = "/media/meerkat/Meerkat/aryan_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py"

    #teleop_launch_file = '/media/meerkat/Meerkat/meerkat_ws/src/meerkat_motors/launch/meerkat_teleop_launch.py'

    odometry_launch_file = '/media/meerkat/Meerkat/aryan_ws/src/meerkat_odometry/launch/odometry_data_launch.py'

    bringup_dir = get_package_share_directory('meerkat_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')


    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        #'velocity_smoother'
    ]
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('cmd_vel', '/meerkat/cmd_vel'),
        ('odom', '/odometry/filtered')
    ]
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=None,
        param_rewrites=param_substitutions,
        convert_types=True
    )

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
 
    odometry_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(odometry_launch_file)
    )

    #amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("meerkat_navigation"), "maps", "skunworkslab_map.yaml"]),
        description="/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/maps/skunworkslab_map.yaml",
    )


    #amcl_params_file_arg = DeclareLaunchArgument(
    #    "amcl_params_file",
    #    default_value=PathJoinSubstitution(
    #   ),
    #    description="Full path to the ROS2 parameters file to use for the amcl node",
    #)

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
        parameters=[configured_params],
        remappings=remappings
    )
    
    # Nav2 lifecycle-managed servers
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[configured_params],
        remappings=remappings,
        output='screen'
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[configured_params],
        remappings=remappings,
        output='screen'
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[configured_params],
        output='screen'
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[configured_params],
        remappings=remappings,
        output='screen'
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[configured_params],
        remappings=remappings,
        output='screen'
    )
    oriental_ros_node = Node(
                package='meerkat_motors',
                executable='oriental_ros',
                name='oriental_ros',
                output='screen'
    )
    
    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'bond_timeout': 10.0},
            {'node_names': lifecycle_nodes},
            {'use_sim_time': use_sim_time}
        ],
        remappings=remappings
)
    nav_lifecycle_timer = TimerAction(period=10.0, actions=[nav_lifecycle_manager])

    return LaunchDescription(
        [

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'dwb_params.yaml'),
            description='/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/config/dwb_params.yaml'
            #/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/config/ak_params_working.yaml
            #'/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/config/dwb_params.yaml'
        ),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_w_replanning_and_recovery.xml'
            )
        ),

        DeclareLaunchArgument('map_subscribe_transient_local', default_value='true'),

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
            oriental_ros_node,
            rviz_node,
            #amcl_params_file_arg,
            map_file_arg,
            map_server_node,
            amcl_node,
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            nav_lifecycle_timer  

    ])