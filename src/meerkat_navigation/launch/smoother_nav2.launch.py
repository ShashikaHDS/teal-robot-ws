import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('meerkat_navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        #'waypoint_follower',  #priviously commented
        'velocity_smoother'
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel_smoothed', '/meerkat/cmd_vel'),
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

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'dwb_params.yaml'),
            description='/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/config/dwb_params.yaml'
            #/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/config/smooth_dwb_params.yaml
            #'/media/meerkat/Meerkat/aryan_ws/src/meerkat_navigation/config/dwb_params.yaml'
        ),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_to_pose_w_replanning_and_recovery.xml'  # 🔄 changed here
            )
        ),


        DeclareLaunchArgument('map_subscribe_transient_local', default_value='true'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
            arguments=[
                '--ros-args',
                '--param', 'topic_qos_overrides./map.subscription.durability:=transient_local',
                '--param', 'topic_qos_overrides./map.subscription.reliability:=reliable'
            ]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        # Node(
        #     package='nav2_waypoint_follower',
        #     executable='waypoint_follower',
        #     name='waypoint_follower',
        #     output='screen',
        #     parameters=[configured_params],
        #     remappings=remappings
        # ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[configured_params],
            remappings=remappings

        ),
        
    
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 10.0
            }]
        )
    ])

