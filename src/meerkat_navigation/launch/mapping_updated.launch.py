import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = launch.substitutions.LaunchConfiguration('pkg_share')
    pkg_share_path = get_package_share_directory('meerkat_description')
    default_model_path = os.path.join(pkg_share_path, 'src/description/meerkat_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share_path, 'rviz/urdf_config.rviz')
    
    

    lidar2d_launch_file = "/home/teal/Downloads/lio_ws/src/sick_scan_xd-develop/launch/sick_tim_5xx.launch.py"
    odometry_launch_file = "/home/teal/Downloads/lio_ws/src/meerkat_odometry/launch/odometry_data_launch.py"
    slam_launch_file     = "/opt/ros/humble/share/slam_toolbox/launch/online_async_launch.py"

    return LaunchDescription([

        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('model', default_value=default_model_path),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[default_model_path],
            parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share_path, 'config/ekf.yaml')]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/meerkat/.rviz2/meerkat_slam.rviz']
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(lidar2d_launch_file)
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(odometry_launch_file)
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(slam_launch_file)
        ),

        # Replacing teleop with motor node
        Node(
            package='meerkat_motors',
            executable='oriental_ros',
            name='oriental_ros',
            output='screen')
            
           
        
    ])

