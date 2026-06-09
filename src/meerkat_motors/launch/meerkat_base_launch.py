import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource



def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='meerkat_description').find('meerkat_description')
    default_model_path = os.path.join(pkg_share, 'src/description/handheld.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # Paths to other launch files
    imu_launch_file = "/home/teal/Downloads/lio_ws/src/vectornav/vectornav/launch/vectornav.launch.py"
    
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
        arguments=['-d', '/home/meerkat/.rviz2/odom_tf.rviz'],
    )


    # Include the XML launch file
    #lidar2d_launch = IncludeLaunchDescription(
     #   AnyLaunchDescriptionSource(lidar2d_launch_file)
    #)

    imu_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(imu_launch_file)
    )


    return launch.LaunchDescription([
        imu_launch,
        #odometry_launch,
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                           description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                           description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                           description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=['/media/meerkat/Meerkat/osprey_ws/src/robot_localization/config/ekf.yaml']
        # ),
        # Node(
        #     package='meerkat_odometry',
        #     executable='odom_transform',
        #     name='odom_transform',
        #     output='screen'
        # ),
        #rviz_node
    ])
