from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():

    vectornav_launch_file = '/home/teal/Downloads/lio_ws/src/vectornav/vectornav/launch/vectornav.launch.py'

    # Include the launch file
    vectornav_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(vectornav_launch_file)
    )

    return LaunchDescription([
        vectornav_launch,
        Node(
            package='meerkat_odometry',
            executable='wheel_odometry',
            name='wheel_odometry')
        # ),
        # Node(
        #     package='ekf_node',
        #     executable='imu_data',
        #     name='vectornav_data'
        # )
    ])
