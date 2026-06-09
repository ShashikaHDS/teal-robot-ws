from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():

    dualshock_launch_file = '/home/teal/Downloads/lio_ws/src/ds4_driver/ds4_driver/launch/demo.launch.xml'

    # Include the XML launch file
    dualshock_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(dualshock_launch_file)
    )

    return LaunchDescription([
        dualshock_launch,
        Node(
            package='meerkat_motors',
            executable='oriental_ros',
            name='oriental_ros'
        ),
        Node(
            package='meerkat_motors',
            executable='dualshock_ros',
            name='dualshock_ros'
        )
    ])
