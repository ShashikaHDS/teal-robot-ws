
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    # To Publish static transform between the following frames: 
    # base_link -> os_sensor
    # base_link -> vectornav

    base_to_os_sensor_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_os_sensor_tf_publisher',
        arguments=[
            '--x', '0.145', '--y', '0.0', '--z', '0.31',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'os_sensor'
        ],
        output='screen'
    )
    base_to_vectornav_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_vectornav_tf_publisher',
        arguments=[
            '--x', '0.08', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'vectornav'
        ],
        output='screen'
    )

    #Including the launch file for the os_sensor 
    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/teal/Downloads/lio_ws/src/ouster-ros/ouster-ros/launch/driver.launch.py'])
    )
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/teal/Downloads/lio_ws/src/vectornav/vectornav/launch/vectornav.launch.py'])
    )

    return launch.LaunchDescription([
        base_to_os_sensor_tf_node,
        base_to_vectornav_tf_node,
        ouster_launch,
        vectornav_launch 
    ])
