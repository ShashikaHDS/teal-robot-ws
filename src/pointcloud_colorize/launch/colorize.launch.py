from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('cloud_topic', default_value='/ouster/points'),
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('output_topic', default_value='/colored_points'),
        DeclareLaunchArgument('drop_uncolored', default_value='false'),
    ]

    node = Node(
        package='pointcloud_colorize',
        executable='colorize_node',
        name='pointcloud_colorize',
        output='screen',
        parameters=[{
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'drop_uncolored': LaunchConfiguration('drop_uncolored'),
        }],
    )

    return LaunchDescription(args + [node])
