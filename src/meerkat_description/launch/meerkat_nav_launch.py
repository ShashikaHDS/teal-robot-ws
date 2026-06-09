from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():

    lidar2d_launch_file = "/media/meerkat/Meerkat3/meerkat_ws/src/sick_scan_xd/launch/sick_tim_5xx.launch.py"

    teleop_launch_file = '/media/meerkat/Meerkat3/meerkat_ws/src/meerkat_motors/launch/meerkat_teleop_launch.py'

    odometry_launch_file = '/media/meerkat/Meerkat3/meerkat_ws/src/meerkat_odometry/launch/odometry_data_launch.py'
    
    display_launch_file = "/media/meerkat/Meerkat3/meerkat_ws/src/meerkat_description/launch/display.launch.py"


    # Include the XML launch file
    lidar2d_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(lidar2d_launch_file)
    )
    
    teleop_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(teleop_launch_file)
    )
    
    odometry_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(odometry_launch_file)
    )

    display_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(display_launch_file)
    )


    

    return LaunchDescription([
    	teleop_launch,
        lidar2d_launch,
        odometry_launch,
        display_launch
    ])
