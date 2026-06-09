from launch import LaunchDescription
from launch_ros.actions import Node


# base_link is 0.26 m above the ground.
# base_link → os_lidar: z = +0.31 m (LiDAR height above base_link)
# odom is initialized at the LiDAR start pose, so odom → base_link: z ≈ −0.31 m
# ground in odom frame: z ≈ −(0.31 + 0.26) = −0.57 m

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', '/lio_sam/mapping/cloud_registered'),   # LIO-SAM cloud
                ('projected_map', '/map'),                           # 2D occupancy out
            ],
            parameters=[{
                # Frames
                'frame_id': 'odom',             # Use 'map' if LIO-SAM provides a stable map frame; else 'odom'
                'base_frame_id': 'base_link',

                # OctoMap resolution (meters): 0.05–0.15 typical
                'resolution': 0.05,

                'pointcloud_min_z': -1.0,    # keep a bit below floor to catch noise
                'pointcloud_max_z':  1.0,

                # Raycasting & cleanup
                'filter_speckles': True,
                'sensor_model/max_range': 30.0,   # correct param key

                # 2D projection band (GROUND-RELATIVE in 'odom')
                'project_2d_map': True,
                'occupancy_min_z': -0.52,   # -0.57 + 0.05
                'occupancy_max_z':  0.33,   # -0.57 + 0.90


                # Optional: leave default hit/miss unless need sharper maps
                # 'prob_hit': 0.7,
                # 'prob_miss': 0.4,
                # 'clamping_thres_min': 0.12,
                # 'clamping_thres_max': 0.97,
            }]
        )
    ])
