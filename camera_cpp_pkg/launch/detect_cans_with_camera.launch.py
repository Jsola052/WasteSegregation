from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'pointcloud.enable': True,
                'allow_no_texture_points': True,
                'ordered_pc': True,
                'enable_sync': True,
                'align_depth.enable': True,
                # ✅ No depth_module.profile
                # ✅ No rgb_camera.profile
                'depth_module.enable_auto_exposure': True,
                'rgb_camera.enable_auto_exposure': True,
                'enable_color': True,
                'enable_depth': True,
            }],
        ),

        Node(
            package='camera_cpp_pkg',
            executable='detect_cans_publisher',
            name='detect_cans_publisher',
            output='screen',
            parameters=[{
                'api_key': 'YOUR_API_KEY_HERE',
                'endpoint': 'https://fiuaiservice.com/api/cv/instance_segmentation/LLW'
            }]
        ),
    ])
