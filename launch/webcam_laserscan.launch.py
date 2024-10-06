import argparse
import os
from pathlib import Path  # noqa: E402
import sys

from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    
    return LaunchDescription([
   
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            remappings=[
                ("image_raw","usb_cam/image_raw"),
                ("camera_info","usb_cam/camera_info"),
                ("image_raw/compressed","usb_cam/image_raw/compressed"),
                ("image_raw/compressed_depth","usb_cam/image_raw/compressed_depth"),
                ("image_raw/theora","usb_cam/image_raw/theora"),
            ],parameters=[{
                "video_device": "/dev/video0",
                "camera_name":"usb_cam",
                "camera_info_url" : "package://depth_anything_v2_ros2/config/camera_info.yaml"
                }]
        ),
        Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ['0', '0', '0', '0', '0','0', 'base_link', 'camera']),

        Node(
            package='depth_anything_v2_ros2', executable='depth_anything_v2_node', output='screen',
        ),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[('depth','/depth/image_raw'),
                        ('depth_camera_info', '/depth/camera_info'),
                        ('scan', '/scan')],
            parameters=[{
                "scan_time": 0.033,
                "range_min": 0.3,
                "range_max": 20.0,
                "output_frame": "camera",
                "offset": 0
        }]),

         Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('depth_anything_v2_ros2'), 'config', 'rviz.rviz')]
        ),
])
