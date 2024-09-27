import argparse
import os
from pathlib import Path  # noqa: E402
import sys

from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth_anything_v2_ros2', executable='depth_anything_v2_node', output='screen',
        ),
])
