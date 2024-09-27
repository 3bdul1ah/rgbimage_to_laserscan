from setuptools import setup
import os
from glob import glob
package_name = 'depth_anything_v2_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][ymal]*')),
        # ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name + "/config", glob('config/*.yaml')),
        ('share/' + package_name + "/config", glob('config/*.rviz')),
        ('share/' + package_name + "/Depth_Anything_V2/checkpoints", glob(os.path.join('depth_anything_v2_ros2/Depth_Anything_V2/checkpoints', '*.pth'))),
        ('lib/' + package_name + "/Depth_Anything_V2/metric_depth/depth_anything_v2", glob(os.path.join('depth_anything_v2_ros2/Depth_Anything_V2/metric_depth/depth_anything_v2', '*.py'))),
        ('lib/' + package_name + "/Depth_Anything_V2/metric_depth/depth_anything_v2/util", glob(os.path.join('depth_anything_v2_ros2/Depth_Anything_V2/metric_depth/depth_anything_v2/util', '*.py'))),
        ('lib/' + package_name + "/Depth_Anything_V2/metric_depth/depth_anything_v2/dinov2_layers", glob(os.path.join('depth_anything_v2_ros2/Depth_Anything_V2/metric_depth/depth_anything_v2/dinov2_layers', '*.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anas',
    maintainer_email='ameranas24@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_anything_v2_node = depth_anything_v2_ros2.depth_anything_v2_node:main'
        ],
    },
)
