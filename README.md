#ROS Package for Converting RGB Images to Depth to LaserScan Data

## Description
This ROS package provides a pipeline to convert an RGB image into a depth image using [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2), and then converts the depth image into a LaserScan message using the [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan/tree/melodic-devel) package. This enables the use of a camera as a virtual LiDAR sensor for both SLAM and Visual SLAM (VSLAM) applications in robotics.

## Features
- Converts RGB images to depth maps using **Depth Anything V2**.
- Converts depth maps to LaserScan format using **depthimage_to_laserscan**.
- Integrates with the ROS navigation stack for SLAM and obstacle avoidance.
- Supports both Visual SLAM (VSLAM) and 2D SLAM using a camera as a virtual LiDAR.

## Installation

Coming soon.
