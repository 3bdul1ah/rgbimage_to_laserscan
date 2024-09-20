# ros_rgb_image_to_laserscan

**ROS Package for Converting RGB Images to Depth and LaserScan Data**

## Description
This ROS package provides a pipeline to convert an RGB image into a depth image using [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2), and then converts the depth image into a LaserScan message using the [depthimage_to_laserscan](https://github.com/ros-perception/depthimage_to_laserscan/tree/melodic-devel) package.

## Features
- Converts `rgbimage` to `depthimage` using **Depth Anything V2**.
- Transforms `depthimages` into `LaserScan` format.
- Allows a camera to function as a virtual LiDAR sensor for SLAM and VSLAM applications.

## Installation

Coming soon.
