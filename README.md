# ROS Package: RGB to DRGB to LaserScan Converter

## Description
This ROS package enables the conversion of RGB images to depth images using [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2) and subsequently transforms the depth image into LaserScan data.

## Features
- **SLAM:** Uses an RGB camera for 2D mapping, so no additional **LiDAR** sensor is required.
- **VSLAM:** Uses an RGB camera for visual SLAM, so a **depth camera** is not necessary.
- ..
- ..


## Nodes

- ### RGB to Depth Conversion

**Subscribed Topics:**
- **RGB Image (`sensor_msgs/Image`):**
  - **Topic:** `image_topic` (default: `/usb_cam/image_raw`)
  - Receives raw RGB images for depth conversion.

- **Camera Info (`sensor_msgs/CameraInfo`):**
  - **Topic:** `camera_info` (default: `/usb_cam/camera_info`)
  - Provides intrinsic parameters for accurate depth estimation.

**Published Topics:**
- **Depth Image (`sensor_msgs/Image`):**
  - **Topic:** `depth_topic` (default: `/depth/image_raw`)
  - Outputs the processed depth image.

- **Depth Camera Info (`sensor_msgs/CameraInfo`):**
  - **Topic:** `depth_info_topic` (default: `/depth/camera_info`)
  - Contains intrinsic parameters for the depth camera.



- ### Depth to LaserScan Conversion

**Subscribed Topics:**
- **Depth Image (`sensor_msgs/Image`):**
  - **Topic:** `/depth/image_raw`
  - Contains depth image.

- **Camera Info (`sensor_msgs/CameraInfo`):**
  - **Topic:** `/depth/camera_info`
  - Provides camera information for the depth image.

**Published Topics:**
- **Laser Scan (`sensor_msgs/LaserScan`):**
  - **Topic:** `scan`
  - The final 2D laser scan generated from the depth image.

---

## Installation
Coming soon.
