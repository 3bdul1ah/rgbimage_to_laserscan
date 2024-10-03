# ROS Package: RGB to DRGB to LaserScan Converter

## Description
This ROS package enables the conversion of RGB images to depth images using [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2) and subsequently transforms the depth image into LaserScan data.

## Possible Applications
- **SLAM:** Uses an RGB camera for 2D mapping, so no additional **LiDAR** sensor is required.
- **VSLAM:** Uses an RGB camera for visual SLAM, so a **depth camera** is not necessary.
- ..

## Nodes

- ### RGB to Depth Conversion

**Subscribed Topics:**
- **RGB Image (`sensor_msgs/Image`):**
  - **Topic:** (`/usb_cam/image_raw`)

- **Camera Info (`sensor_msgs/CameraInfo`):**
  - **Topic:** (`/usb_cam/camera_info`)

**Published Topics:**
- **Depth Image (`sensor_msgs/Image`):**
  - **Topic:** (`/depth/image_raw`)

- **Depth Camera Info (`sensor_msgs/CameraInfo`):**
  - **Topic:** (`/depth/camera_info`)

- ### Depth to LaserScan Conversion

**Subscribed Topics:**
- **Depth Image (`sensor_msgs/Image`):**
  - **Topic:** `/depth/image_raw`

- **Camera Info (`sensor_msgs/CameraInfo`):**
  - **Topic:** `/depth/camera_info`

**Published Topics:**
- **Laser Scan (`sensor_msgs/LaserScan`):**
  - **Topic:** `scan`

---
## Installation

### Dependencies 
This package is built for ROS Foxy, but it might build for subsequent versions too. It is recommended to [calibrate](https://docs.ros.org/en/rolling/p/camera_calibration/) your camera for the best perfomance. Additionally, the package depends on following packages and python libraries:

#### ROS Packages
 - [usb-cam](https://wiki.ros.org/usb_cam)
 - [depthimage_to_laserscan](https://wiki.ros.org/depthimage_to_laserscan)
#### Python Libraries
 - torch 
 - torchvision
 - opencv-python

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install ros-foxy-usb-cam
sudo apt install ros-foxy-depthimage-to-laserscan
pip install opencv-python torch torchvision
```
### 2. Building
```bash
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/3bdul1ah/rgbimage_to_laserscan
cd .. && colcon build
source install/setup.bash
```
### 3. Run the Project

After completing the installation steps, you can then run the example:

```bash
ros2 launch depth_anything_v2_ros2 webcam_laserscan.launch
```

