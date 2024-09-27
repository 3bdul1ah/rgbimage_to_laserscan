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
This package is built for ROS Noetic, but it might build for ROS Melodic too. Additionally, the package depends on following packages and python libraries:

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
sudo apt install ros-noetic-usb-cam
sudo apt install ros-noetic-depthimage-to-laserscan
pip install opencv-python torch torchvision
```
### 2. Building
```bash
cd ~/catkin_ws/src
git clone https://github.com/3bdul1ah/rgbimage_to_laserscan
cd .. && catkin_make
source devel/setup.bash
```
### 3. Run the Project

After completing the installation steps, make sure you calibrate your camera using the instructions [here](https://wiki.ros.org/camera_calibration). You can then run the example:

```bash
roslaunch depth_anything_v2_ros webcam_laser.launch
```

