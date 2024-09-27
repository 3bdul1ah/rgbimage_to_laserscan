from Depth_Anything_V2.metric_depth.depth_anything_v2.dpt import DepthAnythingV2
import time
import numpy as np
import torch
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
import ament_index_python

bridge = CvBridge()

DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}
}

encoder = 'vits' 
dataset = 'hypersim' # 'hypersim' for indoor model, 'vkitti' for outdoor model
max_depth = 20 # 20 for indoor model, 80 for outdoor model
pkg_path = ament_index_python.get_package_share_directory("depth_anything_v2_ros2")
model = DepthAnythingV2(**{**model_configs[encoder], 'max_depth': max_depth})
model.load_state_dict(torch.load(f'{pkg_path}/Depth_Anything_V2/checkpoints/depth_anything_v2_metric_hypersim_vits.pth', map_location='cpu'))
model = model.to(DEVICE).eval()


def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = (value - leftMin) / leftSpan
    return rightMin + (valueScaled * rightSpan)

class image_converter(Node):

  def __init__(self):
    super().__init__('depth_anything')
    self.declare_parameter('image_topic', '/usb_cam/image_raw')
    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.declare_parameter('camera_info', '/usb_cam/camera_info')
    camera_info = self.get_parameter('camera_info').get_parameter_value().string_value
    self.declare_parameter('depth_topic', '/depth/image_raw')
    depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
    self.declare_parameter('depth_info_topic', '/depth/camera_info')
    depth_info_topic = self.get_parameter('depth_info_topic').get_parameter_value().string_value


    self.image_pub =  self.create_publisher(Image, depth_topic,qos_profile=1)
    self.info_pub = self.create_publisher(CameraInfo,depth_info_topic,qos_profile=1)

    self.bridge = CvBridge()
    self.image_sub = self.create_subscription(Image,image_topic,self.callback, qos_profile=1)
    self.info_sub = self.create_subscription(CameraInfo,camera_info,self.info_callback, qos_profile=1)
    self.camera_info = CameraInfo()


  def info_callback(self,msg):
     self.camera_info.header = msg.header
     self.camera_info.binning_x = msg.binning_x
     self.camera_info.binning_y = msg.binning_y
     self.camera_info.k = msg.k
     self.camera_info.p = msg.p
     self.camera_info.r = msg.r
     self.camera_info.d = msg.d
     self.camera_info.height = msg.height
     self.camera_info.width = msg.width
     self.camera_info.distortion_model = msg.distortion_model

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    depth = model.infer_image(cv_image)

    # cv2.imshow("Image window", depth)
    # cv2.waitKey(3)S
    time_now = self.get_clock().now().to_msg()
    img_msg = self.bridge.cv2_to_imgmsg(depth, "32FC1")
    img_msg.header.stamp = time_now
    try:
      self.image_pub.publish(img_msg)
    except CvBridgeError as e:
      print(e)
    self.camera_info.header.stamp = time_now
    self.info_pub.publish(self.camera_info)



def main():
    rclpy.init()
    ic = image_converter()
    rclpy.spin(ic)


if __name__ == '__main__':
    main()