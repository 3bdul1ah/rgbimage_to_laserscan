#!/usr/bin/env python3
import cv2
import torch
import numpy as np
import matplotlib
from Depth_Anything_V2.depth_anything_v2.dpt import DepthAnythingV2
import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import  CvBridge, CvBridgeError
import rospkg

rospack = rospkg.RosPack()
bridge = CvBridge()

DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]}
}

encoder = 'vits' # or 'vits', 'vitb'
dataset = 'hypersim' # 'hypersim' for indoor model, 'vkitti' for outdoor model
max_depth = 20 # 20 for indoor model, 80 for outdoor model

model = DepthAnythingV2(**{**model_configs[encoder], 'max_depth': max_depth})
pkg = rospack.get_path('depth_anything_v2_ros')
model.load_state_dict(torch.load(f'{pkg}/src/Depth_Anything_V2/checkpoints/depth_anything_v2_metric_{dataset}_{encoder}.pth', map_location='cpu'))
model = model.to(DEVICE).eval()


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = (value - leftMin) / leftSpan

    return rightMin + (valueScaled * rightSpan)

class image_converter:

  def __init__(self):
    image_topic = rospy.get_param('image_topic', '/usb_cam/image_raw')
    camera_info = rospy.get_param('camera_info', '/usb_cam/camera_info')

    depth_topic = rospy.get_param('depth_topic', '/depth/image_raw')
    depth_info_topic = rospy.get_param('depth_info_topic', '/depth/camera_info')


    self.image_pub = rospy.Publisher(depth_topic,Image, queue_size=1)
    self.info_pub = rospy.Publisher(depth_info_topic,CameraInfo, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback, queue_size=1)
    self.info_sub = rospy.Subscriber(camera_info,CameraInfo,self.info_callback, queue_size=1)
    self.camera_info = CameraInfo()


  def info_callback(self,msg):
     self.camera_info.header = msg.header
     self.camera_info.binning_x = msg.binning_x
     self.camera_info.binning_y = msg.binning_y
     self.camera_info.K = msg.K
     self.camera_info.P = msg.P
     self.camera_info.R = msg.R
     self.camera_info.D = msg.D
     self.camera_info.height = msg.height
     self.camera_info.width = msg.width
     self.camera_info.distortion_model = msg.distortion_model

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except CvBridgeError as e:
      print(e)

    depth = model.infer_image(cv_image)

    # cv2.imshow("Image window", depth)
    # cv2.waitKey(3)S
    time_now = rospy.Time.now()
    img_msg = self.bridge.cv2_to_imgmsg(depth, "32FC1")
    img_msg.header.stamp = time_now
    try:
      self.image_pub.publish(img_msg)
    except CvBridgeError as e:
      print(e)
    self.camera_info.header.stamp = time_now
    self.info_pub.publish(self.camera_info)



def main(args):
    rospy.init_node('depth_anything_v2', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

