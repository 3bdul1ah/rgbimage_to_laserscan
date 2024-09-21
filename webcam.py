import cv2
import torch
import numpy as np
import matplotlib
from ros_depth_anything_v2.depth_anything_v2.dpt import DepthAnythingV2
import time

DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

encoder = 'vits' # or 'vits', 'vitb', 'vitg'

model = DepthAnythingV2(**model_configs[encoder])
model.load_state_dict(torch.load(f'ros_depth_anything_v2/checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
model = model.to(DEVICE).eval()

# raw_img = cv2.imread('your/image/path')
 # HxW raw depth map in numpy

cam = cv2.VideoCapture(0)
cmap = matplotlib.colormaps.get_cmap('Spectral_r')

# used to record the time when we processed last frame
prev_frame_time = 0

# used to record the time at which we processed current frame
new_frame_time = 0

while True:
    ret, frame = cam.read()

    depth = model.infer_image(frame)

    depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
    depth = depth.astype(np.uint8)

    depth = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
    new_frame_time = time.time()
    fps = int(1/(new_frame_time-prev_frame_time))
    prev_frame_time = new_frame_time

    cv2.putText(depth,str(fps),(40,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255),2)
    cv2.imshow('Camera', depth)
    if cv2.waitKey(1) == ord('q'):
        break