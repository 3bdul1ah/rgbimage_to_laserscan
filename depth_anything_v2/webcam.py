import argparse
import cv2
import matplotlib
import numpy as np
import torch
import os

from depth_anything_v2.dpt import DepthAnythingV2

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Anything V2 with Webcam')
    parser.add_argument('--input-size', type=int, default=518)
    parser.add_argument('--outdir', type=str, default='./vis_depth')
    parser.add_argument('--encoder', type=str, default='vitl', choices=['vits', 'vitb', 'vitl', 'vitg'])
    parser.add_argument('--pred-only', dest='pred_only', action='store_true', help='only display the prediction')
    parser.add_argument('--grayscale', dest='grayscale', action='store_true', help='do not apply colorful palette')
    args = parser.parse_args()
    # Select device (GPU if available)
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    # Model configurations
    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }
    # Load model
    depth_anything = DepthAnythingV2(**model_configs[args.encoder])
    depth_anything.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{args.encoder}.pth', map_location='cpu'))
    depth_anything = depth_anything.to(DEVICE).eval()

    # Create output directory
    os.makedirs(args.outdir, exist_ok=True)

    # Colormap for visualizing depth
    cmap = matplotlib.colormaps.get_cmap('Spectral_r')

    # Start webcam capture
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()
    frame_counter = 0
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            break

        # Infer depth from the captured frame
        depth = depth_anything.infer_image(frame, args.input_size)

        # Normalize depth map to 0-255 for visualization
        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth = depth.astype(np.uint8)

        # Convert depth to grayscale or color
        if args.grayscale:
            depth_visual = np.repeat(depth[..., np.newaxis], 3, axis=-1)
        else:
            depth_visual = (cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)

        # Combine original frame and depth map side-by-side if pred_only is not set
        if not args.pred_only:
            split_region = np.ones((frame.shape[0], 50, 3), dtype=np.uint8) * 255
            combined_result = cv2.hconcat([frame, split_region, depth_visual])
            cv2.imshow('Depth Prediction', combined_result)
        else:
            cv2.imshow('Depth Prediction', depth_visual)

        # Save the frame if needed
        frame_counter += 1
        cv2.imwrite(os.path.join(args.outdir, f'frame_{frame_counter}.png'), depth_visual)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

