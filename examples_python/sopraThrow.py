# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import glob
import multiprocessing as mp
import numpy as np
import os
import tempfile
import time
import warnings
import cv2
import tqdm

import pyrealsense2 as rs


from detectron2.config import get_cfg
from detectron2.data.detection_utils import read_image
from detectron2.utils.logger import setup_logger

from predictor import VisualizationDemo

# constants
WINDOW_NAME = "COCO detections"


def setup_cfg(args):
    # load config from file and command-line arguments
    cfg = get_cfg()
    # To use demo for Panoptic-DeepLab, please uncomment the following two lines.
    # from detectron2.projects.panoptic_deeplab import add_panoptic_deeplab_config  # noqa
    # add_panoptic_deeplab_config(cfg)
    cfg.merge_from_file(args.config_file)
    cfg.merge_from_list(args.opts)
    # Set score_threshold for builtin models
    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = args.confidence_threshold
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = args.confidence_threshold
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = args.confidence_threshold
    cfg.freeze()
    return cfg


def get_parser():
    parser = argparse.ArgumentParser(description="Detectron2 demo for builtin configs")
    parser.add_argument(
        "--config-file",
        default="../configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml",
        metavar="FILE",
        help="path to config file",
    )

    parser.add_argument(
        "--confidence-threshold",
        type=float,
        default=0.5,
        help="Minimum score for instance predictions to be shown",
    )
    parser.add_argument(
        "--opts",
        help="Modify config options using the command-line 'KEY VALUE' pairs",
        default=["MODEL.WEIGHTS", "detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl"],
        nargs=argparse.REMAINDER,
    )
    return parser


if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    args = get_parser().parse_args()
    setup_logger(name="fvcore")
    logger = setup_logger()
    logger.info("Arguments: " + str(args))

    cfg = setup_cfg(args)

    demo = VisualizationDemo(cfg)


    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # We already know the device resolution. 720p with 30 fps possible, or 480p 60fps. RGB can go up to 1080p 30fps.
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    img_center = (320, 240)

    # Start streaming
    profile = pipeline.start(config)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Get data scale from the device and convert to meters
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    try:
        # Get three timesteps of target positions
        xt_2 = None
        xt_1 = None
        xt = None
        target_point = None

        alpha = 0.3
        v_running_mean = 0
        a_running_mean = 0

        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Process color image
            predictions, vis_output = demo.run_on_image(color_image)
            pred_img = color_image
            try:
                center = predictions['instances'].pred_boxes.get_centers()[0].detach().cpu().numpy()    # (height, width) by default, need to be transposed for cv visualization
                center_int = (int(center[1]), int(center[0]))
                img = cv2.cvtColor(vis_output.get_image(), cv2.COLOR_RGB2BGR)
                pred_img = cv2.circle(img, (center_int[1], center_int[0]), 5, (0, 0, 255), -1)

                margin = 8
                depth_center = depth_image[max(0, center_int[0]-margin): center_int[0]+margin, max(0, center_int[1]-margin): center_int[1]+margin].mean() * depth_scale

                #print(f"Depth at {center}: {depth_center}")


                ### Assume that we have the camera right below the sopra arm. x to the down, y is right and z is outward (not a coord system???). Origin of y can be shifted. Measure in meters.
                xt_2 = xt_1
                xt_1 = xt
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                xt = np.array(rs.rs2_deproject_pixel_to_point(depth_intrin, center_int, depth_center))
                # up down first coord, negative is up. left neg right pos, then depth outwards.
                
                print(f"3D depth_point: {xt}")

                if xt_2 is not None:
                    dt = 0.033  # 30fps
                    v = (xt - xt_1)
                    #v[v<5e-3] = 0
                    v /= dt
                    v_running_mean = alpha * v + (1-alpha) * v_running_mean
                    
                    a = (xt - 2*xt_1 + xt_2)
                    #a[a<1e-2] = 0
                    a /= dt**2
                    a_running_mean = alpha * a + (1-alpha) * a_running_mean

                    #print(f"Velocity: {v}")
                    #print(f"Acceleration: {a}")


                    fut_time = 0.1  # Time into the future
                    target_point = xt + fut_time*v_running_mean + a_running_mean*(fut_time**2)/2

                    print(f"3D target: {target_point}")
                    target = np.array(rs.rs2_project_point_to_pixel(depth_intrin, target_point)).astype(np.uint32)
                    pred_img = cv2.circle(pred_img, (min(max(0,target[1]), 480), min(max(0,target[0]), 640)), 5, (0, 255, 0), -1)

                    ### Normalize just for direction
                    norm_ref = xt / np.linalg.norm(xt)

                    ### First go to some position in negative direction to "wind-up" prepare for throw
                    neg_ref = -norm_ref

                    ref = norm_ref
                    

            except:
                print("Prediction skipped")



            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((pred_img, resized_color_image, depth_colormap))
            else:
                images = np.hstack((pred_img, color_image, depth_colormap))


            # Show images
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(WINDOW_NAME, images)

            if cv2.waitKey(1) == 27:
                break  # esc to quit

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()
        
