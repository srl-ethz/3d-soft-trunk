### Easy way to pre-load and use Detectron
import cv2 
import multiprocessing as mp
import pyrealsense2 as rs
import numpy as np

from predictor import VisualizationDemo

from detectron2.utils.logger import setup_logger
from detectron2.config import get_cfg


def loadNetwork ():
    mp.set_start_method("spawn", force=True)
    setup_logger(name="fvcore")
    logger = setup_logger()

    cfg = get_cfg() 
    config_file = "../configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"
    opts = ["MODEL.WEIGHTS", "detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl"]
    confidence_threshold = 0.5
    cfg.merge_from_file(config_file)
    cfg.merge_from_list(opts)
    cfg.MODEL.RETINANET.SCORE_THRESH_TEST = confidence_threshold
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = confidence_threshold
    cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = confidence_threshold
    cfg.freeze()


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

    return demo, pipeline, align, depth_scale




def detectronTargetPosition (demo, pipeline, align, depth_scale, visualize=False):
    # The 3D point might be out of reach for the manipulation configuration space of the gripper. Does this cause any issues?
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        return None

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
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        target_point = np.array(rs.rs2_deproject_pixel_to_point(depth_intrin, center_int, depth_center))
        # up down first coord, negative is up. left neg right pos, then depth outwards.
        
        #print(f"3D target: {target_point}")
        target = np.array(rs.rs2_project_point_to_pixel(depth_intrin, target_point)).astype(np.uint32)
        pred_img = cv2.circle(pred_img, (min(max(0,target[1]), 480), min(max(0,target[0]), 640)), 5, (0, 255, 0), -1)

    except:
        print("Prediction skipped")
        return None


    if visualize:
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
        WINDOW_NAME = "COCO detections"
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(WINDOW_NAME, images)



    ### Normalize just for direction
    norm_ref = target_point / np.linalg.norm(target_point)
    # First go to some position in negative direction to "wind-up" prepare for throw
    neg_ref = -norm_ref


    distance_to_soprabase = np.array([0,0,1])
    target_point -= distance_to_soprabase

    return target_point