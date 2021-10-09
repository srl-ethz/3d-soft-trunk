### Easy way to pre-load and use Detectron
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




def detectronTargetPosition (demo, pipeline, align, depth_scale):
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



    return pos