import cv2
from transitions import Machine
import open3d as o3d

# from parasight.segment_ui import SegmentAnythingUI
from segment_ui import SegmentAnythingUI

from parasight.registration import RegistrationPipeline
from parasight.utils import *

import time
from parasight_interfaces.srv import StartTracking, StopTracking
from parasight_interfaces.msg import TrackedPoints
from std_srvs.srv import Empty as EmptySrv
from functools import partial

from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
import glob

path_to_depth = "./../../data_sree/depth_realsense/"
path_to_rgb = "./../../data_sree/rgb_realsense/"
all_rgb_frames = sorted(glob.glob(path_to_rgb + "*"))

# all_depth_frames = sorted(glob.glob(path_to_depth + "*"))
# query_rgb_frame = all_rgb_frames[100]

segmentation_ui = SegmentAnythingUI()


for frame_num in range(40, 40+40):
    query_rgb_frame = all_rgb_frames[frame_num]

    print(f"The query RGB frame: {query_rgb_frame}")
    image = cv2.imread(query_rgb_frame)
    # ip = input()
    masks, annotated_points, all_mask_points = segmentation_ui.segment_using_ui(image)
    cv2.destroyAllWindows()

    # if ip == "0":
        # cv2.destroyAllWindows()
        # print(f"moving on from {query_rgb_frame}")    

# print(f"Datatypes: {type(masks)}, {type(annotated_points)} and {type(all_mask_points)}")
# # list, list of 2 points, list
    print(f"len masks: {len(masks)} \n {annotated_points} \n len all mask points: {len(all_mask_points)}")

# print(f"{type(masks[0])}")
# print(f"{masks[0].shape}")



