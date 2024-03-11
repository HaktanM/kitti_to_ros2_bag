import numpy as np
import cv2

import os
import shutil
import math

import utils
import importlib
importlib.reload(utils)

######## LOAD THE KITTI DATA ########
main_path = '/home/hakito/datasets/kitti_raw_unsncy/2011_09_30_drive_0028_extract/2011_09_30/2011_09_30_drive_0028_extract'
data = utils.data_loader(main_path)

out_folder = 'rosbag_deneme'
bag_creator = utils.rosbag_writer(out_folder)

prev_id = -1
# Write cam0 images
for img_idx,img_name in enumerate(data.img0_list):
    # First check the order of the images
    curr_id = int(img_name[:-4])
    if prev_id>curr_id:
        print("img0, check your sorting")
        break
    prev_id = curr_id
    
    img_path = os.path.join(data.img0_folder,img_name)
    img_daytime = data.img0_daytimes[img_idx]
    img_stamp = utils.datetime_to_stamp(img_daytime)
    bag_creator.write_cam0(img_path,img_stamp)

prev_id = -1
# Write cam1 images    
for img_idx,img_name in enumerate(data.img1_list):
    # First check the order of the images
    curr_id = int(img_name[:-4])
    if prev_id>curr_id:
        print("img1, check your sorting")
        break
    prev_id = curr_id
    
    curr_id = int(img_name[:-4])
    img_path = os.path.join(data.img1_folder,img_name)
    img_daytime = data.img1_daytimes[img_idx]
    img_stamp = utils.datetime_to_stamp(img_daytime)
    bag_creator.write_cam1(img_path,img_stamp)

prev_id = -1
# Write imus
for oxts_idx,oxts_name in enumerate(data.oxts_list):
    # First check the order of the oxts
    curr_id = int(oxts_name[:-4])
    if prev_id>curr_id:
        print("oxts, check your sorting")
        break
    prev_id = curr_id
    
    curr_id = int(img_name[:-4])
    oxts_path = os.path.join(data.oxts_folder,oxts_name)
    oxts_daytime = data.oxts_daytimes[oxts_idx]
    oxts_stamp = utils.datetime_to_stamp(oxts_daytime)
    bag_creator.write_imu(oxts_path,oxts_stamp)
    
bag_creator.end_writing()