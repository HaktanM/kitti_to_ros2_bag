from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

import cv2
import numpy as np
import os
import shutil

from datetime import datetime

######## GET THE REQUIRED DATA STRUCTURES ########
typestore = get_typestore(Stores.LATEST)
String = typestore.types['std_msgs/msg/String']
Image = typestore.types['sensor_msgs/msg/Image']
Imu = typestore.types['sensor_msgs/msg/Imu']
Image = typestore.types['sensor_msgs/msg/Image']

Header = typestore.types['std_msgs/msg/Header']
Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
Vector = typestore.types['geometry_msgs/msg/Vector3']

time = typestore.types['builtin_interfaces/msg/Time']



######## UTILITIES TO CONVERT DATA INTO MSG FORMAT ########
def img_to_msg(img_path,stamp,frame_id = "cam"):
    img = cv2.imread(img_path,cv2.IMREAD_GRAYSCALE)
    # cv2.imshow('w',img)
    # print(stamp)
    # cv2.waitKey()
    header = Header(stamp,frame_id)
    height = img.shape[0]
    width = img.shape[1]
    is_bigendian = 0
    step = img.shape[1]  #only with mono8! (step = width * byteperpixel * numChannels)
    encoding = "mono8"
    data = np.array(img).flatten()
    img_msg = Image(header, height, width, encoding, is_bigendian, step, data)
    return img_msg

def imu_to_msg(oxts_path,stamp,frame_id="imu"):
    oxts_data = np.loadtxt(oxts_path,delimiter=' ')
    gyr_data = np.array([oxts_data[17],oxts_data[18],oxts_data[19]])
    acc_data = np.array([oxts_data[11],oxts_data[12],oxts_data[13]])
    header = Header(stamp,frame_id)
    ang_vel = Vector(gyr_data[0],gyr_data[1],gyr_data[2])
    lin_acc = Vector(acc_data[0],acc_data[1],acc_data[2])
    quat = Quaternion(0.0,0.0,0.0,0.0)
    cov = np.zeros((9,), dtype=np.float64)
    imu_msg = Imu(header, quat, np.copy(cov), ang_vel, np.copy(cov), lin_acc, np.copy(cov))
    return imu_msg


def datetime_to_stamp(daytime : datetime):
        hour2sec = 60 * 60
        min2sec  = 60 
        sec = int(daytime.hour * hour2sec + daytime.minute * min2sec + daytime.second  + 1e9)
        nsec = int(daytime.microsecond * 1e3)
        stamp = time(sec,nsec)
        return stamp

class data_loader:
    def __init__(self, main_path):
        self.img0_folder = os.path.join(main_path,'image_00','data')
        self.img1_folder = os.path.join(main_path,'image_01','data')
        self.oxts_folder = os.path.join(main_path,'oxts','data')

        # Sort the data so that the data is in the correct order
        self.img0_list = os.listdir(self.img0_folder)
        self.img0_list.sort()

        self.img1_list = os.listdir(self.img1_folder)
        self.img1_list.sort()

        self.oxts_list = os.listdir(self.oxts_folder)
        self.oxts_list.sort()
        
        ######## Read The Timestamps ########
        #### Cam0 ####
        img0_timestamps_path = os.path.join(main_path,'image_00','timestamps.txt')
        with open(img0_timestamps_path, 'r') as file:
            img0_timestamps_data = file.read()
            
        # Split the data into lines and remove empty lines
        img0_timestamps_lines = img0_timestamps_data.strip().split('\n')

        # Iterate over lines and parse each datetime string
        self.img0_daytimes = []
        for line in img0_timestamps_lines:
            event_time = datetime.strptime(line[:-3], '%Y-%m-%d %H:%M:%S.%f')
            self.img0_daytimes.append(event_time)
            
            
        #### Cam1 ####
        img1_timestamps_path = os.path.join(main_path,'image_01','timestamps.txt')
        with open(img1_timestamps_path, 'r') as file:
            img1_timestamps_data = file.read()
            
        # Split the data into lines and remove empty lines
        img1_timestamps_lines = img1_timestamps_data.strip().split('\n')

        # Iterate over lines and parse each datetime string
        self.img1_daytimes = []
        for line in img1_timestamps_lines:
            event_time = datetime.strptime(line[:-3], '%Y-%m-%d %H:%M:%S.%f')
            self.img1_daytimes.append(event_time)
            

        #### oxts ####
        oxts_timestamps_path = os.path.join(main_path,'oxts','timestamps.txt')
        with open(oxts_timestamps_path, 'r') as file:
            oxts_timestamps_data = file.read()
            
        # Split the data into lines and remove empty lines
        oxts_timestamps_lines = oxts_timestamps_data.strip().split('\n')

        # Iterate over lines and parse each datetime string
        self.oxts_daytimes = []
        for line in oxts_timestamps_lines:
            event_time = datetime.strptime(line[:-3], '%Y-%m-%d %H:%M:%S.%f')
            self.oxts_daytimes.append(event_time)
            
            
        ######### Check If the Loaded Data is Valid #########
        self.valid = True
        
        if len(self.img0_daytimes) != len(self.img0_list):
            self.valid = False
            
        if len(self.img1_daytimes) != len(self.img1_list):
            self.valid = False
            
        if len(self.oxts_daytimes) != len(self.oxts_list):
            self.valid = False
            
        if(self.valid):
            print("The loaded data seems to be OK. In other words")
            print("The number of images and number of the corresponding timestamps are equal")
            print("The number of GPS/IMU measurements and number of the corresponding timestamps are equal")
                   
    
class rosbag_writer:
    def __init__(self, out_folder):
        # Check if the folder exists
        if os.path.exists(out_folder):
            shutil.rmtree(out_folder)
            print(f"We notice that the folder '{out_folder}' already exists.\nIt has been deleted now.")
        
        self.writer = Writer(out_folder)
        print(f"The bag will be created at : {out_folder}")
        self.writer.open()

        # Define Connections
        self.msgtype_img = Image.__msgtype__
        topic_img0 = '/img0'
        self.connection_img0 = self.writer.add_connection(topic_img0, self.msgtype_img, typestore=typestore)
        topic_img1 = '/img1'
        self.connection_img1 = self.writer.add_connection(topic_img1, self.msgtype_img, typestore=typestore)
        
        self.topic_imu = '/imu'
        self.msgtype_imu = Imu.__msgtype__
        self.connection_imu = self.writer.add_connection(self.topic_imu, self.msgtype_imu, typestore=typestore)
    
    def write_cam0(self,img_path, stamp):
        publishing_time = int((stamp.sec * 1e9) + stamp.nanosec)
        img_msg = img_to_msg(img_path,stamp,frame_id='cam0')
        self.writer.write(self.connection_img0, publishing_time, typestore.serialize_cdr(img_msg, self.msgtype_img))
    
    def write_cam1(self,img_path, stamp):
        publishing_time = int((stamp.sec * 1e9) + stamp.nanosec)
        img_msg = img_to_msg(img_path,stamp,frame_id='cam1')
        self.writer.write(self.connection_img1, publishing_time, typestore.serialize_cdr(img_msg, self.msgtype_img))
        
    def write_imu(self,oxts_path, stamp):
        publishing_time = int((stamp.sec * 1e9) + stamp.nanosec)
        imu_msg = imu_to_msg(oxts_path,stamp,frame_id='imu')
        self.writer.write(self.connection_imu, publishing_time, typestore.serialize_cdr(imu_msg, self.msgtype_imu))
        
    def end_writing(self):
        print("Rosbag is completed")
        self.writer.close()