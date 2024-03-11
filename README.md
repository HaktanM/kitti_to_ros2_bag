# Convert Raw Kitti Data To ROS2 Bag Format
This repo is designed for converting the raw unsync Kitti data to ros2 bag format. I write the IMU and Images to a bag file. This file format is especially useful for OpenVINS, VinsMONO, ORB-SLAM in ROS2 implementations.

## Installation
Dowload the repo using git.
```sh
git clone https://github.com/HaktanM/kitti_to_ros2_bag.git
```
I recommend creating a python environment. 
```sh
cd kitti_to_ros2_bag
python3 -m venv env # creates a virtual environment
source env/bin/activate
```
You need to install rosbag library. Even if you have ros2 only, do not worry.
```sh
pip install rosbags
```
That's it.

## Demo
İnstall one of the raw data from [here](https://www.cvlibs.net/datasets/kitti/raw_data.php). Note that you are expected to dowload \[unsynced+unrectified data\]. Modify the main path inside the kitti_to_ros2_bag.py.
```sh
main_path = '/home/hakito/datasets/kitti_raw_unsncy/2011_09_30_drive_0028_extract/2011_09_30/2011_09_30_drive_0028_extract'
```
The just run.

## Additional Notes
You can check your rosbag by
```sh
ros2 bag info /rosbag_deneme
```
