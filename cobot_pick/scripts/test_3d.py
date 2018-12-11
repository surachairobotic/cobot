#!/usr/bin/env python

'''

test data from capture.py

'''

import sys
import os
import rospy
import tf
import tf2_ros
import time
import pickle
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


if __name__ == "__main__":
  with open('realsense.pkl', 'rb') as f:
    topics = pickle.load(f)
  
  depth = topics['depth_image']
  cam_info = topics['depth_info']
  print(cam_info)
  

