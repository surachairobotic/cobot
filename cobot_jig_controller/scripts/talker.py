# sphinx_gallery_thumbnail_number = 3
import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt
import numpy as np

file_path = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/"

if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)
  
  file_name = "torque_j5_-15.txt"
#  fp = open(file_path + file_name, "r") 
  t, tq1, pos, vel, tq2 = np.loadtxt(file_path + file_name, usecols=(0, 1, 2, 3, 4), unpack=True)
#  print(t)

#  x = np.arange(0, 10, 0.2)
#  y = np.sin(x)
  fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
  ax1.plot(t, tq1, '*-', t, tq2, '*-', linewidth=0.5, markersize=0.5)
  ax2.plot(t, pos, '*-', t, vel, '*-', linewidth=0.5, markersize=0.5)
  plt.show()
    
