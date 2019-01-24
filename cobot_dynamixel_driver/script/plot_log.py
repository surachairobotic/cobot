#!/usr/bin/env python

import sys
import os
import math
import rospy
import numpy as np
import matplotlib.pyplot as plt

file_path = "/home/mtec/catkin_ws/src/cobot/cobot_dynamixel_driver/log/"
file_all  = "2019-01-24T09:24:31.346374_driver_log.txt"

if __name__ == "__main__":
  rospy.init_node('plot_tq', anonymous=True)
  data_all = open(file_path + file_all, "r").read().splitlines()

  s = []
  n = len(data_all)
  for data in data_all:
    s.append(map(float, data.split(',')));
  ss = np.array(s)
  t = ss[:,0]
  seq = ss[:,1]
  pos = np.array(ss[:,2:7])
  vel = np.array(ss[:,8:13])
  eff = np.array(ss[:,14:19])
  volt = np.array(ss[:,20:25])
  temp = np.array(ss[:,26:31])
#  exit()

  f, axarr = plt.subplots(5, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')
  
  indx = 2
  axarr[0].plot(t, pos[:,indx]*180/3.1416)
  axarr[1].plot(t, vel[:,indx]*180/3.1416)
  axarr[2].plot(t, eff[:,indx])
  axarr[3].plot(t, volt[:,indx])
  axarr[4].plot(t, temp[:,indx])
  
  axarr[0].legend("pos")
  axarr[0].set_xlabel('time [s]')
  axarr[1].legend('vel')
  axarr[1].set_xlabel('time [s]')
  axarr[2].legend('eff')
  axarr[3].set_xlabel('time [s]')
  axarr[3].legend('volt')
  axarr[3].set_xlabel('time [s]')
  axarr[4].legend('temp')
  axarr[4].set_xlabel('time [s]')
  plt.show()
  print('end')
