#!/usr/bin/env python


import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt

fname = '/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_2018-09-03-11-26-25/torque.txt'
fname_q = '/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_2018-09-03-11-26-25/joint_states.txt'

directions = np.array([1,-1,1,1,1,1])


def load(fname):
  points = []
  if not os.path.isfile(fname):
    print("file not exist : " + fname)
    return points
  with open(fname,'rt') as f:
    for line in f:
      vals = line.strip().split(' ')
      if len(vals)>0:
        points.append([float(x) for x in vals])
  return np.array(points)

if __name__ == "__main__":
  leg = []
  for i in range(3):
    leg.append('q'+str(i))
  
    
    
  pre = '/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_'
  for i in range(3,4):
    f, axarr = plt.subplots(3, sharex=True)
    for l in range(3):
      axarr[l].hold(True)
      axarr[l].grid(linestyle='-', linewidth='0.5')
      axarr[l].legend(leg)
    axarr[0].set_ylabel('torque [N-m]')
    axarr[1].set_ylabel('current [mA]')
    axarr[2].set_ylabel('ddq [rad/s^2]')
    axarr[2].set_xlabel('time [s]')
    for j in [10]:
      for k in range(1,2):
        fname = pre + 'j%d_v%02d_s%d/torque.txt' % (i,j,k)
        print(fname)
  
        data = load(fname)
        data[:,1:7] *= directions
        t = data[:,0]
        torque = data[:,1:7]# * directions
        current = data[:,7:13]
        ddq = data[:,13:19]
        for i1 in range(3):
          i2 = i-1
          axarr[i1].plot(t,data[:,i2+i1*6+1])
    for l in range(3):
      axarr[l].legend(leg)
  plt.show()
