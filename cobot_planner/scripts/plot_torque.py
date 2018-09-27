#!/usr/bin/env python


import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt


#pre = '/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_wave_j3/'
pre = 'bag2txt_j5_1hz_s1/'
#pre = 'bag2txt_wave_j3_1hz_s1/'
#pre = 'bag2txt_wave_j3_1hz_s2/'

fname = pre + 'torque.txt'
fname_q = pre + 'joint_states.txt'
n_joint = None

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
  if len(sys.argv)>1:
    fname = sys.argv[1]
  leg = []
  for i in range(6):
    leg.append('q'+str(i))


  data = load(fname_q)
  t = data[:,0]
  q = data[:,1:7]
  dq = data[:,7:13]
  effort = data[:,13:19]
  '''
  f, axarr = plt.subplots(3, sharex=True)
  for i in range(3):
    axarr[i].hold(True)

  for i in range(3):
    for j in range(6):
      axarr[i].plot(t,data[:,j+i*6+1])

  for i in range(3):
    axarr[i].grid(linestyle='-', linewidth='0.5')
    axarr[i].legend(leg)

  axarr[0].set_ylabel('q [rad]')
  axarr[1].set_ylabel('dq [rad/s]')
  axarr[2].set_ylabel('effort')
  axarr[2].set_xlabel('time [s]')

  '''
  data = load(fname)
  data[:,1:7] *= directions
  t2 = data[:,0]
  torque = data[:,1:7]# * directions
  current = data[:,7:13]
  ddq = data[:,13:19]
  torque_no_ddq = data[:,19:25]
  xyz = data[:,25:28]

  f, axarr = plt.subplots(7, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')

  arr_data = [[t,q],[t,dq],[t2,ddq],[t2,torque],[t2,current],[t2, ddq*0.08645460999999999]]
  for i in range(len(arr_data)):
    if n_joint is None:
      for j in range(6):
        axarr[i].plot(arr_data[i][0],arr_data[i][1][:,j])
    else:
      axarr[i].plot(arr_data[i][0],arr_data[i][1][:,n_joint])

  axarr[6].plot(t2,xyz[:,0],t2,xyz[:,1],t2,xyz[:,2])
  
  '''
  if data.shape[1]>20:
    for j in range(6):
      axarr[3].plot(t2,torque_no_ddq[:,j])
  '''
  if n_joint is None:
    axarr[0].legend(leg)
  else:
    axarr[0].legend(leg[n_joint])

  axarr[0].set_ylabel('q [rad]')
  axarr[1].set_ylabel('dq [rad/s]')
  axarr[2].set_ylabel('ddq [rad/s^2]')
  axarr[3].set_ylabel('torque [N-m]')
  axarr[4].set_ylabel('current [mA]')
  axarr[len(axarr)-1].set_xlabel('time [s]')


  '''
  f, axarr = plt.subplots(3, sharex=True)
  for i in range(3):
    axarr[i].hold(True)

  for i in range(3):
    for j in range(6):
      axarr[i].plot(t,data[:,j+i*6+1])

  for i in range(3):
    axarr[i].grid(linestyle='-', linewidth='0.5')
    axarr[i].legend(leg)

  axarr[0].set_ylabel('torque [N-m]')
  axarr[1].set_ylabel('current [mA]')
  axarr[2].set_ylabel('ddq [rad/s^2]')
  axarr[2].set_xlabel('time [s]')
  '''

  plt.show()
