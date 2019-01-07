#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt
import numpy as np

import eq
from spectrum import *
from scipy import signal

file_path = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/"
b1, a1 = signal.butter(1, 1.5/((1/0.017)/2.0), btype='lowpass', analog=False)
b2, a2 = signal.butter(2, 1.5/((1/0.017)/2.0), btype='lowpass', analog=False)
b3, a3 = signal.butter(3, 1.5/((1/0.017)/2.0), btype='lowpass', analog=False)
b4, a4 = signal.butter(4, 1.5/((1/0.017)/2.0), btype='lowpass', analog=False)
b5, a5 = signal.butter(5, 1.5/((1/0.017)/2.0), btype='lowpass', analog=False)
  
if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)
  
  t = []
  pos = []
  vel = []
  cur = []
  cur_cal = []
  fA1 = []
  fB1 = []
  fA2 = []
  fB2 = []
  fA3 = []
  fB3 = []
  fA4 = []
  fB4 = []
  fA5 = []
  fB5 = []
  err = []
  
  file_all = "tq_j0_sin.txt"
  data_all = open(file_path + file_all, "r").read().splitlines()
  tq1 = []
  tq2 = []
  
  for data in data_all:
    xxx = data.split(',');
    t.append(float(xxx[0]))
    pos.append(float(xxx[1]))
    vel.append(float(xxx[2]))
    cur.append(float(xxx[3]))
    cur_cal.append(float(xxx[4]))
    tq1.append(float(xxx[5]))
    tq2.append(float(xxx[6]))
    cur1 = signal.lfilter(b1, a1, cur)
    cur2 = signal.lfilter(b2, a2, cur)
    cur3 = signal.lfilter(b3, a3, cur)
    cur4 = signal.lfilter(b4, a4, cur)
    cur5 = signal.lfilter(b5, a5, cur)
    cur_cal1 = signal.lfilter(b1, a1, cur_cal)
    cur_cal2 = signal.lfilter(b2, a2, cur_cal)
    cur_cal3 = signal.lfilter(b3, a3, cur_cal)
    cur_cal4 = signal.lfilter(b4, a4, cur_cal)
    cur_cal5 = signal.lfilter(b5, a5, cur_cal)
    err = abs(cur5-cur_cal5)
#    fA1.append(y1[len(y1)-1])
#    fB1.append(y2[len(y2)-1])
#    err.append(y3[len(y3)-1])
#    y1 = signal.lfilter(b1, a1, cur)
#    y2 = signal.lfilter(b1, a1, cur_cal)

  fA10 = signal.filtfilt(b1, a1, cur)
  fB10 = signal.lfilter(b1, a1, cur)
  fA11 = signal.filtfilt(b2, a2, cur)
  fB11 = signal.lfilter(b2, a2, cur)
  fA12 = signal.filtfilt(b3, a3, cur)
  fB12 = signal.lfilter(b3, a3, cur)
  fA13 = signal.filtfilt(b4, a4, cur)
  fB13 = signal.lfilter(b4, a4, cur)
  fA14 = signal.filtfilt(b5, a5, cur)
  fB14 = signal.lfilter(b5, a5, cur)

#    if len(t) > 250:
#      break
    
#  print("%d,%d,%d,%d,%d,%d,%d,%d" % (len(t), len(pos), len(vel), len(cur), len(cur_cal), len(fA1), len(fB1), len(err)))
#  print(t)

  ### histrogram of (dt)
  fig1, axarr = plt.subplots(2, 1)
  dt = np.array(t[1:]) - np.array(t[0:-1])
  dtmax = int(dt.max()*1000)
  dtmin = int(dt.min()*1000)
  print("max = %f, min = %f" % (dt.max(), dt.min()))
  print("max = %d, min = %d" % (dtmax, dtmin))
  tmp = range(dtmin, dtmax, 1)
  val = []
  for i in tmp:
    val.append(i/1000.0)
  freq = np.zeros(len(val), dtype=int)
  for i in dt:
    for j in range(len(val)):
      if i>=val[j] and i<val[j]+0.001:
        freq[j] += 1
  axarr[0].grid(linestyle='-', linewidth='1.0')
  axarr[0].plot(val, freq)
  
  ###  spectrum
  f1x, f1y = data2spectrum(cur, t)
  f2x, f2y = data2spectrum(cur1, t)
  f3x, f3y = data2spectrum(cur2, t)
  axarr[1].grid(linestyle='-', linewidth='1.0')
  axarr[1].plot(f1x, f1y, f2x, f2y, f3x, f3y)
  axarr[1].legend(['current', 'cur1', 'cur2'])
  ###

  fig2, axarr = plt.subplots(3, 1, sharex=True)
  axarr[0].plot(t, cur, t, cur1, t, cur2, t, cur3, t, cur4, t, cur5)
  axarr[0].legend(['cur', 'lfilter1', 'lfilter2', 'lfilter3', 'lfilter4', 'lfilter5'])
#  axarr[0].plot(t, cur, t, cur3)
#  axarr[0].legend(['cur', 'lfilter'])
  axarr[1].plot(t, cur_cal, t, cur_cal1, t, cur_cal2, t, cur_cal3, t, cur_cal4, t, cur_cal5)
  axarr[1].legend(['cur_cal', 'lfilter1', 'lfilter2', 'lfilter3', 'lfilter4', 'lfilter5'])
#  axarr[1].plot(t, cur_cal, t, cur_cal3)
#  axarr[1].legend(['cur_cal', 'lfilter'])
  axarr[2].plot(t, cur5, t, cur_cal5, t, err)
  axarr[2].legend(['cur', 'cur_cal', 'err'])
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='1.0')

#  f, axarr = plt.subplots(1, sharex=True)

#  axarr[0].plot(t, pos, t, vel)
#  axarr[1].plot(t, cur, t, fB1)
#  axarr[2].plot(t, cur_cal, t, fA1, t, fA2, t, fA3, t, fA4, t, fA5)
#  axarr[3].plot(t, fB1, t, fA1, t, err)
#  axarr[3].plot(fq1, fq2, fq3, fq4)


#  ax1.plot(t, cur, t, fA10, t, fA11, t, fA12, t, fA13, t, fA14)
#  ax1.legend(['current', 'f1', 'f2', 'f3', 'f4', 'f5'])

#  fb = []
#  t2 = []
#  for i in range(len(t)):
#      if t[i] > 0.70:
#        fb.append(fB4[i])
#        t2.append(t[i]-0.70)

#  ax1.plot(t, cur, t, fB4)
#  ax1.legend(['current', 'f1', 'f2', 'f3', 'f4', 'f5'])

  plt.show()
  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

