#!/usr/bin/env python


import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt

fname = 'q.txt'
joint_num = None

def load(fname):
  global joint_num
  points = []
  if not os.path.isfile(fname):
    print("file not exist : " + fname)
    return points
  with open(fname,'rt') as f:
    line = f.readline()
    while line:
      vals = line.split(' ')
      if joint_num is None:
        data_num = len(vals)
        joint_num = int((data_num-3)/3)
        print(data_num)
        print(joint_num)
      if len(vals)!=data_num:
        print('invalid val num : ' + str(len(vals)))
        print(line)
        exit()
      else:
        for i in range(len(vals)):
          vals[i] = float(vals[i])
        points.append(vals)
      line = f.readline()
  return np.array(points)

if __name__ == "__main__":
  if len(sys.argv)>1:
    fname = sys.argv[1]

  data = load(fname)
  t = data[:,0]
  q = data[:,1:(joint_num*3+1)]
  U = data[:,-2]
  T = data[:,-1]
  '''
  plt.plot(t,U+T)
  plt.grid(linestyle='-', linewidth='0.5')
  plt.show()
  exit()
  '''
  f, axarr = plt.subplots(4, sharex=True)
  for i in range(3):
    axarr[i].hold(True)

  for i in range(3):
    for j in range(joint_num):
      axarr[i].plot(t,q[:,j*3+i])
  axarr[3].plot(t,U,t,T,t,U+T)

  leg = []
  for i in range(4):
    axarr[i].grid(linestyle='-', linewidth='0.5')

  for i in range(joint_num):
    leg.append('q'+str(i))
  axarr[0].legend(leg)
  axarr[3].legend(['U', 'T', 'SUM'])

  axarr[0].set_ylabel('q [rad]')
  axarr[1].set_ylabel('dq [rad/s]')
  axarr[2].set_ylabel('ddq [rad/s^2]')
  axarr[3].set_xlabel('time [s]')

  plt.show()
