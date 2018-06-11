#!/usr/bin/env python

# plot velo

import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt

fname = 'data/velo'
t0 = None

def load(fname, data_num):
  global t0
  points = []
  if not os.path.isfile(fname):
    print("file not exist : " + fname)
    return points
  with open(fname,'rt') as f:
    line = f.readline()
    while line:
      vals = line.split(' ')
      if len(vals)!=data_num:
        print('invalid val num : ' + str(len(vals)))
#        exit()
      else:
        for i in range(len(vals)):
          vals[i] = float(vals[i])
          
        if t0 is None:
          t0 = vals[0]
        vals[0]-= t0
        points.append(vals)
  #      points.append([vals[0] - t0, math.sqrt(vals[1]**2 + vals[2]**2 + vals[3]**2)])
      line = f.readline()
  return np.array(points)

def cal_norm(points):
  p = []
  for i in range(len(points)):
    p.append( [ points[i][0], math.sqrt(points[i][1]**2 + points[i][2]**2 + points[i][3]**2)] )
  return np.array(p)
  
if __name__ == "__main__":
  if len(sys.argv)>1:
    fname = sys.argv[1]
  s = load('data.txt', 19)
  t = s[:,0]
  q = s[:,1:7]
  dq = s[:,7:13]
  effort = s[:,13:19]
  n_joint = len(q[0])

  f, axarr = plt.subplots(3, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')
  
  for i in range(n_joint):
    axarr[0].plot(t, q[:,i], '+-')
    axarr[1].plot(t, dq[:,i], '+-')
    axarr[2].plot(t, effort[:,i], '+-')
  
  leg = [('q'+str(i)) for i in range(n_joint)]
  axarr[0].legend(leg)
  axarr[0].set_ylabel('q [rad]')
  axarr[1].set_ylabel('dq [rad/s]')
  axarr[2].set_ylabel('effort')
  axarr[2].set_xlabel('time [s]')
  plt.show()
  print('end')
