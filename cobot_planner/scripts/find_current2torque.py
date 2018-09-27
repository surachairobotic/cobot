#!/usr/bin/env python

import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt

import eq
from test_cal_torque import cal_torque

fname = 'running_torque/torque_j5_all.txt'
n_joint = 4

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


class LSM:
  def fit(self,x,y):
    xy = np.sum(x*y)
    nx = np.sum(x)
    ny = np.sum(y)
    xx = np.sum(x*x)
    n = len(x)

    div = n*xx - nx**2
    a = (n*xy - nx*ny)/div
    b = (xx*ny - xy*nx)/div
    return a,b

  def get_error(self,x,y,a,b):
    err = y - self.get_y(x,a,b)
    return np.mean(abs(err))

  def get_y(self,x,a,b):
    return a*x+b


if __name__ == "__main__":
  if len(sys.argv)>1:
    fname = sys.argv[1]

  data = load(fname)
  t = data[:,0]
  goal_torque_up = data[:,1]
  effort_up = data[:,2]
  goal_torque_down = data[:,3]
  effort_down = data[:,4]
  q = data[:,5:11]
  q_rad = q*math.pi/180.0
  effort_mean = (effort_up + effort_down)*0.5
  effort_fric = abs((effort_up - effort_down)*0.5)

  # cal torque
  dq = np.array([0.0]*6)
  ddq = dq
  torque_cal = []
  for i in range(q_rad.shape[0]):
    q2 = q_rad[i,:]
    vars = eq.get_vars(q2)
    t1, t2 = cal_torque(q2, dq, ddq, vars)
    torque_cal.append(t1)
  torque_cal = np.array(torque_cal)

  # cal current 2 torque
  lsm = LSM()
  a,b = lsm.fit(effort_mean, torque_cal[:,n_joint])
  err = lsm.get_error(effort_mean, torque_cal[:,n_joint], a, b)
  torque_est = lsm.get_y(effort_mean,a,b)
  torque_fric = a*effort_fric + b
  print('current2torque : a = %f, b = %f, fric = %f' % (a,b,np.mean(torque_fric)))
  print('mean error [N-m]: '+str(err))


  f, axarr = plt.subplots(2, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')

  axarr[0].plot(q[:,n_joint], effort_up, '*-', q[:,n_joint], effort_down, '*-')
  axarr[0].plot(q[:,n_joint], effort_up - effort_down, '*-')
  axarr[1].plot(q[:,n_joint], torque_cal[:,n_joint], '*-'
    ,q[:,n_joint], torque_est, '*-',q[:,n_joint], torque_fric, '*-')

  axarr[0].legend(['goal_current_up', 'goal_current_down', 'diff_up_down'])
  axarr[0].set_ylabel('torque [N-m]')
  axarr[1].legend(['torque', 'torque_est'])
  axarr[1].set_ylabel('torque [N-m]')
  axarr[len(axarr)-1].set_xlabel('q'+str(n_joint+1)+' [degree]')
  plt.show()
