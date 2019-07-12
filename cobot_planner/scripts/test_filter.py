#!/usr/bin/env python


import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt


#pre = '/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_wave_j3/'
pre = 'bag2txt_wave_j3/'
fname = pre + 'torque.txt'
fname_q = pre + 'joint_states.txt'

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

def limit_data(data, t, min, max):
  i1 = 0
  i2 = 0
  for i in range(len(t)):
    if t[i]>min and i1==0:
      i1 = i
    elif t[i]>max:
      i2 = i
      break
  return data[i1:i2,:]


if __name__ == "__main__":
  if len(sys.argv)>1:
    fname = sys.argv[1]

  data = load(fname_q)
  data = limit_data(data, data[:,0], 4.2, 13.5)
  t = data[:,0]
  q = data[:,1:7]
  dq = data[:,7:13]
  effort = data[:,13:19]

  data = load(fname)
  data = limit_data(data, data[:,0], 4.2, 13.5)
  data[:,1:7] *= directions
  t = data[:,0]
  torque = data[:,1:7]# * directions
  current = data[:,7:13]
  ddq = data[:,13:19]

  n_joint = 2
  best_k = None
  min_err = 1.0e6

#  k = [[-0.01,0.01,100], [-5.0,5.0,100], [-5.0,5.0,100]]
#  best : 0.04058727522262602 : [0.0003999999999999993, -0.2999999999999998, -2.5]

  k = [[-0.02,0.02,20], [-10.0,10.0,20], [-10.0,0.0,20], [0.0,2.0, 20]]
  for i1 in range(k[0][2]):
    k_current = k[0][0] + i1*(k[0][1]-k[0][0])/k[0][2]
    for i2 in range(k[1][2]):
      k_velo = k[1][0] + i2*(k[1][1]-k[1][0])/k[1][2]
      for i3 in range(k[2][2]):
        k_const = k[2][0] + i3*(k[2][1]-k[2][0])/k[2][2]
        for i4 in range(k[3][2]):
          k_ddq = k[3][0] + i4*(k[3][1]-k[3][0])/k[3][2]
          err = np.linalg.norm(torque[:,n_joint] - (current[:,n_joint]*k_current + k_velo*dq[:,n_joint] + k_const + k_ddq*ddq[:,n_joint]))
          if err<min_err:
            min_err = err
            best_k = [k_current, k_velo, k_const, k_ddq]
            print(str(min_err/len(t))+' : '+str(best_k))
    print(i1)

  print('best : ' + str(min_err/len(t))+' : '+str(best_k))
  data2 = [torque[:,n_joint], (current[:,n_joint]*best_k[0] +
    best_k[1]*dq[:,n_joint] +
    best_k[2] +
    best_k[3]*ddq[:,n_joint])]
  f, axarr = plt.subplots(2, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')

  axarr[0].plot(t,data2[0],t,data2[1])
  axarr[1].plot(t,current[:,n_joint]*best_k[0], t, best_k[1]*dq[:,n_joint], t, best_k[2]*np.array([1.0]*len(t)), t, best_k[3]*ddq[:,n_joint])
  axarr[0].legend(['torque', 'torque_current'])
  axarr[1].legend(['current', 'velo', 'const'])

  axarr[0].set_ylabel('torque [N-m]')
  axarr[1].set_ylabel('torque_current [N-m]')
  axarr[len(axarr)-1].set_xlabel('time [s]')
  plt.show()
