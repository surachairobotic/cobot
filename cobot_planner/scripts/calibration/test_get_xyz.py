#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')
sys.path.insert(0,'/home/tong/catkin_ws/src/cobot/cobot_planner/scripts')

import math
import eq
import set_const
import numpy as np

off_q = np.array([-0.038397, 0.010472, -0.012218, -0.008727, 0.008727, 0.000000])
off_bq = np.array([0.003491, 0.013963, 0.000000])
off_R = None
L6 = np.array( set_const.L[6]+[1.0] )

def rx(q):
  c = math.cos(q)
  s = math.sin(q)
  return np.array([
    [1, 0, 0],
    [0, c,-s],
    [0, s, c],
  ])

def ry(q):
  c = math.cos(q)
  s = math.sin(q)
  return np.array([
    [c, 0, s],
    [0, 1, 0],
    [-s, 0, c],
  ])

def get_R(off_bq):
  Rx = rx( off_bq[0] )
  Ry = ry( off_bq[1] )
  return Ry.dot(Rx)

def get_xyz(q):
  global off_R, L6
#  R = eq.get_R(q)
#  return R[5].dot(np.array( np.array( set_const.L[6]+[1.0])))[0:3]
  R = eq.get_R(q + off_q)
  return off_R.dot( R[5].dot(L6)[0:3] )


if __name__ == "__main__":
  off_R = get_R(off_bq)
  old_off_1 = np.array([0.0, -0.016406, -0.009949, 0.028798, -0.000698, 0.0])
  old_off_2 = np.array([-0.04363323,  0.038048,-0.00384,-0.036303,0.011519,0.0])
  pos = np.array([[-0.489069,0.566022,0.741561,-0.022339,-1.298750,-0.467469]
    , [-0.489069,0.566022,0.741561,-0.022339,-1.298750,-0.467469]
    , [-0.665501,0.821970,0.338862,0.252136,-0.518435,-0.747735]
    , [-0.117053,0.728471,0.607378,-0.800046,-0.829833,0.528449]]
  )
  for i in range(pos.shape[0]):
    xyz = get_xyz(pos[i,:] - old_off_1 - old_off_2)
    print(xyz)

