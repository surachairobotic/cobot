import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq
import numpy as np
import eq
import set_const
import copy


def cal_cg(q,vars):
  R,dR_dq,Kw,dKw_dq,J \
  , dJ_dq,dz_dq, M, I, g = vars

  com = set_const.COM
  com_pos = [None]*len(R)
  R2 = [None]*len(R)
  axis_pos = [None]*len(R)
  axis_tip = [None]*len(R)
  v0 = np.zeros(4)
  v0[3] = 1.0
  vz = np.zeros(4)
  vz[2] = 1.0
  vz[3] = 1.0
  for i in range(len(R)):
    '''
    if i==0:
      R2[i] = R[i]
    else:
      R2[i] = R[i].dot(R2[i-1])
    axis_pos[i] = (R2[i].dot(v0)) #[0:3]
    axis_tip[i] = (R2[i].dot(vz))[0:3]
    com_pos[i] = (R2[i].dot(com[i]))[0:3]
    '''
    axis_pos[i] = (R[i].dot(v0))[0:3]
    axis_tip[i] = (R[i].dot(vz))[0:3]
    com_pos[i] = (R[i].dot(com[i]))[0:3]
    
  torque = [0.0]*len(R)
  for i in range(len(R)):
    '''
    if i!=4:
      continue
    '''
    axis = axis_tip[i] - axis_pos[i]
    #print('a : '+str(axis_pos[i]))
    for j in range(i, len(R)):
      c = com_pos[j]
      force = np.array([0.0,0.0,-M[j]*g])
      force_on_axis = axis.dot(force) * axis
      force_on_plane = force - force_on_axis
      assert(abs(axis.dot(force_on_plane))<0.000001)
      com_on_plane = c - axis.dot(c - axis_pos[i]) * axis
      assert(abs(axis.dot(com_on_plane - axis_pos[i]))<0.000001)
      torque[i]-= np.cross(com_on_plane - axis_pos[i], force_on_plane).dot(axis)
      '''
      print('c : '+str(c))
      print('f : '+str(force))
      print('force_on_axis : '+str(force_on_axis))
      print('force_on_plane : '+str(force_on_plane))
      print('com_on_plane : '+str(com_on_plane))
      print('com_on_plane : '+str(np.cross(com_on_plane, force_on_plane)))
      exit()
      '''
    #print(torque[i])
    print(com_pos[i] - axis_pos[i])
  #print(torque)
  #print(axis_pos)
  #print(axis_tip)

def print_xyz(q,vars):
  R,dR_dq,Kw,dKw_dq,J \
  , dJ_dq,dz_dq, M, I, g = vars

  p = np.zeros(4)
  p[3] = 1.0
  for r in R:
    print(r.dot(p))


if __name__ == "__main__":
  q = np.zeros(6)
  q = np.array([-0.785443541965, 0.393923520632, -0.477382530168, -1.2518225519e-05, 0.0834862087235, -0.78538264936])
  torque = np.array([-0.000000, -5.570980, -2.766240, -0.046627, 0.000219, 0.000000])
  vars = eq.get_vars(q)
  cal_cg(q,vars)

#  print_xyz(q,vars)

