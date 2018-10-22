
import sys
sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')
sys.path.insert(0,'/home/tong/catkin_ws/src/cobot/cobot_planner/scripts')


import sympy as sp
from numpy import *
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq
import eq
import matplotlib.pyplot as plt
import find_rotation_matrix


data_num = 100

if __name__ == "__main__":
  R = mat([[ 0.85907309, -0.02061672,  0.51143756],
    [ 0.51161294,  0.00399138, -0.85920677],
    [ 0.01567268,  0.99977949,  0.01397667]])
  t = mat([-0.26211133, -0.18565007, -0.01812064])
  names = ['RigidBody-Endeffector:Marker1',
    'RigidBody-Endeffector:Marker2',
    'RigidBody-Endeffector:Marker3']

  pnt, t1 = find_rotation_matrix.get_data('Take 2018-10-19 Test 01.csv', names, data_num)

  markers = []
  for i in range(len(names)):
    markers.append(mat(pnt[:,i*3:(i+1)*3]))

  t = tile(t.T, (1, len(pnt)))
  p1 = []
  for marker in markers:
    p1.append(R*marker.T + t)

  tip_z = 0.005+0.00536
  tips = array([[0.0, 0.03, tip_z]
    , [0.03, 0.0, tip_z]
    , [0.0, -0.04, tip_z]])
    
  p2 = [[],[],[]]
  t2 = []
  with open('bag2txt_2018-10-19-14-15-08/joint_states.txt', 'rt') as f:
    n_line = 0
    for line in f:
      n_line+=1
      arr = line.strip().split(' ')
      if len(arr)==19:
        q = array([float(a) for a in arr[1:7]])
        Rq = eq.get_R(q)
        for i in range(3):
          xyz = Rq[5].dot(append(tips[i],1.0))
          p2[i].append(xyz)
        t2.append(float(arr[0]))
        
      if n_line>data_num:
        break
        
  for i in range(3):
    p2[i] = array(p2[i])
  t1 = array(t1)
  t2 = array(t2) - t2[0]
  
  f, axarr = plt.subplots(3, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')
  for i in range(3):
    for j in range(3):
      '''
      print(len(t1))
      print(p1[i].shape)
      print(len(t2))
      print(p2[i].shape)
      '''
      axarr[i].plot(t1,p1[i][j,:].T,t2,p2[i][:,j])

  axarr[0].set_xlabel('time [s]')
  plt.show()

