
import sys
sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')

import sympy as sp
from numpy import *
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq
import numpy as np
import eq

import find_rotation_matrix


if __name__ == "__main__":
  R = mat([[ 0.85907309, -0.02061672,  0.51143756],
    [ 0.51161294,  0.00399138, -0.85920677],
    [ 0.01567268,  0.99977949,  0.01397667]])
  t = mat([-0.26211133, -0.18565007, -0.01812064])
  names = ['RigidBody-Endeffector:Marker1',
    'RigidBody-Endeffector:Marker2',
    'RigidBody-Endeffector:Marker3']

  pnt, err_marker = find_rotation_matrix.get_data('Take 2018-10-19 Test 01.csv', names, 1000)

  markers = []
  for i in range(len(names)):
    markers.append(mat(pnt[:,i*3:(i+1)*3]))

  t2 = tile(t.T, (1, len(pnt)))
  markers2 = []
  for marker in markers:
    markers2.append(R*marker.T + t2)

  tip_z = 0.005+0.00536
  tips = array([[0.0, 0.03, tip_z]
    , [0.03, 0.0, tip_z]
    , [0.0, -0.04, tip_z]])
  with open('bag2txt_2018-10-19-14-15-08/joint_states.txt', 'rt') as f:
    for line in f:
      arr = line.strip().split(' ')
      if len(arr)==19:
        q = array([float(a) for a in arr[1:7]])
        Rq = eq.get_R(q)
        for i in range(3):
          xyz = Rq[5].dot(np.array(tips[i]))
        
        break
#  Rq = lib_equation.get_R()
