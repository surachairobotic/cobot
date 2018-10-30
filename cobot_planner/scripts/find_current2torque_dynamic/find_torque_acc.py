# -*- coding: utf-8 -*-

'''
estimate torque with the equation got from find_torque.py
'''

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
from test_cal_torque import cal_torque
import matplotlib.pyplot as plt
import copy
from scipy import signal
import os
import find_torque_velo as fv


abc = [0.0043, -0.03200000000000003, -0.32000000000000006]


def get_t_range(data):
  dq = data['dq_f']
  n = 10
  for i in range(n,len(dq)):
    dq2 = dq[i-n:i, fv.n_joint]
    if max(dq2)-min(dq2) > 0.01:
      i1 = i+100
      for j in range(i1,len(dq)):
        dq2 = dq[j-n:j, fv.n_joint]
        if max(dq2)-min(dq2) < 0.01:
          i2 = j-100
          data['t_range'] = [i1, i2]
          return

def get_data_from_t_range(data, data_range):
  t = data['t_range']
  if len(data_range.shape)==1:
    return data_range[t[0]:t[1]]
  else:
    return data_range[t[0]:t[1],:]


# w = abc[0]*x + abc[1]*y + abc[2]*x + abc[3]
def find_eq( x, y, z, w, range_abc, n_loop ):
  abc = []
  dabc = []
  min_err = 99999999.0
  for i in range(4):
    dabc.append( (range_abc[i][1] - range_abc[i][0])/float(n_loop) )

  bias = fv.create_bias(y)
  for i in range(n_loop):
    a = range_abc[0][0] + dabc[0]*i
    for j in range(n_loop):
      b = range_abc[1][0] + dabc[1]*j
      for k in range(n_loop):
        c = range_abc[2][0] + dabc[2]*k
        for l in range(n_loop):
          d = range_abc[3][0] + dabc[3]*k

          w2 = a*x + b*y + c*z + d*bias
          e = linalg.norm(w2-w)
          if e<min_err:
            min_err = e
            abc = [a,b,c,d]
            print('['+str((i*(n_loop**3) + j*(n_loop**2) + k*n_loop + l)/float(n_loop**4))+'] '+str(e/len(x))+' : '+str(abc))
  return abc

def get_ddq(data):
  dq = data['dq_f']
  ddq = zeros(dq.shape)
  t = data['t']
  dt2 = (t[1:] - t[:len(t)-1])
  for i in range(dq.shape[1]):
    ddq[1:,i] = (dq[1:,i] - dq[:dq.shape[0]-1,i]) / dt2
  data['ddq_f'] = ddq

if __name__ == "__main__":
  dt = 0.020
  fnames = []


  dir = 'bag2txt_j5_'
  hzs = ['0.5', '1']
  for hz in hzs:
    for s in range(1,3):
      f = dir + hz+'hz_s'+str(s)+'/joint_states.txt'
      if not os.path.isfile(f):
        raise Exception('Invalid file : '+f)
      fnames.append(f)

  for fname in fnames:
    data = fv.load_file(fname,b_filter=True, b_torque_acc=True)
    get_t_range(data)
    get_ddq(data)

    t = get_data_from_t_range(data, data['t'])
    cur = get_data_from_t_range(data, data['current'][:,fv.n_joint])
    cur_f = get_data_from_t_range(data, data['current_f'][:,fv.n_joint])
    dq = get_data_from_t_range(data, data['dq'][:,fv.n_joint])
    dq_f = get_data_from_t_range(data, data['dq_f'][:,fv.n_joint])
    tq = get_data_from_t_range(data, data['torque'][:,fv.n_joint])
    ddq_f = get_data_from_t_range(data, data['ddq_f'][:,fv.n_joint])

    bias = fv.create_bias(dq_f)
    abc = find_eq( cur, dq_f, ddq_f, tq, [[0.002, 0.007], [-0.8,0.8], [-0.8, 0.8], [-3.0, 3.0]], 20)
    data['torque_est'] = abc[0]*cur_f + abc[1]*dq_f + abc[2]*ddq_f + abc[3]*bias
    # for plotting

    # plot
    f, axarr = plt.subplots(4, sharex=True)
    for i in range(len(axarr)):
      axarr[i].hold(True)
      axarr[i].grid(linestyle='-', linewidth='0.5')

    axarr[0].plot(data['t'],data['q'][:,fv.n_joint])
    axarr[1].plot(data['t'],data['dq'][:,fv.n_joint], t, dq_f)
    axarr[2].plot(data['t'],data['current'][:,fv.n_joint], t, cur_f)
    axarr[3].plot(t, data['torque_est'] , t, tq)

    axarr[0].set_ylabel('q [rad]')
    axarr[1].set_ylabel('dq [rad/s]')
    axarr[2].set_ylabel('current [mA]')
    axarr[3].set_ylabel('torque [N-m]')
    axarr[0].legend(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    axarr[3].legend(['estimate', 'calculation'])
    axarr[len(axarr)-1].set_xlabel('time [s]')
    break
  plt.show()
