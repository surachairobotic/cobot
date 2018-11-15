# -*- coding: utf-8 -*-

'''
estimate torque with the equation got from find_torque_velo.py
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
  '''

  dir = 'bag2txt_j5_v'
  for v in [30,60,90]:
    fnames.append(dir+str(v)+'/joint_states.txt')
  '''
  for fname in fnames:
    data = fv.load_file(fname)
    fv.get_torque(data, fname, b_filter_dq=True, b_filter_ddq=True, b_torque_acc=True)
    #data = fv.load_file(fname,b_filter=True, b_torque_acc=True)
    cur = data['current'][:,fv.n_joint]
    dq = data['dq'][:,fv.n_joint]
    cur_f = data['current_f'][:,fv.n_joint]
    dq_f = data['dq_f'][:,fv.n_joint]
    tq = data['torque'][:,fv.n_joint]

    bias = fv.create_bias(dq_f)
    data['torque_est'] = abc[0]*cur_f + abc[1]*dq_f + abc[2]*bias
    # for plotting

    # plot
    f, axarr = plt.subplots(4, sharex=True)
    for i in range(len(axarr)):
      axarr[i].hold(True)
      axarr[i].grid(linestyle='-', linewidth='0.5')

    axarr[0].plot(data['t'],data['q'][:,fv.n_joint])
    axarr[1].plot(data['t'],data['dq'][:,fv.n_joint], data['t'], dq_f)
    axarr[2].plot(data['t'],data['current'][:,fv.n_joint], data['t'], cur_f)
    axarr[3].plot(data['t'], data['torque_est'] , data['t'], tq)

    axarr[0].set_ylabel('q [rad]')
    axarr[1].set_ylabel('dq [rad/s]')
    axarr[2].set_ylabel('current [mA]')
    axarr[3].set_ylabel('torque [N-m]')
    axarr[0].legend(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    axarr[3].legend(['estimate', 'calculation'])
    axarr[len(axarr)-1].set_xlabel('time [s]')
    #break
  plt.show()
