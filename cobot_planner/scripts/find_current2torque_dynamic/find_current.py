# -*- coding: utf-8 -*-

'''
find equation to estimate current from torque and dq
data : wave + constant velo
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
import find_torque_acc as fa



'''
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
'''


if __name__ == "__main__":
  dt = 0.020
  files, data_est = fa.load_data(5, False)
  
  if fv.n_joint==0:
    abc_range = [[100.0, 400.0], [0.0,2000.0], [0.0, 500.0]]
    #abc = [46.0, 1230.0, 250.0]
    abc = [355.0, 1240.0, 245.0]
  elif fv.n_joint==1:
    abc_range = [[-300.0, 0.0], [-1500.0,0.0], [-1000.0, 0.0]]
#    abc = [-168.0, -855.0, -390.0]
    abc = [-177.0, -690.0, -650.0]
  elif fv.n_joint==2:
    abc_range = [[100.0, 300.0], [400.0,800.0], [100.0, 300.0]]
    abc = [180.0, 564.0, 206.0]
  elif fv.n_joint==3:
    abc_range = [[100.0, 400.0], [500.0,800.0], [100.0, 400.0]]
    abc = [256.0, 689.0, 232.0]
  elif fv.n_joint==4:
    abc_range = [[100.0, 400.0], [0.0,100.0], [0.0, 100.0]]
#    abc_range = [[-200.0, 200.0], [-200.0,200.0], [-200.0, 200.0]]
#    abc = [250.0, 38.0, 46.0]
    abc = [241.0, 22.0, 58.0]
  elif fv.n_joint==5:
    abc_range = [[0.0, 0.0], [10.0, 100.0], [0.0,50.0], [20.0, 100.0]]
    abc = [0.0, 40.0, 15.0, 52.0]
  else:
    assert(0)

  #abc = fv.find_eq(data_est['torque'], data_est['dq'], data_est['current'], abc_range, 20)
  #abc = fv.find_eq_with_acc(data_est['torque'], data_est['ddq_f'], data_est['dq'], data_est['current'], abc_range, 20)
  

  for f in files:
    data = f['data']
    t = f['get_data_from_t_range'](data, data['t'])
    cur = f['get_data_from_t_range'](data, data['current'][:,fv.n_joint])
    cur_f = f['get_data_from_t_range'](data, data['current_f'][:,fv.n_joint])
    dq = f['get_data_from_t_range'](data, data['dq'][:,fv.n_joint])
    dq_f = f['get_data_from_t_range'](data, data['dq_f'][:,fv.n_joint])
    tq = f['get_data_from_t_range'](data, data['torque'][:,fv.n_joint])
    ddq_f = f['get_data_from_t_range'](data, data['ddq_f'][:,fv.n_joint])


    bias = fv.create_bias(dq_f)
    if len(abc)==3:
      data['current_est'] = abc[0]*tq + abc[1]*dq_f + abc[2]*bias
      data['torque_est'] = (cur_f - (abc[1]*dq_f + abc[2]*bias))/abc[0]
    else:
      data['current_est'] = abc[0]*tq + abc[1]*ddq_f + abc[2]*dq_f + abc[3]*bias
      data['torque_est'] = (cur_f - (abc[1]*ddq_f + abc[2]*dq_f + abc[3]*bias))/abc[0]


    # for plotting

    # plot
    _, axarr = plt.subplots(5, sharex=True)
    for i in range(len(axarr)):
      axarr[i].hold(True)
      axarr[i].grid(linestyle='-', linewidth='0.5')

    axarr[0].plot(data['t'],data['q'][:,fv.n_joint])
    axarr[1].plot(data['t'],data['dq'][:,fv.n_joint], t, dq_f)
    axarr[2].plot(data['t'],data['ddq'][:,fv.n_joint], t, ddq_f)
    axarr[3].plot(t, tq, t, data['torque_est'])
#    axarr[2].plot(data['t'],data['current'][:,fv.n_joint], t, cur_f)
    '''
    if f['b_filter_current']:
      c = cur_f
    else:
      c = cur
    '''
    axarr[4].plot(t, cur, t, cur_f, t, data['current_est'], 'k')

    axarr[0].set_ylabel('q [rad]')
    axarr[1].set_ylabel('dq [rad/s]')
    axarr[2].set_ylabel('ddq [rad/s2]')
    axarr[3].set_ylabel('torque [N-m]')
    axarr[4].set_ylabel('current [mA]')
#    axarr[0].legend(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    axarr[3].legend(['torque', 'estimate'])
    axarr[4].legend(['current', 'current_filter', 'estimate'])
    axarr[len(axarr)-1].set_xlabel('time [s]')
    #break
  plt.show()
