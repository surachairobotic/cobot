# -*- coding: utf-8 -*-

'''
find equation to estimate torque from current and dq
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


abc = [0.00272, -0.06, -0.15999999999999998]
abc = [0.00020000000000000052, -0.21599999999999997, -0.020000000000000018]

def get_t_range(data):
  dq = data['dq_f']
  n = 10
  '''
  for i in range(n,len(dq)):
    dq2 = dq[i-n:i, fv.n_joint]
    if max(dq2)-min(dq2) > 0.01:
      i1 = i+100
      for j in range(i1,len(dq)):
        dq2 = dq[j-n:j, fv.n_joint]
        if max(dq2)-min(dq2) < 0.05:
          i2 = j-100
          data['t_range'] = [i1, i2]
          return
  '''
  for i in range(n,len(dq)):
    dq2 = dq[i-n:i, fv.n_joint]
    if max(dq2)-min(dq2) > 0.01:
      i1 = i+100
      break
  for i in range(len(dq)-1, n-1, -1):
    dq2 = dq[i-n:i, fv.n_joint]
    if max(dq2)-min(dq2) > 0.01:
      i2 = i-100
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


def check_data_est(data_est):
  f, axarr = plt.subplots(3, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')

  axarr[0].plot(data_est['dq'])
  axarr[1].plot(data_est['current'])
  axarr[2].plot(data_est['torque'])

  axarr[0].set_ylabel('dq [rad/s]')
  axarr[1].set_ylabel('current [mA]')
  axarr[2].set_ylabel('torque [N-m]')
  plt.show()
  exit()


def load_data(n_joint):
  files = []
  fv.n_joint = n_joint
  
  if fv.n_joint==0:
    abc_range = [[-0.01, 0.01], [-0.3,0.3], [-0.5, 0.5]]
    hzs = ['0.125', '0.25', '0.5']
    wave_s = 1
    velos = [30,60]
  elif fv.n_joint==4:
    abc_range = [[0.001, 0.005], [-0.3,0.0], [-0.5, 0.0]]
    hzs = ['0.5', '1']
    wave_s = 2
    velos = [30,60,90]

  # wave
  dir = 'bag2txt_j'+str(fv.n_joint+1)+'_'
  for hz in hzs:
    for s in range(1,wave_s+1):
      f = dir + hz+'hz_s'+str(s)+'/joint_states.txt'
      if not os.path.isfile(f):
        raise Exception('Invalid file : '+f)
      files.append({'path': f
        , 'get_t_range': get_t_range
        , 'get_data_from_t_range': get_data_from_t_range
        , 'b_filter_dq': False
        , 'b_filter_current': False
        , 'b_torque_acc': True
        })


  # constant velo
  dir = 'bag2txt_j'+str(fv.n_joint+1)+'_v'
  for v in velos:
    fname = dir+str(v)+'/joint_states.txt'
    if not os.path.isfile(fname):
      raise Exception('Invalid file : '+fname)
    files.append({'path': fname
      , 'get_t_range': fv.get_t_range
      , 'get_data_from_t_range': fv.get_data_from_t_range
      , 'b_filter_dq': True
      , 'b_filter_current': True
      , 'b_torque_acc': False
    })

  data_ests = []
  min_est = 9999999
  for f in files:
    print('file : '+f['path'])
    data = fv.load_file(f['path'])
    f['get_t_range'](data)
    print(data['t_range'])
    get_ddq(data)
    fv.get_torque(data, f['path'],b_filter_dq=False, b_filter_ddq=True, b_torque_acc=f['b_torque_acc'])
    f['data'] = data

    if f['b_filter_current']:
      cur = f['get_data_from_t_range'](data, data['current_f'][:,fv.n_joint])
    else:
      cur = f['get_data_from_t_range'](data, data['current'][:,fv.n_joint])
    dq_f = f['get_data_from_t_range'](data, data['dq_f'][:,fv.n_joint])
    tq = f['get_data_from_t_range'](data, data['torque'][:,fv.n_joint])

    '''
    if data_est is None:
      data_est = {'current': cur, 'dq': dq_f, 'torque': tq}
    else:
      data_est['current'] = concatenate( (data_est['current'], cur), axis=0 )
      data_est['dq'] = concatenate( (data_est['dq'], dq_f), axis=0 )
      data_est['torque'] = concatenate( (data_est['torque'], tq), axis=0 )
    '''
    
    data_ests.append({'current': cur, 'dq': dq_f, 'torque': tq})
    if min_est<len(cur):
      min_est = len(cur)


  data_est = None
  for d in data_ests:
    if len(d['current'])>min_est:
      d['current'] = d['current'][:min_est]
      d['dq'] = d['dq'][:min_est]
      d['torque'] = d['torque'][:min_est]
    if data_est is None:
      data_est = d
    else:
      data_est['current'] = concatenate( (data_est['current'], d['current']), axis=0 )
      data_est['dq'] = concatenate( (data_est['dq'], d['dq']), axis=0 )
      data_est['torque'] = concatenate( (data_est['torque'], d['torque']), axis=0 )
  return files, data_est

if __name__ == "__main__":
  dt = 0.020

  files, data_est = load_data(0)
  #check_data_est(data_est)

  # j0
  #abc = fv.find_eq(data_est['current'], data_est['dq'], data_est['torque'], abc_range, 100)
  # j4


  for f in files:
    data = f['data']
    t = f['get_data_from_t_range'](data, data['t'])
    cur = f['get_data_from_t_range'](data, data['current'][:,fv.n_joint])
    cur_f = f['get_data_from_t_range'](data, data['current_f'][:,fv.n_joint])
    dq = f['get_data_from_t_range'](data, data['dq'][:,fv.n_joint])
    dq_f = f['get_data_from_t_range'](data, data['dq_f'][:,fv.n_joint])
    tq = f['get_data_from_t_range'](data, data['torque'][:,fv.n_joint])
    ddq_f = f['get_data_from_t_range'](data, data['ddq_f'][:,fv.n_joint])


    if f['b_filter_current']:
      c = cur_f
    else:
      c = cur
    bias = fv.create_bias(dq_f)
    data['torque_est'] = abc[0]*c + abc[1]*dq_f + abc[2]*bias


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
    #break
  plt.show()
