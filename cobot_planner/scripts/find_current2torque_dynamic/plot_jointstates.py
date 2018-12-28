# -*- coding: utf-8 -*-

'''
plot joint state
'''

import sys
sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')
sys.path.insert(0,'/home/tong/catkin_ws/src/cobot/cobot_planner/scripts')


import sympy as sp
from numpy import *
import math
import time
import matplotlib.pyplot as plt
import os
import find_torque_velo as fv
import find_torque_acc as fa


fv.n_joint = 2

def get_t_from_t_range(data):
  val = 0.5
  t = data['t']
  if type(data['t_range'][0]) is int:
    t1 = t[data['t_range'][0]]
    t2 = t[data['t_range'][1]]
    data['t_range_x'] = [ t[0], t1-0.0001,t1,t2,t2+0.0001, t[-1] ]
    data['t_range_y'] = [ 0, 0, val, val,0,0 ]
  else:
    x = [t[0]]
    y = [0.0]
    for tr in data['t_range']:
      x.append( tr['t1']-0.0001 )
      y.append(0.0)
      x.append( tr['t1'] )
      y.append(val)
      x.append( tr['t2'] )
      y.append(val)
      x.append( tr['t2']+0.0001 )
      y.append(0.0)
    x.append( t[-1] )
    y.append(0.0)
    data['t_range_x'] = x
    data['t_range_y'] = y


if __name__ == "__main__":
  dt = 0.020
  files = []
  
  if fv.n_joint==0:
    hzs = ['0.125', '0.25', '0.5']
    wave_s = 1
  elif fv.n_joint==2:
    hzs = ['0.125', '0.25']
    wave_s = 1
  elif fv.n_joint==3:
    hzs = ['0.125', '0.25']
    wave_s = 1
  elif fv.n_joint==4:
    hzs = ['0.5', '1']
    wave_s = 1
  elif fv.n_joint==5:
    hzs = ['0.125', '0.25', '0.5']
    wave_s = 1
  else:
    assert(0)

  # wave
  dir = 'bag2txt_j'+str(fv.n_joint+1)+'_'
  for hz in hzs:
    for s in range(1,wave_s+1):
      f = dir + hz+'hz_s'+str(s)+'/joint_states.txt'
      if not os.path.isfile(f):
        raise Exception('Invalid file : '+f)
      files.append({'path': f
        , 'get_t_range': fa.get_t_range
        , 'get_data_from_t_range': fa.get_data_from_t_range
        , 'b_filter_current': False
        , 'b_torque_acc': True
        })


  # constant velo
  dir = 'bag2txt_j'+str(fv.n_joint+1)+'_v'
  velos = [30,60,90]
  for v in velos:
    fname = dir+str(v)+'/joint_states.txt'
    if not os.path.isfile(fname):
      raise Exception('Invalid file : '+fname)
    files.append({'path': fname
      , 'get_t_range': fv.get_t_range
      , 'get_data_from_t_range': fv.get_data_from_t_range
      , 'b_filter_current': True
      , 'b_torque_acc': False
    })

  for f in files:
    data = fv.load_file(f['path'])
    fv.get_torque(data, f['path'],b_filter_dq=False,b_filter_ddq=True, b_torque_acc=f['b_torque_acc'])
    f['get_t_range'](data)
    get_t_from_t_range(data)
    
    # plot
    f, axarr = plt.subplots(4, sharex=True)
    for i in range(len(axarr)):
      axarr[i].hold(True)
      axarr[i].grid(linestyle='-', linewidth='0.5')

    for i in range(6):
      axarr[0].plot(data['t'],data['q'][:,i], data['t_range_x'], data['t_range_y'])
      axarr[1].plot(data['t'],data['dq'][:,i])
      axarr[2].plot(data['t'],data['current'][:,i])
      axarr[3].plot(data['t'], data['torque'][:,i])

    axarr[0].set_ylabel('q [rad]')
    axarr[1].set_ylabel('dq [rad/s]')
    axarr[2].set_ylabel('current [mA]')
    axarr[3].set_ylabel('Torque [N-m]')
    axarr[0].legend(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    axarr[len(axarr)-1].set_xlabel('time [s]')
  plt.show()
    
    
    
