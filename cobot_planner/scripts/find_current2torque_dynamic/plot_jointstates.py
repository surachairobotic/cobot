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
import matplotlib.pyplot as plt
import os
import find_torque_velo as fv


fv.n_joint = 4

if __name__ == "__main__":
  dt = 0.020
  files = []
  
  if fv.n_joint==0:
    hzs = ['0.125', '0.25', '0.5']
    wave_s = 1
  elif fv.n_joint==4:
    hzs = ['0.5', '1']
    wave_s = 2

  # wave
  dir = 'bag2txt_j'+str(fv.n_joint+1)+'_'
  for hz in hzs:
    for s in range(1,wave_s+1):
      f = dir + hz+'hz_s'+str(s)+'/joint_states.txt'
      if not os.path.isfile(f):
        raise Exception('Invalid file : '+f)
      files.append({'path': f
#        , 'get_t_range': get_t_range
#        , 'get_data_from_t_range': get_data_from_t_range
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
      , 'b_filter_current': True
      , 'b_torque_acc': False
    })

  for f in files:
    data = fv.load_file(f['path'])
    fv.get_torque(data, f['path'],b_filter_dq=False,b_filter_ddq=True, b_torque_acc=f['b_torque_acc'])
    
    # plot
    f, axarr = plt.subplots(4, sharex=True)
    for i in range(len(axarr)):
      axarr[i].hold(True)
      axarr[i].grid(linestyle='-', linewidth='0.5')

    for i in range(6):
      axarr[0].plot(data['t'],data['q'][:,i])
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
    
    
    
