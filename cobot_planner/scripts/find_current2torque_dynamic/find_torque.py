# -*- coding: utf-8 -*-

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

data_num = 10000
n_joint = 4



# ax + by + c = z
class LSM3:
  def fit(self,x,y,z):
    xx = sum(x*x)
    xy = sum(x*y)
    xz = sum(x*z)
    yy = sum(y*y)
    yz = sum(y*z)
    nx = sum(x)
    ny = sum(y)
    nz = sum(z)
    n = len(x)

    A = array([[xx, xy, nx], [xy, yy, ny], [nx, ny, n]])
    B = array([xz, yz, nz])
    abc = linalg.inv(A).dot(B)
    return abc

  def get_error(self,x,y,z,abc):
    err = z - self.get_z(x,y,abc)
    return mean(abs(err))

  def get_z(self,x,y,abc):
    return abc[0]*x+abc[1]*y+abc[2]

  def test(self):
    import random

    x = []
    y = []
    abc = []
    for i in range(100):
      x.append(random.uniform(-10.0,50.0))
      y.append(random.uniform(-8.0,5.0))
    x = array(x)
    y = array(y)
    for i in range(3):
      abc.append(random.uniform(-8.0,8.0))
    z = abc[0]*x + abc[1]*y + abc[2]
    for i in range(len(z)):
      z[i]+= random.uniform(-0.1,0.1)

    abc2 = self.fit(x,y,z)
    print('err abc : '+str(abc-abc2))


def load_file(fname):
  with open(fname, 'rt') as f:
    n_line = 0
    q = []
    t = []
    dq = []
    current = []

    for line in f:
      n_line+=1
      arr = line.strip().split(' ')
      if len(arr)==19:
        t.append(float(arr[0]))
        q.append( array([float(a) for a in arr[1:7]]) )
        dq.append( array([float(a) for a in arr[7:13]]) )
        current.append( array([float(a) for a in arr[13:19]]) )
    t = array(t)
    q = array(q)
    dq = array(dq)
    current = array(current)
    return {'t': t, 'q': q, 'dq': dq, 'current': current}


def get_torque(data):
  ddq = zeros(6)
  torque = []
  torque_no_ddq = []
  q = data['q']
  dq = data['dq']
  for i in range(len(q)):
    vars = eq.get_vars(q[i,:])
    t_ddq, t_no_ddq = cal_torque(q[i,:], dq[i,:], ddq, vars)
    torque.append(t_ddq)
    torque_no_ddq.append(t_no_ddq)

  data['torque'] = array(torque)
  data['torque_no_ddq'] = array(torque_no_ddq)



def get_t_range(data):
  global n_joint
  dq = data['dq']
  t = data['t']
  n = len(t)

  n_mean = 10
  n_clip = 15
  b_found = False

  t_range = []
  i1 = -1
  for i in range(n_mean, n):
    v = dq[i-n_mean:i,n_joint]
    v1 = min(v)
    v2 = max(v)
    v_cen = (v1+v2)*0.5
    if abs(v_cen)<0.2:
      b_fix = False
    else:
      b_fix = (abs((v2-v1)/v_cen) < 0.4)

    if b_found:
      if not b_fix:
        i1+= n_clip
        i2 = i-n_clip
        if t[i2] - t[i1] > 1.0:
          t_range.append({'t1': t[i1], 'i1': i1, 't2': t[i2], 'i2': i2})
        b_found = False
    else:
      if b_fix:
        b_found = True
        i1 = i
  data['t_range'] = t_range


def get_data_from_t_range(data, data_range):
  data2 = None
  for tr in data['t_range']:
    if len(data_range.shape)==1:
      new_d = data_range[tr['i1']:tr['i2']]
    else:
      new_d = data_range[tr['i1']:tr['i2'],:]
    if data2 is None:
      data2 = new_d
    else:
      data2 = concatenate( (data2, new_d), axis=0 )
  return data2


'''
def filter(data):
  n = 5
  data2 = copy.deepcopy(data)
  for i in range(n, len(data)):
    data2[i] = mean(data[i-n:i+n+1])
  return data2
'''


def filter(data, dt):
  fn = 1.0/(2*dt)                   # ナイキスト周波数
  fp = 1.0                          # 通過域端周波数[Hz]
  fs = 2.0                          # 阻止域端周波数[Hz]
  gpass = 1                       # 通過域最大損失量[dB]
  gstop = 50                      # 阻止域最小減衰量[dB]
  # 正規化
  Wp = fp/fn
  Ws = fs/fn
  '''
  N, Wn = signal.buttord(Wp, Ws, gpass, gstop)
  print(N)
  print(Wn)
  b1, a1 = signal.butter(N, Wn, "low")
  return signal.filtfilt(b1, a1, data)
  '''

  '''
  N, Wn = signal.cheb1ord(Wp, Ws, gpass, gstop)
  b2, a2 = signal.cheby1(N, gpass, Wn, "low")
  return signal.filtfilt(b2, a2, data)
  '''

  N, Wn = signal.cheb2ord(Wp, Ws, gpass, gstop)
  b3, a3 = signal.cheby2(N, gstop, Wn, "low")
  return signal.filtfilt(b3, a3, data)

def test_filter(data):
  global n_joint

  t2 = get_data_from_t_range(data, data['t'])
  cur = get_data_from_t_range(data, data['current'])[:,n_joint]
  dq = get_data_from_t_range(data, data['dq'])[:,n_joint]
  cur_f = filter(data['current'][:,n_joint], 0.020)
  cur_f = get_data_from_t_range(data, cur_f)
  dq_f = filter(data['dq'][:,n_joint], 0.020)
  dq_f = get_data_from_t_range(data, dq_f)

  f, axarr = plt.subplots(2, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')

  axarr[0].plot(t2, cur, t2, cur_f, '*-')
  axarr[1].plot(t2, dq, t2, dq_f, '*-')

  axarr[0].set_ylabel('current [mA]')
  axarr[1].set_ylabel('dq [rad/s]')
  axarr[0].legend(['raw', 'filter'])
  axarr[len(axarr)-1].set_xlabel('time [s]')
  plt.show()
  exit()




if __name__ == "__main__":
  lsm = LSM3()

  dir = 'bag2txt_j5_v'
  dt = 0.020
  for v in [30,60,90]:
    fname = dir+str(v)+'/joint_states.txt'
    data = load_file(fname)
    get_t_range(data)

    #test_filter(data)

    t2 = get_data_from_t_range(data, data['t'])
    cur = get_data_from_t_range(data, data['current'][:,n_joint])

    dq = get_data_from_t_range(data, data['dq'][:,n_joint])
    cur_f = filter(data['current'][:,n_joint], dt)
    cur_f = get_data_from_t_range(data, cur_f)
    dq_f = filter(data['dq'][:,n_joint], dt)
    dq_f = get_data_from_t_range(data, dq_f)

    get_torque(data)
    tq = get_data_from_t_range(data, data['torque_no_ddq'][:,n_joint])

    abc = lsm.fit( cur, dq_f, tq )
    print(abc)
    print(lsm.get_error(cur_f, dq_f, tq, abc))
    torque2 = lsm.get_z( cur_f, dq_f, abc )


    # for plotting
    t_range = data['t_range']
    t_range_plot = zeros(len(data['t']))
    for tr in t_range:
      t_range_plot[ tr['i1']:tr['i2'] ] = 1.0


    # plot
    f, axarr = plt.subplots(4, sharex=True)
    for i in range(len(axarr)):
      axarr[i].hold(True)
      axarr[i].grid(linestyle='-', linewidth='0.5')

    axarr[0].plot(data['t'],data['q'][:,n_joint])
    axarr[1].plot(data['t'],data['dq'][:,n_joint], t2, dq_f)
    axarr[2].plot(data['t'],data['current'][:,n_joint], t2, cur_f)
    axarr[3].plot(t2, tq , t2, torque2)
    axarr[1].plot(data['t'],t_range_plot, '*-')

    axarr[0].set_ylabel('q [rad]')
    axarr[1].set_ylabel('dq [rad/s]')
    axarr[2].set_ylabel('current [mA]')
    axarr[3].set_ylabel('torque [N-m]')
    axarr[0].legend(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    axarr[3].legend(['measure', 'estimate'])
    axarr[len(axarr)-1].set_xlabel('time [s]')
    break
  plt.show()
