# -*- coding: utf-8 -*-

'''
find equation to estimate torque from current and dq
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

n_joint = 4
dt = 0.020

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
  global dt
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
    data = {'t': t, 'q': q, 'dq': dq, 'current': current}

    data['current_f'] = filter(data['current'], dt)
    data['dq_f'] = filter(data['dq'], dt)

    return data



def get_torque(data, fname, b_filter_dq, b_filter_ddq, b_torque_acc):
  path = os.path.dirname(os.path.abspath(fname))
  
  fname_torque = path + '/torque'
  if b_filter_dq:
    fname_torque += '_f_dq'
  if b_filter_ddq:
    fname_torque += '_f_ddq'
  fname_torque+= '.txt'
  
  torque = []
  torque_no_ddq = []
  if os.path.isfile(fname_torque):
    with open(fname_torque, 'rt') as f:
      for line in f:
        arr = line.strip().split(' ')
        if len(arr)==12:
          torque.append([float(a) for a in arr[0:6]])
          torque_no_ddq.append([float(a) for a in arr[6:12]])

      if len(torque)!=len(data['t']):
        raise Exception('Torque length does not match : %d / %d' % (len(torque), len(data['t'])))

      data['torque'] = array(torque)
      data['torque_no_ddq'] = array(torque_no_ddq)
  else:
    ddq0 = zeros(6)

    q = data['q']
    t = data['t']
    if b_filter_dq:
      dq = data['dq_f']
    else:
      dq = data['dq']
    for i in range(len(q)):
      vars = eq.get_vars(q[i,:])
      if b_torque_acc and i>0:
        if b_filter_ddq:
          ddq = data['ddq_f'][i,:]
        else:
          ddq = data['ddq'][i,:]
        #ddq = (dq[i,:] - dq[i-1,:])/(t[i]-t[i-1])
      else:
        ddq = ddq0
      t_ddq, t_no_ddq = cal_torque(q[i,:], dq[i,:], ddq, vars)
      torque.append(t_ddq)
      torque_no_ddq.append(t_no_ddq)

    data['torque'] = array(torque)
    data['torque_no_ddq'] = array(torque_no_ddq)

    with open(fname_torque, 'wt') as f:
      for i in range(len(torque)):
        for j in range(len(torque[i])):
          f.write('%f ' % (torque[i][j]))
        for j in range(len(torque_no_ddq[i])):
          f.write('%f ' % (torque_no_ddq[i][j]))
        f.write('\n')



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
  if len(data.shape)==1 or data.shape[1]==1:
    return signal.filtfilt(b3, a3, data)
  else:
    d2 = copy.deepcopy(data)
    for i in range(data.shape[1]):
      d2[:,i] = signal.filtfilt(b3, a3, data[:,i])
    return d2



def test_filter(data):
  global n_joint

  t2 = get_data_from_t_range(data, data['t'])
  cur = get_data_from_t_range(data, data['current'])[:,n_joint]
  dq = get_data_from_t_range(data, data['dq'])[:,n_joint]
  cur_f = filter(data['current'], 0.020)[:,n_joint]
  cur_f = get_data_from_t_range(data, cur_f)
  dq_f = filter(data['dq'], 0.020)[:,n_joint]
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



# z = abc[0]*x + abc[1]*y + abc[2]
def find_eq( x, y, z, range_abc, n_loop ):
  abc = []
  dabc = []
  min_err = 99999999.0
  for i in range(3):
    dabc.append( (range_abc[i][1] - range_abc[i][0])/float(n_loop) )

  bias = create_bias(y)
  for i in range(n_loop):
    a = range_abc[0][0] + dabc[0]*i
    for j in range(n_loop):
      b = range_abc[1][0] + dabc[1]*j
      for k in range(n_loop):
        c = range_abc[2][0] + dabc[2]*k

        z2 = a*x + b*y + c*bias
        e = linalg.norm(z2-z)
        if e<min_err:
          min_err = e
          abc = [a,b,c]
          print('['+str((i*(n_loop**2) + j*n_loop + k)/float(n_loop**3))+'] '+str(e/len(x))+' : '+str(abc))
  return abc


def create_bias(data):
  bias = zeros(len(data))
  for i in range(len(data)):
    if data[i]>0.0:
      bias[i] = 1.0
    elif data[i]<0.0:
      bias[i] = -1.0
    else:
      bias[i] = 0.0
  return bias

if __name__ == "__main__":
  lsm = LSM3()
  #abc = [0.0043, -0.03200000000000003, -0.32000000000000006]

  dir = 'bag2txt_j5_v'
  velos = [30,60,90]

  datas = []
  data_est = None
  for v in velos:
    # load data
    fname = dir+str(v)+'/joint_states.txt'
    data = load_file(fname)
    get_torque(data, fname, b_filter_dq=True, b_filter_ddq=True, b_torque_acc=False)
    get_t_range(data)

    t2 = get_data_from_t_range(data, data['t'])
    cur = get_data_from_t_range(data, data['current'][:,n_joint])
    dq = get_data_from_t_range(data, data['dq'][:,n_joint])
    cur_f = get_data_from_t_range(data, data['current_f'][:,n_joint])
    dq_f = get_data_from_t_range(data, data['dq_f'][:,n_joint])
    tq = get_data_from_t_range(data, data['torque_no_ddq'][:,n_joint])

    if data_est is None:
      data_est = {'current': cur_f, 'dq': dq_f, 'torque': tq}
    else:
      data_est['current'] = concatenate( (data_est['current'], cur_f), axis=0 )
      data_est['dq'] = concatenate( (data_est['dq'], dq_f), axis=0 )
      data_est['torque'] = concatenate( (data_est['torque'], tq), axis=0 )
    datas.append(data)


  abc = find_eq(data_est['current'], data_est['dq'], data_est['torque'], [[0.002, 0.007], [-0.8,0.8], [-0.8, 0.8]], 100)


  for iv in range(len(velos)):
    data = datas[iv]

    t2 = get_data_from_t_range(data, data['t'])
    cur = get_data_from_t_range(data, data['current'][:,n_joint])
    dq = get_data_from_t_range(data, data['dq'][:,n_joint])
    cur_f = get_data_from_t_range(data, data['current_f'][:,n_joint])
    dq_f = get_data_from_t_range(data, data['dq_f'][:,n_joint])
    tq = get_data_from_t_range(data, data['torque_no_ddq'][:,n_joint])

    data['torque_est'] = abc[0]*cur_f + abc[1]*dq_f + abc[2]*create_bias(dq_f)

    # fit
    '''
    abc = lsm.fit( cur, dq_f, tq )
    print(abc)
    print(lsm.get_error(cur_f, dq_f, tq, abc))
    torque2 = lsm.get_z( cur_f, dq_f, abc )
    '''

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
    axarr[1].plot(data['t'],t_range_plot)
    axarr[1].plot(data['t'],data['dq'][:,n_joint], t2, dq_f)
    axarr[2].plot(data['t'],data['current'][:,n_joint], t2, cur_f)
    axarr[3].plot(t2, data['torque_est'] , t2, tq)

    axarr[0].set_ylabel('q [rad]')
    axarr[1].set_ylabel('dq [rad/s]')
    axarr[2].set_ylabel('current [mA]')
    axarr[3].set_ylabel('torque [N-m]')
    axarr[0].legend(['q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    axarr[3].legend(['estimate', 'calculation'])
    axarr[len(axarr)-1].set_xlabel('time [s]')
    #break
  plt.show()
