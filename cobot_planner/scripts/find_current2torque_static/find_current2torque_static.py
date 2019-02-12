#!/usr/bin/env python

'''
- find equation to approximate torque from current

- find current needed to move motor in each direction (CW,CCW)
- and assume that the mean value is a current that is used to compensate gravity
 - current_grav = (current_CW + current_CCW)*0.5
- find a,b that satisfied
 - torque_grav = a*current_grav + b
- where torque_grav is a torque calculated from a dynamic equation
- equation :
 - torque = a*current + b
'''

import sys
sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')
sys.path.insert(0,'/home/tong/catkin_ws/src/cobot/cobot_planner/scripts')


import sys
import os
import math
import numpy as np
from numpy import linalg
import matplotlib.pyplot as plt

import eq
from test_cal_torque import cal_torque

# data file for creating equation
# (not for test)
#n_joint = 4
#fname = 'torque_j' + str(n_joint) + '/set_1/torque_j'+str(n_joint)+'_all.txt'
#fname = 'find_current2torque_static/running_torque/torque_j5_all.txt'

def load(fname):
  points = []
  if not os.path.isfile(fname):
    print("file not exist : " + fname)
    exit()
#    return None
  with open(fname,'rt') as f:
    for line in f:
      vals = line.strip().split(' ')
      if len(vals)>0:
        points.append([float(x) for x in vals])
  return np.array(points)


class LSM:
  def fit(self,x,y):
    xy = np.sum(x*y)
    nx = np.sum(x)
    ny = np.sum(y)
    xx = np.sum(x*x)
    n = len(x)

    div = n*xx - nx**2
    assert(div!=0)
    a = (n*xy - nx*ny)/div
    b = (xx*ny - xy*nx)/div
    return a,b
  
  def fit_a(self,x,y):
    xy = np.sum(x*y)
    xx = np.sum(x*x)
    a = xy / (2.0*xx)
    return a,0.0

  def get_error(self,x,y,a,b):
    err = y - self.get_y(x,a,b)
    return np.mean(abs(err))

  def get_y(self,x,a,b):
    return a*x+b


class cFindEq:
  def __init__(self):
    self.bias = None
    self.y_est = None
    
  def set_data( self, x, y, ab ):
    self.x = x
    self.y = y
    self.ab = ab
  
  
  def get_bias(self, data):
    if self.bias is not None:
      return self.bias
      
    self.bias = np.zeros(len(data))
    for i in range(len(data)):
      if data[i]>0.0:
        self.bias[i] = 1.0
      elif data[i]<0.0:
        self.bias[i] = -1.0
      else:
        self.bias[i] = 0.0
    return self.bias


  def find_range_ab(self):
    xy = [[],[]]
    for i in range(len(self.x)):
      if self.y[i]>0.0:
        xy[0].append([self.x[i], self.y[i]])
      elif self.y[i]<0.0:
        xy[1].append([-self.x[i], -self.y[i]])
    lsm = LSM()
    ab = [None, None]
    for i in range(2):
      if len(xy[i])>3:
        xy2 = np.array(xy[i])
        ab[i] = np.array( lsm.fit_a( xy2[:,0], xy2[:,1] ) )
    if ab[0] is not None and ab[1] is not None:
      mean = 0.5*(ab[0] + ab[1])
      diff = abs(ab[0] - ab[1])
    else:
      if ab[0] is None:
        ab = ab[1]
      else:
        ab = ab[0]
      mean = ab
      diff = abs( ab*1.5 )
    range_ab = []
    for i in range(2):
      range_ab.append([mean[i]-diff[i], mean[i]+diff[i]])
    print(mean)
    print(diff)
    print(range_ab)
#    exit()
    return range_ab
      
    
  def run( self, x, y, n_loop ):
    self.x = x
    self.y = y
    bias = self.get_bias(x)
    range_ab = self.find_range_ab()
    
    self.ab = []
    dab = []
    min_err = 99999999.0
    for i in range(2):
      dab.append( (range_ab[i][1] - range_ab[i][0])/float(n_loop) )

    for i in range(n_loop):
      a = range_ab[0][0] + dab[0]*i
      for j in range(n_loop):
        b = range_ab[1][0] + dab[1]*j
        y2 = a*x + b*bias
        e = linalg.norm(y2-y)
        if e<min_err:
          min_err = e
          self.ab = [a,b]
          print('['+str((i*(n_loop) + j)/float(n_loop**2))+'] '+str(e/len(x))+' : '+str(self.ab))
    return self.ab

  def get_est_y(self):
    if self.y_est is None:
      self.y_est = self.ab[0] * self.x + self.ab[1] * self.get_bias(self.x)
    return self.y_est
    
  def get_error(self):
    return linalg.norm( self.get_est_y() - self.y )/len(self.x)



def get_data(data):
  t = data[:,0]
  goal_effort_up = data[:,1]
  effort_up = data[:,2]
  goal_effort_down = data[:,3]
  effort_down = data[:,4]
  q = data[:,5:11]
  q_rad = q*math.pi/180.0
  return t, goal_effort_up, effort_up, goal_effort_down, effort_down, q, q_rad


def find_equation(name, n_joint, n_loop):
  data = load(fname)
  if data.shape[0]==0:
    print('No data found')
    return
  t, goal_effort_up, effort_up, goal_effort_down, effort_down, q, q_rad = get_data(data)
  effort_mean = (effort_up + effort_down)*0.5
  effort_fric = abs((effort_up - effort_down)*0.5)

  # cal torque
  dq = np.array([0.0]*6)
  ddq = dq
  torque_cal = []
  for i in range(q_rad.shape[0]):
    q2 = q_rad[i,:]
    vars = eq.get_vars(q2)
    t1, t2 = cal_torque(q2, dq, ddq, vars)
    torque_cal.append(t1)
  torque_cal = np.array(torque_cal)

  # cal current 2 torque
  '''
  lsm = LSM()
  a,b = lsm.fit(effort_mean, torque_cal[:,n_joint])
  err = lsm.get_error(effort_mean, torque_cal[:,n_joint], a, b)
  torque_est = lsm.get_y(effort_mean,a,b)
  torque_fric = a*effort_fric + b
  print('current2torque : a = %f, b = %f, fric = %f' % (a,b,np.mean(torque_fric)))
  print('mean error [N-m]: '+str(err))
  '''
  find_eq = cFindEq()
  a,b = find_eq.run( torque_cal[:,n_joint], effort_mean, n_loop )
  err = find_eq.get_error()
  effort_est = find_eq.get_est_y()
  print('torque2current : a = %f, b = %f, err = %f' % (a,b,err))


  '''
  f, axarr = plt.subplots(2, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')

  axarr[0].plot(q[:,n_joint], effort_up, '*-', q[:,n_joint], effort_down, '*-')
  axarr[0].plot(q[:,n_joint], effort_up - effort_down, '*-')
  axarr[1].plot(q[:,n_joint], torque_cal[:,n_joint], '*-'
    ,q[:,n_joint], torque_est, '*-',q[:,n_joint], torque_fric, '*-')

  axarr[0].legend(['goal_current_up', 'goal_current_down', 'diff_up_down'])
  axarr[0].set_ylabel('torque [N-m]')
  axarr[1].legend(['torque', 'torque_est'])
  axarr[1].set_ylabel('torque [N-m]')
  axarr[len(axarr)-1].set_xlabel('q'+str(n_joint+1)+' [degree]')
  #plt.show()
  '''
  f, axarr = plt.subplots(1, sharex=True)
  axarr.hold(True)
  axarr.grid(linestyle='-', linewidth='0.5')
  axarr.plot( q[:,n_joint], effort_up, '*-', q[:,n_joint], effort_down, '*-'
    , q[:,n_joint], effort_mean, '*-', q[:,n_joint], effort_est, '*-')
  axarr.legend(['effort_up', 'effort_down', 'effort_mean', 'effort_est'])
  axarr.set_ylabel('Effort')
  axarr.set_xlabel('q'+str(n_joint+1)+' [degree]')
  return [a,b]


def test_equation(n_joint, ab):
  dir = 'torque_j'+str(n_joint+1)+'/set_'
  n = 3

  if n==1:
    f, axarr = plt.subplots(2, sharex=True)
  else:
    f, axarr = plt.subplots(n, sharex=True)
  for k in range(n):
    fname = dir+str(k+1)+'/torque_j'+str(n_joint+1)+'_all.txt'
    data = load(fname)
    t, goal_effort_up, effort_up, goal_effort_down, effort_down, q, q_rad = get_data(data)
    effort_mean = (effort_up + effort_down)*0.5
    effort_fric = abs((effort_up - effort_down)*0.5)

    # cal torque
    dq = np.array([0.0]*6)
    ddq = dq
    torque_cal = []
    for i in range(q_rad.shape[0]):
      q2 = q_rad[i,:]
      vars = eq.get_vars(q2)
      t1, t2 = cal_torque(q2, dq, ddq, vars)
      torque_cal.append(t1)
    torque_cal = np.array(torque_cal)
    
    find_eq = cFindEq()
    find_eq.set_data( torque_cal[:,n_joint], effort_mean, ab )
    effort_est = find_eq.get_est_y()
    err = find_eq.get_error()
    
    print('error : %f' % (err))
    
    axarr[k].hold(True)
    axarr[k].grid(linestyle='-', linewidth='0.5')
    axarr[k].set_ylabel('Effort')

    axarr[k].plot( q[:,n_joint], effort_up, '*-', q[:,n_joint], effort_down, '*-'
    , q[:,n_joint], effort_mean, '*-', q[:,n_joint], effort_est, '*-')

      
    # cal torque 2 current
    '''
    # cal current 2 torque
    lsm = LSM()
    a = ab[0]
    b = ab[1]
    #a,b = lsm.fit(effort_mean, torque_cal[:,n_joint])
    err = lsm.get_error(effort_mean, torque_cal[:,n_joint], a, b)
    torque_est = lsm.get_y(effort_mean,a,b)
    torque_fric = a*effort_fric + b
    print('current2torque : a = %f, b = %f, fric = %f' % (a,b,np.mean(torque_fric)))
    print('mean error [N-m]: '+str(err))


    axarr[k].hold(True)
    axarr[k].grid(linestyle='-', linewidth='0.5')
    axarr[k].set_ylabel('torque [N-m]')

    axarr[k].plot(q[:,n_joint], torque_cal[:,n_joint], '*-'
    ,q[:,n_joint], torque_est, '*-',q[:,n_joint], torque_fric, '*-')

  axarr[0].legend(['torque', 'torque_est'])
  axarr[len(axarr)-1].set_xlabel('q'+str(n_joint+1)+' [degree]')
  '''
  axarr[0].legend(['effort_up', 'effort_down', 'effort_mean', 'effort_est'])
  axarr[len(axarr)-1].set_xlabel('q'+str(n_joint+1)+' [degree]')
  

if __name__ == "__main__":
  n_joint = 3
  fname = 'torque_j' + str(n_joint+1) + '/set_1/torque_j'+str(n_joint+1)+'_all.txt'
  ab = find_equation(fname, n_joint, 100) # find a,b
  test_equation(n_joint, ab) # test a,b with other dataset
  plt.show()

