#!/usr/bin/env python
# coding: utf-8

import numpy as np
import os
import eq


abc = [0.00272, -0.06, -0.15999999999999998]

def cal_torque(q, dq, ddq, vars):
  
  R,dR_dq,Kw,dKw_dq,J \
  , dJ_dq,dz_dq, M, I, g = vars

  #print(dR_dq)
  dR =[]
  dKw = []
  dJ = []
  for i in range(len(q)):
    dr = None
    dk = None
    dj = None
    for j in range(len(q)):
      if j==0:
        dr = dR_dq[i][j]*dq[j]
        dj = dJ_dq[i][j]*dq[j]
        dk = dKw_dq[i][j]*dq[j]
      else:
        dr+= dR_dq[i][j]*dq[j]
        dj+= dJ_dq[i][j]*dq[j]
        dk+= dKw_dq[i][j]*dq[j]
    dR.append(dr)
    dKw.append(dk)
    dJ.append(dj)

  dT_dq = np.zeros(len(q)) # dT/dq
  dU_dq = np.zeros(len(q)) # dU/dq
  dT_ddq = np.zeros(len(q)) # d(dT/ddq)/dt
  K_ddq = np.zeros([len(q),len(q)]) # dT/dq - dU/dq - d(dT/ddq)/dt = K_ddq*ddq
  for i in range(len(q)):
    for j in range(len(q)):
      # i : link no. , j : dq
      dT_dq[j]+= 0.5*dq.T.dot(
        M[i]*(dJ_dq[i][j].T.dot(J[i]) + J[i].T.dot(dJ_dq[i][j]))
        + dKw_dq[i][j].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i])
        + Kw[i].T.dot(dR_dq[i][j][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i])
        + Kw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(dR_dq[i][j][0:3,0:3].T).dot(Kw[i])
        + Kw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(dKw_dq[i][j])
        ).dot(dq)
      dU_dq[j]+= (M[i]*g*dz_dq[i][j])[0]

    dT_ddq+= M[i]*( dJ[i].T.dot(J[i]).dot(dq) + J[i].T.dot(dJ[i]).dot(dq) ) \
      + dKw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i]).dot(dq) \
      + Kw[i].T.dot(dR[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i]).dot(dq) \
      + Kw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(dR[i][0:3,0:3].T).dot(Kw[i]).dot(dq) \
      + Kw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(dKw[i]).dot(dq)
      # + J[i].T.dot(J[i]).dot(ddq)
      # + Kw[i].T.dot(dR[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i]).dot(ddq)

    K_ddq+= M[i]*(J[i].T.dot(J[i])) \
      + Kw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i])
  
    '''
    print('M : '+str(M[i]*(J[i].T.dot(J[i]))))
    print('Kw : '+str(Kw[i]))
    print('I : '+str(R[i][0:3,0:3].dot(I[i]).dot(R[i][0:3,0:3].T)))
    print('KIK : '+str(Kw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i])))
    '''
  torque_no_ddq = - (dT_dq - dU_dq - dT_ddq)
  return K_ddq.dot(ddq) + torque_no_ddq, torque_no_ddq


def current2torque(current, dq):
  global abc
  if dq>0:
    c = abc[2]
  else:
    c = -abc[2]
  return abc[0]*current + abc[1]*dq + c
  
  
  
if __name__ == "__main__":
  import matplotlib.pyplot as plt
  
  # load data from file
  fname = 'bag2txt_wave_j3/joint_states.txt'
  if not os.path.isfile(fname):
    print("file not exist : " + fname)
    
  data = []
  with open(fname,'rt') as f:
    for line in f:
      vals = line.strip().split(' ')
      if len(vals)==19:
        data.append([float(x) for x in vals])
  data = np.array(data)
  t = data[:,0]
  q = data[:,1:7]
  dq = data[:,7:13]
  current = data[:,13:19]
  
  # calculate acceleration
  ddq = np.zeros(q.shape)
  dt = np.zeros(len(t)-1)
  dt = t[1:] - t[:-1]
  dt = np.tile(dt,(6,1)).T
  ddq[1:,:] = (dq[1:,:] - dq[:-1,:])/dt
  
  # calculate torque
  torque = []
  for i in range(len(t)):
    vars = eq.get_vars(q[i,:]) # load equation parameters for calculating torque
    torque_with_ddq, torque_no_ddq = cal_torque(q[i,:], dq[i,:], ddq[i,:], vars)
    torque.append(torque_with_ddq)
  torque = np.array(torque)
  
  # plot
  f, axarr = plt.subplots(5, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')
  for i in range(6):
    axarr[0].plot(t,q[:,i])
    axarr[1].plot(t,dq[:,i])
    axarr[2].plot(t,ddq[:,i])
    axarr[3].plot(t,current[:,i])
    axarr[4].plot(t,torque[:,i])
  
  axarr[0].set_ylabel('q [rad]')
  axarr[1].set_ylabel('dq [rad/s]')
  axarr[2].set_ylabel('ddq [rad/s^2]')
  axarr[3].set_ylabel('current [mA]')
  axarr[4].set_ylabel('torque [N-m]')
  axarr[len(axarr)-1].set_xlabel('time [s]')


  axarr[0].legend(['q1','q2','q3','q4','q5','q6'])
  plt.show()
  

