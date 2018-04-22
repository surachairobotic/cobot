import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq
import numpy as np
import eq

#q = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
#q = np.array([0.5,1.2,0.7,-0.4,1.0, -0.2])
q = np.array([0.0,math.pi*0.5])
dq = np.array([0.0,0.0])
#dq = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
#dq = np.array([0.5,1.2,0.7,-0.4,1.0, -0.2])

def cal_ddq(q, dq, vars):
  R,dR_dq,w_com,dw_com,Kw_com,dKw_dq_com,J_com \
  , dJ_dq_com,dz_dq, M, I, g = vars

  #print(dR_dq)
  dR =[]
  dKw_com = []
  dJ_com = []
  for i in range(len(q)):
    dr = None
    dk = None
    dj = None
    for j in range(len(q)):
      if j==0:
        dr = dR_dq[i][j]*dq[j]
        dj = dJ_dq_com[i][j]*dq[j]
        dk = dKw_dq_com[i][j]*dq[j]
      else:
        dr+= dR_dq[i][j]*dq[j]
        dj+= dJ_dq_com[i][j]*dq[j]
        dk+= dKw_dq_com[i][j]*dq[j]
    dR.append(dr)
    dKw_com.append(dk)
    dJ_com.append(dj)

  dT_dq = np.zeros(len(q)) # dT/dq
  dU_dq = np.zeros(len(q)) # dT/dq
  dT_ddq = np.zeros(len(q)) # d(dT/ddq)/dt
  K_ddq = np.zeros([len(q),len(q)])
  for i in range(len(q)):
    for j in range(len(q)):
  #    print(Kw_com[i].T.dot(R[i][0:3,0:3]).dot(I).shape)
      # i : link no. , j : dq
      dT_dq[j]+= 0.5*dq.T.dot(
        M[i]*(dJ_dq_com[i][j].T.dot(J_com[i]) + J_com[i].T.dot(dJ_dq_com[i][j]))
        + dKw_dq_com[i][j].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw_com[i])
        + Kw_com[i].T.dot(dR_dq[i][j][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw_com[i])
        + Kw_com[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(dR_dq[i][j][0:3,0:3].T).dot(Kw_com[i])
        + Kw_com[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(dKw_dq_com[i][j])
        ).dot(dq)
      dU_dq[j]+= (M[i]*g*dz_dq[i][j])[0]

    dT_ddq+= M[i]*( dJ_com[i].T.dot(J_com[i]).dot(dq) + J_com[i].T.dot(dJ_com[i]).dot(dq) ) \
      + dKw_com[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw_com[i]).dot(dq) \
      + Kw_com[i].T.dot(dR[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw_com[i]).dot(dq) \
      + Kw_com[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(dR[i][0:3,0:3].T).dot(Kw_com[i]).dot(dq) \
      + Kw_com[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(dKw_com[i]).dot(dq)
      # + J_com[i].T.dot(J_com[i]).dot(ddq)
      # + Kw_com[i].T.dot(dR[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw_com[i]).dot(ddq)

    K_ddq+= M[i]*(J_com[i].T.dot(J_com[i])) \
      + Kw_com[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw_com[i])
  return np.linalg.inv(K_ddq).dot(dT_dq - dU_dq - dT_ddq)

def cal_energy(q,dq, vars):
  R,dR_dq,w_com,dw_com,Kw_com,dKw_dq_com,J_com \
  , dJ_dq_com,dz_dq, M, I, g = vars
  com_z = eq.get_z(q)
  U = 0.0
  T = 0.0
  for i in range(len(q)):
    U+= M[i]*g*com_z[i]
    T+= 0.5*( dq.T.dot(
      M[i]*(J_com[i].T).dot(J_com[i])
      + Kw_com[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw_com[i])
      ).dot(dq) )
  return U+T

t = time.time()
dt = 0.001
for i in range(100):
  vars = eq.get_vars(q)
  print(cal_energy(q,dq,vars))
  ddq = cal_ddq(q, dq, vars)
  dq+= ddq*dt
  q+= dq*dt

print(time.time()-t)
