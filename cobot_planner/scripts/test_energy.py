import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq
import numpy as np
import eq

q = np.array([0.5,1.2,0.7,-0.4,1.0, -0.2])
dq = np.array([0.5,1.2,0.7,-0.4,1.0, -0.2])

g = 9.8
M = [0,3,2,2.2,1.5,1.2,0.8,0.7]
I = [np.diag([2,2,1])
, np.diag([1,1,2])
, np.diag([1,2,1])
, np.diag([1,2,2])
, np.diag([2,1,1])
, np.diag([1,2,1])
, np.diag([2,2,1])
]

t = time.time()
R,dR_dq,w_com,dw_com,Kw_com,dKw_dq_com,J_com \
, dJ_dq_com,dz_dq = eq.get_vars(q)

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

dT_dq = []
dU_dq = []
ddT = []
for i in range(len(q)):
  dT_dq.append(0)
  for j in range(len(q)):
    dT_dq[i] = 0.5*dq.T.dot(
      M[i]*(dJ_dq_com[i][j].T.dot(J_com[i]) + J_com[i].T.dot(dJ_dq_com[i][j]))
      + dKw_dq_com[i][j].T.dot(R[i]).dot(I).dot(R[i].T).dot(Kw_com[i])
      + Kw_com[i][j].T.dot(dR_dq[i][j]).dot(I).dot(R[i].T).dot(Kw_com[i])
      + Kw_com[i][j].T.dot(R[i]).dot(I).dot(dR_dq[i][j].T).dot(Kw_com[i])
      + Kw_com[i][j].T.dot(R[i]).dot(I).dot(R[i].T).dot(dKw_dq_com[i])
      ).dot(dq)
  dU_dq.append(M[i]*g*dz_dq[i])

print(time.time()-t)
