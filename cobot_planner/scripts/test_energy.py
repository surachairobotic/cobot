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
q = np.array([0.5,1.2,0.7,-0.4,1.0, -0.2])
#q = np.array([0.0,math.pi*0.5])
#dq = np.array([1.0,0.0])
#dq = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
dq = np.array([0.5,1.2,0.7,-0.4,1.0, -0.2])

def cal_ddq(q, dq, vars):
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
  return np.linalg.inv(K_ddq).dot(dT_dq - dU_dq - dT_ddq)

def cal_energy(q,dq, vars):
  R,dR_dq,Kw,dKw_dq,J \
  , dJ_dq,dz_dq, M, I, g = vars
  com_z = eq.get_z(q)
  U = 0.0
  T = 0.0
  for i in range(len(q)):
    '''
    print(M[i])
    print(g)
    print(com_z[i])
    '''

    U+= M[i]*g*com_z[i]
    T+= 0.5*( dq.T.dot(
      M[i]*(J[i].T).dot(J[i])
      + Kw[i].T.dot(R[i][0:3,0:3]).dot(I[i]).dot(R[i][0:3,0:3].T).dot(Kw[i])
      ).dot(dq) )
  '''
  print('U : '+str(U))
  print('T : '+str(T))
  exit()
  '''
  '''
  print('U : '+str(U))
  print('T : '+str(T))
  exit()
  '''
  return [U,T]


if __name__ == "__main__":
  t = time.time()
  dt = 0.001
  with open('q.txt','wt') as f:
    for i in range(10000):
      vars = eq.get_vars(q)
  #    print(vars[4])
  #    exit()
      U,T = cal_energy(q,dq,vars)
  #    print(U+T)
      ddq = cal_ddq(q, dq, vars)
      dq+= ddq*dt
      q+= dq*dt

      f.write('%f '% (dt*i))
      for j in range(len(q)):
        f.write('%f %f %f '% (q[j],dq[j],ddq[j]))
      f.write('%f %f\n'% (U, T))
  print(time.time()-t)
