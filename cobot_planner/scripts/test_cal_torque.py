import sympy as sp
import numpy as np
import math
import time
import pickle
import lib_sympy as lib_sp
import lib_equation as lib_eq
import numpy as np
import eq


fname = '/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_wave_j3/joint_states.txt'

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
#  return np.linalg.inv(K_ddq).dot(dT_dq - dU_dq - dT_ddq)
  torque_no_ddq = - (dT_dq - dU_dq - dT_ddq)
  return K_ddq.dot(ddq) + torque_no_ddq, torque_no_ddq
#  return -(dT_dq - dU_dq - dT_ddq)



def save_torque(fname):
  data = []
  with open(fname,'rt') as f:
    for line in f:
      arr = line.strip().split(' ')
      arr = [float(x) for x in arr]
      data.append(arr)

  fname_out = fname[0:fname.rfind('/')+1] + 'torque.txt'
  with open(fname_out, 'wt') as f:
    for i in range(1, len(data)):
      d = data[i]
      q = np.array(d[1:7])
      dq = np.array(d[7:13])
      ddq = np.array([0.0]*6)
      current = np.array(d[13:19])
      d2 = data[i-1]
      for j in range(6):
        ddq[j] = (d[7+j] - d2[7+j])/(d[0]-d2[0])
      vars = eq.get_vars(q)
      torque, torque_no_ddq = cal_torque(q, dq, ddq, vars)
      f.write('%f' % (d[0]))
      for j in range(6):
        f.write(' %f' % (torque[j]))
      for j in range(6):
        f.write(' %f' % (current[j]))
      for j in range(6):
        f.write(' %f' % (ddq[j]))
      for j in range(6):
        f.write(' %f' % (torque_no_ddq[j]))
      f.write('\n')


def run_multiple():
  pre = '/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_'
  for i in range(1,4):
    for j in [5,7,10]:
      for k in range(1,4):
        fname = pre + 'j%d_v%02d_s%d/joint_states.txt' % (i,j,k)
        print(fname)
        save_torque(fname)


if __name__ == "__main__":
  save_torque(fname)
#  run_multiple()
'''
t = time.time()
dt = 0.001
with open('q.txt','wt') as f:
  for i in range(10000):
    vars = eq.get_vars(q)
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
'''
