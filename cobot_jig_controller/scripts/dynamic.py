import numpy as np
import math
import eq
import rospy

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

def torque2current(tq, dq, abc):
#  global abc
  if dq>0:
    c = abc[2]
  else:
    c = -abc[2]
  return abc[0]*tq + abc[1]*dq + c

def jnt_over_limit(jnt_state):
  if jnt_state.position[0] < -1.83 or jnt_state.position[0] > +1.83:
    rospy.loginfo("jnt_state.position[0]")
    return True
  if jnt_state.position[1] < -1.2214 or jnt_state.position[1] > +2.25:
    rospy.loginfo("jnt_state.position[1]")
    return True
  if jnt_state.position[2] < -3.45 or jnt_state.position[2] > +1.25:
    rospy.loginfo("jnt_state.position[2]")
    return True
  if jnt_state.position[3] < -3.15 or jnt_state.position[3] > +3.15:
    rospy.loginfo("jnt_state.position[3]")
    return True
  if jnt_state.position[4] < -3.15 or jnt_state.position[4] > +0.35:
    rospy.loginfo("jnt_state.position[4]")
    return True
  if jnt_state.position[5] < -3.15 or jnt_state.position[5] > +3.15:
    rospy.loginfo("jnt_state.position[5]")
    return True
  return False
