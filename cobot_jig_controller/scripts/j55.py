#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState

import numpy as np
import eq
from scipy import signal

# J1 = [-1.5   , +1.5 ] 0
# J2 = [-1.2214, +2.25] 1
# J3 = [-3.45  , +1.25] 2
# J4 = [-3.15  , +3.15] 3
# J5 = [-3.15  , +0.35] 4
# J6 = [-3.15  , +3.15] 5
MIN = [-1.5, -1.2214, -3.45, -3.15, -3.15, -3.15] #-math.pi/2
MAX = [ 1.5,  2.25,    1.25,  3.15,  0.0,  3.15] #math.pi/2
numJnt = 4
nameJnt = 'J5'

file_path = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/pid/"
file_name = ""
arm_state = JointState()
jntReturn = JointState()
b_return_start = b_lock = False
t_fp = time.time()
t_offset = time.time()
effort = 0.0
t_fb = time.time()
en_fb = False

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

def set_jnt(jnt, pub):
#  print('set_jnt')
  global b_return_start, effort, en_fb, t_fb
  rate = rospy.Rate(100) # 100hz

  chk = False
  num = 0
  while not chk and not rospy.is_shutdown():
    jnt.header.frame_id = str(num)
    num += 1
    pub.publish(jnt)
    rate.sleep()
    b_lock = True
    chk = chk_return(jnt)
    b_lock = False
    if en_fb is True:
      rospy.loginfo("frame_id send : %s", jnt.header.frame_id)

  if en_fb is True:
    rospy.loginfo("t_fb : %s", time.time()-t_fb)
    rospy.loginfo("seq : %s", jntReturn.header.seq)
    en_fb = False
  return True;

#  if len(jnt.effort) != 0:
##    save_file(jnt.effort[0], file_name)
#    effort = jnt.effort[0]
#    return True;
#  else:
#    effort = 0.0

#  if len(jnt.position) == 0:
#    return True;

#  b_stop = False
#  while not b_stop and not rospy.is_shutdown():
#      b_stop = True
#      rate.sleep()
#      save_file(0.0, file_name)
#      for v in arm_state.velocity:
#          if v > 0.005:
#              b_stop = False
#              break
  
#  b_ok = True
#  for i in range(0, len(jnt.name)-1):
#    for j in range(0, len(arm_state.name)-1):
#      if jnt.name[i] == arm_state.name[j]:
#        if abs(arm_state.position[j]-jnt.position[i]) > 0.05:
#          print("arm[", j, "]=", arm_state.position[j], " - jnt[", i, "]=", jnt.position[i])
#          b_ok = False
#          break
  
#  save_file(999)
#  print('end set_jnt')
#  return b_ok;
###################################

def callback(data):
  global arm_state
  arm_state = data
#  print(math.degrees(arm_state.velocity[numJnt]))
#  save_file()

def callback_return(jnt_re):
  global jntReturn
#  rospy.loginfo("callback num : %s", jnt_re.header.frame_id)
  if en_fb is True:
    rospy.loginfo("callback_return")
  if not b_lock:
    jntReturn = jnt_re
  
def chk_return(jntPush):
  global jntReturn, b_lock
  if en_fb is True:
    rospy.loginfo("chk_return")
  if len(jntReturn.name) == len(jntPush.name):
    for i in range(len(jntPush.name)):
      if jntPush.name[i] != jntReturn.name[i]:
        return False
  else:
    return False
  if len(jntReturn.position) == len(jntPush.position):
    for i in range(len(jntPush.position)):
      if jntPush.position[i] != jntReturn.position[i]:
        return False
  else:
    return False
  if len(jntReturn.velocity) == len(jntPush.velocity):
    for i in range(len(jntPush.velocity)):
      if jntPush.velocity[i] != jntReturn.velocity[i]:
        return False
  else:
    return False
  if len(jntReturn.effort) == len(jntPush.effort):
    for i in range(len(jntPush.effort)):
      if jntPush.effort[i] != jntReturn.effort[i]:
        return False
  else:
    return False

  return True;

def reset_torque(pub):
  jntTorque = JointState()
  jntTorque.name = [nameJnt]
  jntTorque.position = []
  jntTorque.velocity = []
  jntTorque.effort = [0.0]

  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(jntTorque, pub);
    #print("122:chk = ", chk)
  
#  save_file(jntTorque.effort[0], file_name)

def move(state_1, state_move, state_2, mmin, mmax, pub, pub_status):
#  return val, effort_return;
#  reset_torque(pub)
  global en_fb, t_fb
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_1, pub);
    if abs(arm_state.position[numJnt]-state_1.position[numJnt]) > 0.01:
      chk = False
#    print("state_1 : chk = ", chk)

  t_start = time.time()
  rospy.loginfo("delay time")
  while (time.time()-t_start) < 0.05 and not rospy.is_shutdown(): a=0

  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_move, pub)
  t_start = time.time()
  first = False
  dt = 0.0
#  print(chk)
#  exit()

  tq1 = []
  tq2 = []
  b, a = signal.butter(2, 0.1, btype='lowpass')

  stat = JointState()
  stat.name = []
  stat.position = []
  stat.velocity = []
  stat.effort = []

  while not rospy.is_shutdown():
#    print("state_move : pos = ", arm_state.position[numJnt])
    if (state_move.velocity[0] < 0 and arm_state.position[numJnt] <= mmin) or (state_move.velocity[0] > 0 and arm_state.position[numJnt] >= mmax):
      rospy.loginfo("arm_state.position[numJnt] = %s", arm_state.position[numJnt])
      t_fb = time.time()
      en_fb = True
      break
#    if abs(arm_state.velocity[numJnt]) > 0.7:
#      print("if abs(arm_state.velocity[numJnt]) > 0.7")
#      break
    if arm_state.position[0] < -1.5 or arm_state.position[0] > +1.5:
      rospy.loginfo("arm_state.position[0]")
      break
    if arm_state.position[1] < -1.2214 or arm_state.position[1] > +2.25:
      rospy.loginfo("arm_state.position[1]")
      break
    if arm_state.position[2] < -3.45 or arm_state.position[2] > +1.25:
      rospy.loginfo("arm_state.position[2]")
      break
    if arm_state.position[3] < -3.15 or arm_state.position[3] > +3.15:
      rospy.loginfo("arm_state.position[3]")
      break
    if arm_state.position[4] < -3.15 or arm_state.position[4] > +0.35:
      rospy.loginfo("arm_state.position[4]")
      break
    if arm_state.position[5] < -3.15 or arm_state.position[5] > +3.15:
      rospy.loginfo("arm_state.position[5]")
      break

    if first:
      pos = np.array(arm_state.position)
      vel = np.array(arm_state.velocity)
      dt = time.time()-t_start
      acc = np.zeros(vel.shape)
      for n in range(len(vel)):
        acc[n] = (vel.item(n)-old_vel.item(n))/dt
      TqCurrent = current2torque(arm_state.effort[4], arm_state.velocity[4])
      vars = eq.get_vars(arm_state.position) # load equation parameters for calculating torque
      torque_with_ddq, torque_no_ddq = cal_torque(pos, vel, acc, vars)
      tq1.append(torque_with_ddq[4])
      tq2.append(TqCurrent)
      y1 = signal.filtfilt(b, a, tq1)
      y1 = signal.filtfilt(b, a, y1)
      y2 = signal.filtfilt(b, a, tq2)
      y2 = signal.filtfilt(b, a, y2)
      y3 = abs(y1-y2)
      indx = len(y3)
      stat.position = [float(torque_with_ddq[4]), float(TqCurrent), float(y1[indx-1]), float(y2[indx-1]), float(y3[indx-1])]
      stat.header.stamp = rospy.Time.now()
      pub_status.publish(stat)
      print(y3[indx-1])
      print(time.time()-t_start)
      if y3[indx-1] >= 0.25 and time.time()-t_start > 0.25:
        print("over_%f" % y3[indx-1])
        break

    old_vel = np.array(arm_state.velocity)
    first = True

  state_move.velocity = [0.0]
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_move, pub)

#  print("state_move : pos = ", arm_state.position[numJnt])
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_2, pub);
    if abs(arm_state.position[numJnt]-state_2.position[numJnt]) > 0.01:
      chk = False
#    print("state_2 : chk = ", chk)
#  t_start = time.time()
#  while (time.time()-t_start) < 0.05 and not rospy.is_shutdown():
#    print("delay time")

def save_file():
  global t_fp, t_offset, numJnt, file_path, file_name, effort
  if time.time()-t_fp > 0.001:
    fp = open(file_path + file_name, "a")
    fp.write("%lf %lf %lf %lf %lf\r\n" % (time.time()-t_offset, effort, math.degrees(arm_state.position[numJnt]), math.degrees(arm_state.velocity[numJnt]), arm_state.effort[numJnt]))
    fp.close()
    t_fp = time.time()

if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)

  sub = rospy.Subscriber("/joint_states", JointState, callback)
  sub_return = rospy.Subscriber("/cobot_dynamixel_driver/joint_states_return", JointState, callback_return)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  pub_status = rospy.Publisher("/cobot/status", JointState, queue_size=10)

  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntHome.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntHome.effort = []

  speed = 30
  jntGo = JointState()
  jntGo.name = [nameJnt]
  jntGo.position = []
  jntGo.velocity = [math.radians(-speed)]
  jntGo.effort = []

  jntEnd = JointState()
  jntEnd.name = ['J1','J2','J3','J4','J5','J6']
  jntEnd.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntEnd.position[numJnt] = math.radians(-180)
  jntEnd.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntEnd.effort = []

  #rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    jntGo.velocity = [math.radians(-speed)]
    move(jntHome, jntGo, jntEnd, math.radians(-180), 0.0, pub, pub_status)
    jntGo.velocity = [math.radians(speed)]
    move(jntEnd, jntGo, jntHome, math.radians(-180), 0.0, pub, pub_status)

#  jntGo.velocity = [math.radians(-speed)]
#  move(jntHome, jntGo, jntEnd, math.radians(-180), 0.0)
#  jntGo.velocity = [math.radians(speed)]
#  move(jntEnd, jntGo, jntHome, math.radians(-180), 0.0)

#  jntGo.velocity = [math.radians(-speed)]
#  move(jntHome, jntGo, jntEnd, math.radians(-180), 0.0)
#  jntGo.velocity = [math.radians(speed)]
#  move(jntEnd, jntGo, jntHome, math.radians(-180), 0.0)

  sub.unregister()
  sub_return.unregister()
  pub.unregister()
  pub_status.unregister()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
