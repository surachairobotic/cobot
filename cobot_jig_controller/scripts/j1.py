#!/usr/bin/env python

import subprocess
import signal

import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState

import numpy as np
from scipy import signal

from dynamic import *
from rosbag_record import RosbagRecord

# J1 = [-1.5   , +1.5 ] 0
# J2 = [-1.2214, +2.25] 1
# J3 = [-3.45  , +1.25] 2
# J4 = [-3.15  , +3.15] 3
# J5 = [-3.15  , +0.35] 4
# J6 = [-3.15  , +3.15] 5
MIN = [-1.5, -1.2214, -3.45, -3.15, -3.15, -3.15] #-math.pi/2
MAX = [ 1.5,  2.25,    1.25,  3.15,  0.0,  3.15] #math.pi/2
numJnt = 0
nameJnt = 'J1'

file_path = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/"
file_name = "tq_j0.txt"
arm_state = JointState()
jntReturn = JointState()
b_return_start = b_lock = False
t_fp = time.time()
t_offset = time.time()
effort = 0.0
t_fb = time.time()
en_fb = False

t_callback = t_tq = time.time()
old_vel = 0
first = False
dt = 0.0
tq1 = []
tq2 = []
b2, a2 = signal.butter(2, 1.5/((1/0.017)/2.0), btype='lowpass', analog=False)
stat = JointState()
stat.name = []
stat.position = []
stat.velocity = []
stat.effort = []
pub_status = rospy.Publisher("/cobot/status", JointState, queue_size=10)
tq_over = False

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

###################################

def callback(data):
  global arm_state, t_callback
  arm_state = data
  rospy.loginfo("[%d.%d] seq : %d,  t_call : %lf" % (arm_state.header.stamp.secs, arm_state.header.stamp.nsecs, arm_state.header.seq, time.time()-t_callback))
  t_callback = time.time()
  check_tq()

def check_tq():#(t_from_start):
  global arm_state, first, dt, tq1, tq2, b2, a2, old_vel, pub_status, t_tq, tq_over
  if first:
    pos = np.array(arm_state.position)
    vel = np.array(arm_state.velocity)
    eff = np.array(arm_state.effort)
    dt = time.time()-t_tq
    acc = np.zeros(vel.shape)
    for n in range(len(vel)):
      acc[n] = (vel.item(n)-old_vel.item(n))/dt
#      TqCurrent = current2torque(arm_state.effort[4], arm_state.velocity[4])
    vars = eq.get_vars(arm_state.position) # load equation parameters for calculating torque
    torque_with_ddq, torque_no_ddq = cal_torque(pos, vel, acc, vars)

    abc = [46.0, 1230.0, 250.0]
    current_cal = torque2current(torque_with_ddq[0], arm_state.velocity[0], abc)
    tq1.append(current_cal)
    tq2.append(eff[0])

    y1 = signal.lfilter(b2, a2, tq1)
    y2 = signal.lfilter(b2, a2, tq2)
    y3 = abs(y1-y2)
    indx = len(y3)
    print("dt = %lf - [%d, %d, %d]" % (dt, len(y1), len(y2), len(y3)))
    stat.position = [float(current_cal), float(eff[0]), float(y1[indx-1]), float(y2[indx-1]), float(y3[indx-1])]
    stat.header.stamp = rospy.Time.now()
    pub_status.publish(stat)
    save_file(stat)
    if y3[indx-1] >= 550:# and time.time()-t_from_start > 0.5:
      tq_over = True

  old_vel = np.array(arm_state.velocity)
  first = True
#  print(math.degrees(arm_state.velocity[numJnt]))
  t_tq = time.time()

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

def move(state_1, state_move, state_2, mmin, mmax, pub, pub_status):
#  return val, effort_return;
#  reset_torque(pub)
  global en_fb, t_fb, tq_over, tq1, tq2
  rate = rospy.Rate(25)
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_1, pub);
    if abs(arm_state.position[numJnt]-state_1.position[numJnt]) > 0.01:
      chk = False
    rate.sleep()

  rospy.loginfo("delay time")
  t_start = time.time()
  while (time.time()-t_start) < 0.05 and not rospy.is_shutdown():
    a=0
    rate.sleep()
     
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_move, pub)
    rate.sleep() 

  stat = JointState()
  stat.name = []
  stat.position = []
  stat.velocity = []
  stat.effort = []
  tq_over = False
  t_st = time.time()
  while not rospy.is_shutdown():
    if (state_move.velocity[0] < 0 and arm_state.position[numJnt] <= mmin) or (state_move.velocity[0] > 0 and arm_state.position[numJnt] >= mmax):
      rospy.loginfo("arm_state.position[numJnt] = %s", arm_state.position[numJnt])
      t_fb = time.time()
      en_fb = True
      break
#    if abs(arm_state.velocity[numJnt]) > 0.7:
#      print("if abs(arm_state.velocity[numJnt]) > 0.7")
#      break
    if jnt_over_limit(arm_state):
      print("jnt_over_limit(arm_state)")
      break
    if tq_over:
      print("tq_over")
      break
    rate.sleep() 
    
  state_move.velocity = [0.0]
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_move, pub)
    rate.sleep()
     
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(state_2, pub);
    if abs(arm_state.position[numJnt]-state_2.position[numJnt]) > 0.01:
      chk = False
    rate.sleep() 
    
  tq1 = []
  tq2 = []

def save_file(stat):
  global t_fp, t_offset, numJnt, file_path, file_name
  if time.time()-t_fp > 0.001:
    fp = open(file_path + file_name, "a")
    fp.write("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n" % (time.time()-t_offset, math.degrees(arm_state.position[numJnt]), math.degrees(arm_state.velocity[numJnt]), arm_state.effort[numJnt], stat.position[0], stat.position[2], stat.position[3], stat.position[4]))
    fp.close()
    t_fp = time.time()

if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)

  sub = rospy.Subscriber("/joint_states", JointState, callback)
  sub_return = rospy.Subscriber("/cobot_dynamixel_driver/joint_states_return", JointState, callback_return)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)

  mmin = math.radians(-60)
  mmax = math.radians(60)
  speed = 30

  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntHome.position[numJnt] = mmax
  jntHome.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntHome.effort = []

  set_jnt(jntHome, pub)
  t_start = time.time()
  while (time.time()-t_start) < 2.5 and not rospy.is_shutdown():
    a=0

#  jntHome.position[4] = math.radians(-90)
#  jntHome.position[2] = math.radians(-90)
  set_jnt(jntHome, pub)
  t_start = time.time()
  while (time.time()-t_start) < 2.5 and not rospy.is_shutdown():
    a=0

#  jntHome.position[1] = math.radians(90)
  set_jnt(jntHome, pub)
  t_start = time.time()
  while (time.time()-t_start) < 4.0 and not rospy.is_shutdown():
    a=0

  jntGo = JointState()
  jntGo.name = [nameJnt]
  jntGo.position = []
  jntGo.velocity = [math.radians(-speed)]
  jntGo.effort = []

  jntEnd = JointState()
  jntEnd.name = ['J1','J2','J3','J4','J5','J6']
  jntEnd.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntEnd.position[numJnt] = mmin
#  jntEnd.position[4] = math.radians(-90)
#  jntEnd.position[2] = math.radians(-90)
#  jntEnd.position[1] = math.radians(90)
  jntEnd.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntEnd.effort = []

#  rec_folder = "/home/dell/catkin_ws/src/cobot/record/wave_moving_4/"
#  rosbag_record = RosbagRecord(rec_folder)
  t_start = time.time()
  while (time.time()-t_start) < 2.5 and not rospy.is_shutdown():
    a=0

  while not rospy.is_shutdown():
    jntGo.velocity = [math.radians(-speed)]
    move(jntHome, jntGo, jntEnd, mmin, mmax, pub, pub_status)
    jntGo.velocity = [math.radians(speed)]
    move(jntEnd, jntGo, jntHome, mmin, mmax, pub, pub_status)

  #rate = rospy.Rate(10) # 10hz
#  move(jntHome, jntGo, jntEnd, mmin, mmax)
#  jntGo.velocity = [math.radians(speed)]
#  move(jntEnd, jntGo, jntHome, mmin, mmax)

#  jntGo.velocity = [math.radians(-speed)]
#  move(jntHome, jntGo, jntEnd, mmin, mmax)
#  jntGo.velocity = [math.radians(speed)]
#  move(jntEnd, jntGo, jntHome, mmin, mmax)

#  jntGo.velocity = [math.radians(-speed)]
#  move(jntHome, jntGo, jntEnd, mmin, mmax)
#  jntGo.velocity = [math.radians(speed)]
#  move(jntEnd, jntGo, jntHome, mmin, mmax)

  sub.unregister()
  sub_return.unregister()
  pub.unregister()
  pub_status.unregister()
  
  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
