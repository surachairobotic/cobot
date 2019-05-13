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
MAX = [ 1.5,  2.25,    1.25,  3.15,  0.0,   3.15] #math.pi/2
numJnt = 4
nameJnt = 'J5'

file_path = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/"
file_name = "tq_j0.txt"
arm_state = JointState()
jntReturn = JointState()
old_state = JointState()
b_return_start = b_lock = False
t_fp = time.time()
t_offset = time.time()
effort = 0.0
t_fb = time.time()
en_fb = False

t_callback = time.time()
old_vel = 0
first = False
dt = 0.0
ABC = []
list_current_cal = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
list_current = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
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
#  global b_return_start, effort, en_fb, t_fb
#  rate = rospy.Rate(50) # 100hz
  pub.publish(jnt)

#  chk = False
#  num = 0
#  while not chk and not rospy.is_shutdown():
#    jnt.header.frame_id = str(num)
#    num += 1
#    pub.publish(jnt)
#    rate.sleep()
#    b_lock = True
#    chk = chk_return(jnt)
#    b_lock = False
#    if en_fb is True:
#      rospy.loginfo("frame_id send : %s", jnt.header.frame_id)

#  if en_fb is True:
#    rospy.loginfo("t_fb : %s", time.time()-t_fb)
#    rospy.loginfo("seq : %s", jntReturn.header.seq)
#    en_fb = False
  return True;

###################################

def callback(data):
  global arm_state, t_callback
  arm_state = data
#  rospy.loginfo("[%d.%d] seq : %d,  t_call : %lf" % (arm_state.header.stamp.secs, arm_state.header.stamp.nsecs, arm_state.header.seq, time.time()-t_callback))
  t_callback = time.time()
  check_tq()

def check_tq():#(t_from_start):
  global arm_state, first, dt, tq1, tq2, b2, a2, old_vel, pub_status, tq_over, old_state, ABC, stat
  if first:
    pos = np.array(arm_state.position)
    vel = np.array(arm_state.velocity)
    eff = np.array(arm_state.effort)
    dt = (arm_state.header.stamp-old_state.header.stamp).to_sec()
    acc = np.zeros(vel.shape)
    for n in range(len(vel)):
      acc[n] = (vel.item(n)-old_vel.item(n))/dt
#      TqCurrent = current2torque(arm_state.effort[4], arm_state.velocity[4])
    vars = eq.get_vars(arm_state.position) # load equation parameters for calculating torque
    torque_with_ddq, torque_no_ddq = cal_torque(pos, vel, acc, vars)

    filter_current_cal = [[], [], [], [], [], []]
    filter_current = [[], [], [], [], [], []]
    err = [[], [], [], [], [], []]
    for i in range(6):
      list_current_cal[i].append(torque2current(torque_with_ddq[i], arm_state.velocity[i], ABC[i]))
      list_current[i].append(eff[i])
      x = signal.lfilter(b2, a2, list_current_cal[i])
      y = signal.lfilter(b2, a2, list_current[i])
#      filter_current_cal[i] = signal.lfilter(b2, a2, list_current_cal[i])
#      filter_current[i] = signal.lfilter(b2, a2, list_current[i])
      filter_current_cal[i] = x
      filter_current[i] = y
      err[i] = abs( filter_current_cal[i] - filter_current[i] )

#    y3 = abs(y1-y2)
#    indx = len(y3)
#    print("dt = %lf - [%d, %d, %d]" % (dt, len(y1), len(y2), len(y3)))
#    stat.position = [float(current_cal), float(eff[numJnt]), float(y1[indx-1]), float(y2[indx-1]), float(y3[indx-1])]
    stat.position = []
    stat.velocity = []
    eff = []
    for i in range(6):
      stat.position.append( list_current_cal[i][len(list_current_cal[i])-1] )
      stat.position.append( list_current[i][len(list_current[i])-1] )
      stat.velocity.append( filter_current_cal[i][len(filter_current_cal[i])-1] )
      stat.velocity.append( filter_current[i][len(filter_current[i])-1] )
      eff.append( err[i][len(err[i])-1] )
    stat.header.stamp = arm_state.header.stamp
    stat.effort = eff
    pub_status.publish(stat)
#    save_file(stat)
#    if y3[indx-1] >= 550:# and time.time()-t_from_start > 0.5:
#      tq_over = True

  old_vel = np.array(arm_state.velocity)
  old_state = arm_state
  first = True

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

if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)

  global arm_state, ABC, first, list_current_cal, list_current, stat
    
#  abc = [46.0, 1230.0, 250.0]
#  ABC.append(abc)
#  abc = [-168.0, -855.0, -390.0]
#  ABC.append(abc)
#  abc = [0.0, 0.0, 0.0]
#  ABC.append(abc)
#  abc = [0.0, 0.0, 0.0]
#  ABC.append(abc)
#  abc = [205.0, 20.0, 60.0]
#  ABC.append(abc)
#  abc = [0.0, 0.0, 0.0]
#  ABC.append(abc)
  ABC = [ [ 355.0, 1240.0,  245.0],
          [-177.0, -690.0, -650.0],
          [ 180.0,  564.0,  206.0],
          [ 256.0,  689.0,  232.0],
          [ 241.0,   22.0,   58.0],
          [   0.0,    0.0,    0.0] ]

  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  jj = JointState()
  jj.name = ['J6']
  jj.position = []
  jj.velocity = [0.5]
  jj.effort = []
  while not rospy.is_shutdown():
    set_jnt(jj, pub)
  exit()

  sub = rospy.Subscriber("/joint_states", JointState, callback)
  sub_return = rospy.Subscriber("/cobot_dynamixel_driver/joint_states_return", JointState, callback_return)

  mmin = [math.radians(30), math.radians(-10), math.radians(-10), math.radians(-10), math.radians(-135), math.radians(-10)]
  mmax = [math.radians(60), math.radians(10), math.radians(10), math.radians(10), math.radians(-105), math.radians(10)]
  speed = [math.radians(10), math.radians(10), math.radians(15), math.radians(15), math.radians(30), math.radians(0)]
  
  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntHome.velocity = [0.25,0.25,0.25,0.25,0.25,0.25]
  jntHome.effort = []
  jntStop = JointState()
  jntStop.name = ['J1','J2','J3','J4','J5','J6']
  jntStop.position = []
  jntStop.velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntStop.effort = []

  rate = rospy.Rate(50) # 100hz
  stop = rospy.Rate(1) # 100hz

  set_jnt(jj, pub)
  t_start = time.time()

  jntGo = jntHome
  jntGo.position = []
  jntGo.velocity = speed
  
#  err_min = [9999.99, 9999.99, 9999.99, 9999.99, 9999.99, 9999.99]
#  err_max = [-9999.99, -9999.99, -9999.99, -9999.99, -9999.99, -9999.99]
  err_max = [380, 870, 470, 340, 140, 110]

  t_detect = time.time()
  count = 0
  while not rospy.is_shutdown():
#    rospy.loginfo("count = %d", count)
    b_tq_stop = False
    set_jnt(jntGo, pub)
    if not(len(arm_state.position) is 6) or not(len(arm_state.velocity) is 6) or not(len(arm_state.effort) is 6):
      continue
    for i in range(6):
      if arm_state.position[i] <= mmin[i]:
        jntGo.velocity[i] = abs(jntGo.velocity[i])
      elif arm_state.position[i] >= mmax[i]:
        jntGo.velocity[i] = -abs(jntGo.velocity[i])

    if jnt_over_limit(arm_state):
      print("jnt_over_limit(arm_state)")
      break
#    if len(stat.effort) is 6:
#      print("[%d, %d, %d]" % (len(stat.effort), len(err_min), len(err_max)))
#      b_in = False
#      for i in range(6):
#        if stat.effort[i] > err_max[i]:
#          err_max[i] = stat.effort[i]
#          b_in = True
#      if b_in:
#        print("err_max")
#        print(err_max)
    if not(len(stat.position) is 6) or not(len(stat.velocity) is 6) or not(len(stat.effort) is 6):
      continue
    for i in range(6):
      if stat.effort[i] >= err_max[i]:
        print("stat.effort[%d] is over : %lf" % (i, stat.effort[i]))
        b_tq_stop = True

    if b_tq_stop and (time.time()-t_detect) >= 0.5:
#      print(stat)
      print("------------------")
      stat.effort = []
      set_jnt(jntStop, pub)
      first = False
      list_current_cal = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
      list_current = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
#      list_current = [[0], [0], [0], [0], [0], [0]]
      time.sleep(2)
      t_detect = time.time()

    rate.sleep()

  print("stop . . .")
  print(jntStop)
  while not rospy.is_shutdown():
    set_jnt(jntStop, pub)
    rate.sleep()
    
  sub.unregister()
  sub_return.unregister()
  pub.unregister()
  pub_status.unregister()
  
  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
