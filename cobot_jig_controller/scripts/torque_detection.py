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
  pub.publish(jnt)
  return True;

def callback(data):
  global arm_state, t_callback
  arm_state = data
  t_callback = time.time()
  check_tq()

def check_tq():
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
      if len(list_current_cal[i]) > 1000:
        list_current_cal[i].pop(0)
      if len(list_current[i]) > 1000:
        list_current[i].pop(0)
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

if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)

  global arm_state, ABC, first, list_current_cal, list_current, stat
    
  ABC = [ [ 355.0, 1240.0,  245.0],
          [-177.0, -690.0, -650.0],
          [ 180.0,  564.0,  206.0],
          [ 256.0,  689.0,  232.0],
          [ 241.0,   22.0,   58.0],
          [   0.0,    0.0,    0.0] ]

  sub = rospy.Subscriber("/joint_states", JointState, callback)
  sub_return = rospy.Subscriber("/cobot_dynamixel_driver/joint_states_return", JointState, callback_return)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  pub_tq = rospy.Publisher("/cobot/torque_detection", Bool, queue_size=10)
  
  rate = rospy.Rate(50) # 100hz
  stop = rospy.Rate(1) # 100hz

  err_max = [380, 870, 470, 340, 140, 110]

  t = [time.time(), time.time(), time.time(), time.time(), time.time(), time.time()]
  count = 0
  while not rospy.is_shutdown():
    b_tq_stop = False

#    print(arm_state)
    if len(arm_state.position) is 6 and first is True and len(stat.effort) is 6:
      if jnt_over_limit(arm_state):
        print("jnt_over_limit(arm_state)")
#        break
      for i in range(6):
        if stat.effort[i] >= err_max[i] and abs(arm_state.velocity[i]) > 0.0 and (time.time()-t[i]) > 0.5:
          print("stat.effort[%d] is over : %lf" % (i, stat.effort[i]))
          b_tq_stop = True
        elif abs(arm_state.velocity[i]) < 0.001:
          t[i] = time.time()
    rate.sleep()
    
  sub.unregister()
  sub_return.unregister()
  pub.unregister()
  pub_status.unregister()
  
  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
