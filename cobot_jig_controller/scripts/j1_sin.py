#!/usr/bin/env python

import subprocess
import signal

import sys
import os
import math
import rospy
import copy
import time
import std_msgs.msg
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
numJnt = 3
nameJnt = 'J4'
b_save_file = False

file_path = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/"
file_name = "tq_j2.txt"
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

def set_jnt(jnt, pub):
  pub.publish(jnt)
  return True;

###################################

def callback(data):
  global arm_state
  arm_state = data
#  check_tq()

def check_tq():
  global arm_state, first, dt, tq1, tq2, b2, a2, old_vel, pub_status, t_callback, tq_over
  if first:
    pos = np.array(arm_state.position)
    vel = np.array(arm_state.velocity)
    eff = np.array(arm_state.effort)
    dt = time.time()-t_callback
    acc = np.zeros(vel.shape)
    for n in range(len(vel)):
      acc[n] = (vel.item(n)-old_vel.item(n))/dt
#      TqCurrent = current2torque(arm_state.effort[4], arm_state.velocity[4])
    vars = eq.get_vars(arm_state.position) # load equation parameters for calculating torque
    torque_with_ddq, torque_no_ddq = cal_torque(pos, vel, acc, vars)
    current_cal = torque2current(torque_with_ddq[0], arm_state.velocity[0])
    tq1.append(current_cal)
    tq2.append(eff[0])

    y1 = signal.lfilter(b2, a2, tq1)
    y2 = signal.lfilter(b2, a2, tq2)
    y3 = abs(y1-y2)
#    indx = len(y3)
    stat.position = [float(current_cal), float(eff[0]), float(y1[len(y1)-1]), float(y2[len(y2)-1]), float(y3[len(y3)-1])]

    stat.header = arm_state.header
    pub_status.publish(arm_state)

    if y3[len(y3)-1] >= 750 and time.time()-t_start > 0.5:
      tq_over = False
#      print("over_%f" % y3[indx-1])
#        break

  old_vel = np.array(arm_state.velocity)
  first = True
#  print(math.degrees(arm_state.velocity[numJnt]))
  t_callback = time.time()

def reset_torque(pub_tq):
  jntTorque = JointState()
  jntTorque.name = [nameJnt]
  jntTorque.position = []
  jntTorque.velocity = []
  jntTorque.effort = [0.0]

  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(jntTorque, pub_tq);
    #print("122:chk = ", chk)
  
def save_file(stat):
  global t_fp, t_offset, numJnt, file_path, file_name
  if time.time()-t_fp > 0.001:
    fp = open(file_path + file_name, "a")
    fp.write("%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n" % (stat.header.stamp.nsecs, stat.header.seq, math.degrees(arm_state.position[numJnt]), math.degrees(arm_state.velocity[numJnt]), arm_state.effort[numJnt], stat.position[0], stat.position[2], stat.position[3], stat.position[4]))
    fp.close()
    t_fp = time.time()

def find_pid(pub_target, pid_pub_goal, kp, kpp, ki, fp):
  global b_save_file, tq_over, tq1, tq2
  rospy.loginfo('find_pid')
  rate = rospy.Rate(100) # 100hz
  mmin = math.radians(-60)
  mmax = math.radians(60)
  speed = 10

  amp = math.radians(60)
#  amp = 1.0
  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  if nameJnt is 'J2' or nameJnt is 'J3':
    jntHome.position[4] = -3.0
  if nameJnt is 'J2':
    jntHome.position[0] = math.radians(90)
  if nameJnt is 'J3':
    jntHome.position[numJnt] = math.radians(45)-((math.radians(180)-(2*amp))/2.0)
  else:
    jntHome.position[numJnt] = ((math.radians(180)-(2*amp))/2.0)
#  jntHome.position[numJnt] = 0.0
  if nameJnt is 'J5':
    jntHome.position[numJnt] += math.radians(-90)
  jntHome.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntHome.effort = []
  
  jntGo = JointState()
  jntGo.name = [nameJnt]
  jntGo.position = []
  jntGo.velocity = [math.radians(-speed)]
  jntGo.effort = []
  
  jntTarget = JointState()
  jntTarget.name = []
  jntTarget.position = [0.0]
  jntTarget.velocity = [0.0]
  jntTarget.effort = []

### velocity control to sin wave ::: V2
  f = 0.5
  s = 1/f
  m = (math.pi-(-math.pi))/(s-0)
  c = 0.0 #(-math.pi)-(m*0);
  
  h = std_msgs.msg.Header()
  old_err = 0.0
  vec_err_v = [0.0, 0.0]
  rms_v = []
  rms_p = []

#  kp = 0.1
#  kpp = 0.1
#  ki = 0.005
  kd = 0.0

#  jntHome.position[1] = math.radians(90)
##  jntHome.position[1] += math.radians(90)
  rospy.loginfo('set_jnt A')
  set_jnt(jntHome, pid_pub_goal)
  rospy.loginfo('set_jnt B')
  t_start = time.time()
  rospy.loginfo('set_jnt(jntHome, pub) - start')
  while (time.time()-t_start) < 5.5 and not rospy.is_shutdown():
    set_jnt(jntHome, pid_pub_goal)
  rospy.loginfo('set_jnt(jntHome, pub) - end')

  rosbag_record = RosbagRecord()
  t_start = time.time()
  while (time.time()-t_start) < 2.5 and not rospy.is_shutdown():
    set_jnt(jntHome, pid_pub_goal)

  jntGo.velocity = [0.0]
  set_jnt(jntGo, pid_pub_goal)
  t_start = t_a = t_now = time.time()
  
#  while not ((time.time()-t_start) > 15 and arm_state.velocity[numJnt] < 0.2) and not rospy.is_shutdown():
  stat = JointState()
  stat.name = []
  stat.position = []
  stat.velocity = []
  stat.effort = []
  tq_over = False

  while not (rospy.is_shutdown() or ((time.time()-t_start) > 15 and arm_state.velocity[numJnt] < math.radians(15))):
#    check_tq()
    if tq_over:
      break
    dt = (time.time()-t_a)
    if dt > s:
      t_a = time.time()
#    target = -2*math.pi*f*amp*math.sin(m*dt+c)
#    p_target = amp*math.cos(m*dt+c)
    target = 2*math.pi*f*amp*math.cos(m*dt+c)
    p_target = amp*math.sin(m*dt+c)
    if nameJnt is 'J5':
      p_target += math.radians(-90)
    err_v = target - arm_state.velocity[numJnt]
    err_p = p_target-arm_state.position[numJnt]
    rms_v.append(err_v*err_v)
    rms_p.append(err_p*err_p)
    vec_err_v.append( (old_err+err_v)*dt/2.0 )
    old_err = err_v
    while len(vec_err_v) > 10:
      vec_err_v.pop()
#    print("tar=%s, m*dt=%s", target, m*dt+c)
    jntGo.velocity = [ target + (kp*err_v) + (kpp*err_p) + (ki*sum(vec_err_v)) ]#+ (kd*(old_err+err_v)/dt) ]
    jntTarget.position = [p_target, err_p]
    jntTarget.velocity = [ target + (kp*err_v) + (kpp*err_p) + (ki*sum(vec_err_v)) ]# + (kd*(old_err+err_v)/dt), err_v ]
    jntTarget.effort = [ sum(rms_p)/len(rms_p), sum(rms_v)/len(rms_v) ]
    jntTarget.header.stamp = rospy.Time.now()
#    pub_target.publish(jntTarget)
#    rate.sleep()
    set_jnt(jntGo, pid_pub_goal)
###
  jntGo.velocity = [jntGo.velocity[0]*0.4]
  set_jnt(jntGo, pid_pub_goal)
  jntGo.velocity = [0.0]
  set_jnt(jntGo, pid_pub_goal)
  tq1 = []
  tq2 = []

if __name__ == '__main__':
  pub_goal = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  rospy.init_node('init_torque', anonymous=True)

  sub_joint_state = rospy.Subscriber("/joint_states", JointState, callback)
  global b_save_file

  fp = 0
  if b_save_file:
    fp = open("/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/pid_gain/" + nameJnt + ".txt", "a")
    fp.write("--------------------")

#  while not rospy.is_shutdown():
  find_pid(pub_status, pub_goal, 0.0, 0.0, 0.0, fp)

  if b_save_file:
    fp.write("--------end---------")
    fp.close()
  sub_joint_state.unregister()
  pub_goal.unregister()
  pub_status.unregister()
  
  print("OK !!!")
  rospy.spin()
  
