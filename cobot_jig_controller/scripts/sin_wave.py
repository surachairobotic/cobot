#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time
import std_msgs.msg
from sensor_msgs.msg import JointState

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


def set_jnt(jnt, pub):
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
### def set_jnt(jnt, pub):

def callback(data):
  global arm_state
  arm_state = data
### def callback(data):

def callback_return(jnt_re):
  global jntReturn
#  rospy.loginfo("callback num : %s", jnt_re.header.frame_id)
  if en_fb is True:
    rospy.loginfo("callback_return")
  if not b_lock:
    jntReturn = jnt_re
### def callback_return(jnt_re):
  
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
### def chk_return(jntPush):

def reset_torque(pub):
  jntTorque = JointState()
  jntTorque.name = [nameJnt]
  jntTorque.position = []
  jntTorque.velocity = []
  jntTorque.effort = [0.0]

  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(jntTorque, pub);
### def reset_torque(pub):
  
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
  pub_target = rospy.Publisher("/cobot/target", JointState, queue_size=10)

  rate = rospy.Rate(100) # 100hz
  mmin = math.radians(-60)
  mmax = math.radians(60)
  speed = 10

  amp = 1.0
  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntHome.position[numJnt] = -((math.radians(180)-(2*amp))/2.0) #math.radians(-90)
  jntHome.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntHome.effort = []
  set_jnt(jntHome, pub)
  t_start = time.time()
  while (time.time()-t_start) < 5.0 and not rospy.is_shutdown(): a=0
  
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
  f = 0.25
  s = 1/f
  m = (math.pi-(-math.pi))/(s-0)
  c = 0.0 #(-math.pi)-(m*0);
  
  h = std_msgs.msg.Header()
  old_err = 0.0
  vec_err_v = [0.0, 0.0]
  rms_v = []
  rms_p = []
  kp = 0.0
  ki = 0.0
  kd = 0.0
  jntGo.velocity = [0.0]
  set_jnt(jntGo, pub)
  t_start = t_a = t_now = time.time()
  
  while (time.time()-t_start) < 15:
    dt = (time.time()-t_a)
    if dt > s:
      t_a = time.time()
    target = -2*math.pi*f*amp*math.sin(m*dt+c)
    p_target = amp*math.cos(m*dt+c)
    p_target -= (math.pi/2.0)
    err_v = target - arm_state.velocity[numJnt]
    err_p = p_target-arm_state.position[numJnt]
    rms_v.append(err_v*err_v)
    rms_p.append(err_p*err_p)
    vec_err_v.append( (old_err+err_v)*dt/2.0 )
    while len(vec_err_v) > 10:
      vec_err_v.pop()
#    print("tar=%s, m*dt=%s", target, m*dt+c)
    jntGo.velocity = [ target + (kp*err_v) + (ki*sum(vec_err_v)) + (kd*(old_err+err_v)/dt) ]
    jntTarget.position = [p_target, err_p]
    jntTarget.velocity = [ target + (kp*err_v) + (ki*sum(vec_err_v)) + (kd*(old_err+err_v)/dt), err_v ]
    jntTarget.effort = [ sum(rms_p)/len(rms_p), sum(rms_v)/len(rms_v) ]
    jntTarget.header.stamp = rospy.Time.now()
    pub_target.publish(jntTarget)
    rate.sleep()
    set_jnt(jntGo, pub)
###
  jntGo.velocity = [0.0]
  set_jnt(jntGo, pub)

  sub.unregister()
  sub_return.unregister()
  pub.unregister()
  pub_target.unregister()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
