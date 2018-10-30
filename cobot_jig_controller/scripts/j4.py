#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState

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

def callback(data):
  global arm_state
  arm_state = data
#  print(math.degrees(arm_state.velocity[numJnt]))
#  save_file()

def callback_return(jnt_re):
  global jntReturn
  rospy.loginfo("callback num : %s", jnt_re.header.frame_id)
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

def move(state_1, state_move, state_2, mmin, mmax):
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
#  print(chk)
#  exit()

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

#  jntGo.name = [nameJnt]
#  jntGo.position = [math.radians(-180)]
#  jntGo.velocity = [0.1]
#  jntGo.effort = []
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

  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntHome.position[numJnt] = math.radians(-90)
  jntHome.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntHome.effort = []

  speed = 90
  jntGo = JointState()
  jntGo.name = [nameJnt]
  jntGo.position = []
  jntGo.velocity = [math.radians(speed)]
  jntGo.effort = []

  jntEnd = JointState()
  jntEnd.name = ['J1','J2','J3','J4','J5','J6']
  jntEnd.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntEnd.position[numJnt] = math.radians(90)
  jntEnd.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntEnd.effort = []

  #rate = rospy.Rate(10) # 10hz
  move(jntHome, jntGo, jntEnd, math.radians(-90), math.radians(90))
  jntGo.velocity = [math.radians(-speed)]
  move(jntEnd, jntGo, jntHome, math.radians(-90), math.radians(90))

  jntGo.velocity = [math.radians(speed)]
  move(jntHome, jntGo, jntEnd, math.radians(-90), math.radians(90))
  jntGo.velocity = [math.radians(-speed)]
  move(jntEnd, jntGo, jntHome, math.radians(-90), math.radians(90))

  jntGo.velocity = [math.radians(speed)]
  move(jntHome, jntGo, jntEnd, math.radians(-90), math.radians(90))
  jntGo.velocity = [math.radians(-speed)]
  move(jntEnd, jntGo, jntHome, math.radians(-90), math.radians(90))

  sub.unregister()
  sub_return.unregister()
  pub.unregister()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
