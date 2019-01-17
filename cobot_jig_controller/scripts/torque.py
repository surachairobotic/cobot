#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState

# J1 = [-1.5   , +1.5 ]
# J2 = [-1.2214, +2.25]
# J3 = [-3.45  , +1.25]
# J4 = [-3.15  , +3.15]
# J5 = [-3.15  , +0.35]
# J6 = [-3.15  , +3.15]

file_path = "/home/mtec/catkin_ws/src/cobot/cobot_jig_controller/results/torque_j5/set_over/"
file_name = ""
name = "torque_j5_over_"
numJnt = 4
nameJnt = 'J5'
arm_state = JointState()
jntReturn = JointState()
b_return_start = False
t_fp = time.time()
t_offset = time.time()
effort = 0.0
b_save = False

def set_jnt(jnt, pub):
  print('set_jnt')
  global b_return_start, effort
  rate = rospy.Rate(10) # 10hz
  jnt.header.frame_id = "/torque.py"

  chk = False
  while not chk and not rospy.is_shutdown():
    pub.publish(jnt)
    rate.sleep()
    chk = chk_return(jnt)
    print("chk_return = ", chk)

  if len(jnt.effort) != 0:
#    save_file(jnt.effort[0], file_name)
    effort = jnt.effort[0]
    return True;
  else:
    effort = 0.0

  b_stop = False
  while not b_stop and not rospy.is_shutdown():
      b_stop = True
      rate.sleep()
#      save_file(0.0, file_name)
      for v in arm_state.velocity:
          if abs(v) > 0.005:
              b_stop = False
              break
  
  b_ok = True
  for i in range(len(jnt.name)):
    for j in range(len(arm_state.name)):
      if jnt.name[i] == arm_state.name[j]:
        if abs(arm_state.position[j]-jnt.position[i]) > 0.005:
          print("arm[", j, "]=", arm_state.position[j], " - jnt[", i, "]=", jnt.position[i])
          b_ok = False
          break
  
#  save_file(999)
  print('end set_jnt')
  return b_ok;
###################################

def callback(data):
  global arm_state, b_save
  arm_state = data
  if b_save:
    save_file()

def callback_return(jnt_re):
  global jntReturn
  jntReturn = jnt_re
  
def chk_return(jntPush):
  global jntReturn
  print("=========== jntReturn ============")
  print(jntReturn)
  print("==================================")
  print("============ jntPush ============")
  print(jntPush)
  print("==================================")
  b_return = True
  if len(jntReturn.name) == len(jntPush.name):
    for i in range(0, len(jntPush.name)-1):
      if jntPush.name[i] != jntReturn.name[i]:
        b_return = False
        break
  else:
    b_return = False
  if len(jntReturn.position) == len(jntPush.position) and b_return:
    for i in range(0, len(jntPush.position)-1):
      if jntPush.position[i] != jntReturn.position[i]:
        b_return = False
        break
  else:
    b_return = False
  if len(jntReturn.velocity) == len(jntPush.velocity) and b_return:
    for i in range(0, len(jntPush.velocity)-1):
      if jntPush.velocity[i] != jntReturn.velocity[i]:
        b_return = False
        break
  else:
    b_return = False
  if len(jntReturn.effort) == len(jntPush.effort) and b_return:
    for i in range(0, len(jntPush.effort)-1):
      if jntPush.effort[i] != jntReturn.effort[i]:
        b_return = False
        break
  else:
    b_return = False

  return b_return;

def reset_torque(pub):
  jntTorque = JointState()
  jntTorque.header.frame_id = "/torque.py"
  jntTorque.name = [nameJnt]
  jntTorque.position = []
  jntTorque.velocity = []
  jntTorque.effort = [0.0]

  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(jntTorque, pub);
    print("122:chk = ", chk)
  
#  save_file(jntTorque.effort[0], file_name)

def save_file():
  global t_fp, t_offset, numJnt, file_path, file_name, effort
  if time.time()-t_fp > 0.001:
    fp = open(file_path + file_name, "a")
    fp.write("%lf %lf %lf %lf %lf\r\n" % (time.time()-t_offset, effort, math.degrees(arm_state.position[numJnt]), math.degrees(arm_state.velocity[numJnt]), arm_state.effort[numJnt]))
    fp.close()
    t_fp = time.time()

def torque_finding(theta, direction):
  sub = rospy.Subscriber("/joint_states", JointState, callback)
  sub_return = rospy.Subscriber("/cobot_dynamixel_driver/joint_states_return", JointState, callback_return)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)

  global t_fp, numJnt, file_path, file_name, b_save
  t_fp = time.time()

  set_pos = [0.0]*6
  set_pos[0] = 0.0
  set_pos[1] = 0.0
  set_pos[2] = 0.0
  set_pos[3] = 0.0
  set_pos[4] = 0.0
  set_pos[5] = 0.0
  set_pos[numJnt] = math.radians(theta)

  if direction:
    file_name = name + str(theta) + "_up.txt"
  else:
    file_name = name + str(theta) + "_down.txt"
  fp = open(file_path + file_name, "w")
  fp.close()

  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = set_pos
  jntHome.velocity = [0.5,0.5,0.5,0.5,0.5,0.5]
  jntHome.effort = []
  jntGo = JointState()

  rate = rospy.Rate(10) # 10hz
  
  reset_torque(pub)
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(jntHome, pub);
    print("146:chk = ", chk)
    rate.sleep()
#  exit()

  t_start = time.time()
  while (time.time()-t_start) < 1.0 and not rospy.is_shutdown():
    print("delay time")
    rate.sleep()

  val = 0.0
  out = False
  b_save = True
  while not rospy.is_shutdown():
    print(val)
    effort = val
    print(arm_state)
    if arm_state.velocity[numJnt] != 0:
      print("if arm_state.velocity[numJnt] != 0")
      t_start = time.time()
      print("while (time.time()-t_start) < 1.0")
      while (time.time()-t_start) < 1.0 and not rospy.is_shutdown():
#        save_file(val, file_name)
#        print("while (time.time()-t_start) < 1.0")
        if abs(arm_state.velocity[numJnt]) > 0.7:
          print("if abs(arm_state.velocity[numJnt]) > 0.7")
          out = True
          break
        if abs(arm_state.position[numJnt]-jntHome.position[numJnt]) > math.radians(1):
          print("if arm_state.position[numJnt] > math.radians(2)")
          out = True
          break
        if arm_state.position[0] < -1.3 or arm_state.position[0] > +1.3:
          print("arm_state.position[0]")
          out = True
          break
        if arm_state.position[1] < -1.1 or arm_state.position[1] > +2.1:
          print("arm_state.position[1]")
          out = True
          break
        if arm_state.position[2] < -3.3 or arm_state.position[2] > +1.1:
          print("arm_state.position[2]")
          out = True
          break
        if arm_state.position[3] < -3.0 or arm_state.position[3] > +3.0:
          print("arm_state.position[3]")
          out = True
          break
        if arm_state.position[4] < -3.3 or arm_state.position[4] > +0.37:
          print("arm_state.position[4]")
          out = True
          break
        if arm_state.position[5] < -3.0 or arm_state.position[5] > +3.0:
          print("arm_state.position[5]")
          out = True
          break
      if arm_state.velocity[numJnt] != 0 and out:
        print("arm_state.velocity[numJnt] != 0")
        out = True
        break
      reset_torque(pub)
      chk = False
      while not chk and not rospy.is_shutdown():
        print("182:chk = ", chk)
        chk = set_jnt(jntHome, pub);

    if out:
      break
    if direction:
      val -= 10
    else:
      val += 10
    jntGo.name = [nameJnt]
    jntGo.position = []
    jntGo.velocity = []
    jntGo.effort = [val]
    chk = False
    while not chk and not rospy.is_shutdown():
      chk = set_jnt(jntGo, pub);
      print("194:chk = ", chk)
    time.sleep(0.5)

  b_save = False
  effort_return = arm_state.effort[numJnt]
  reset_torque(pub)
  chk = False
  while not chk and not rospy.is_shutdown():
    chk = set_jnt(jntHome, pub);
    print("202:chk = ", chk)

  sub.unregister()
  sub_return.unregister()
  pub.unregister()
  
  return val, effort_return;

if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)
  
  file_all = "torque_j5_over_all.txt"
  fp = open(file_path + file_all, "w")
  fp.close()
  
  for i in range(20, -21, -5):
    print(i)
    t_fp = time.time()
    t_offset = time.time()
    push_a, state_a = torque_finding(i, True)
    t_start = time.time()
    while (time.time()-t_start) < 1.0 and not rospy.is_shutdown():
      print("delay time : a")
    push_b, state_b = torque_finding(i, False)
    t_start = time.time()
    while (time.time()-t_start) < 1.0 and not rospy.is_shutdown():
      print("delay time : b")

    fp = open(file_path + file_all, "a")
    fp.write("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\r\n" % (time.time()-t_offset, push_a, state_a, push_b, state_b, 0.0, 0.0, 0.0, 0.0, i, 0.0))
    fp.close()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

