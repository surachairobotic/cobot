#!/usr/bin/env python

import rospy
import time

from sensor_msgs.msg import JointState
from dynamic import *

arm_state = JointState()
ABC = []
curr = [0.0]*6
nameJnt = 'J4'
numJnt = int(nameJnt[1])-1

def main():
  rospy.init_node('position_teach', anonymous=True)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  sub = rospy.Subscriber("/joint_states", JointState, callback)

  global ABC, curr, arm_state
  rate = rospy.Rate(50)

  ABC = [ [ 355.0, 1240.0,  245.0],
          [-177.0, -690.0, -650.0],
          [ 180.0,  564.0,  206.0],
          [ 256.0,  689.0,  232.0],
          [ 241.0,   22.0,   58.0],
          [   0.0,    0.0,    0.0] ];
  
  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6'];
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0];
  jntHome.velocity = [0.25,0.25,0.25,0.25,0.25,0.25];
  jntHome.effort = [];
  
  t_start = time.time()
  while (time.time()-t_start) < 3.0 and not rospy.is_shutdown():
    set_jnt(jntHome, pub)
    rate.sleep()
  rospy.loginfo("step 1 : OK")

  jntSend = JointState()
  jntSend.name = [nameJnt]
  jntSend.position = [0.0];
  jntSend.velocity = [];
  jntSend.effort = [];

  t_start = time.time()
  while not rospy.is_shutdown():
    set_jnt(jntSend, pub)
    rate.sleep()
  rospy.loginfo("step 2 : OK")

  sub.unregister()
  pub.unregister()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

def check_tq():#(t_from_start):
  global arm_state, ABC, curr

  pos = np.array(arm_state.position)
  vel = np.zeros(pos.shape)
  acc = np.zeros(pos.shape)

  vars = eq.get_vars(arm_state.position) # load equation parameters for calculating torque
  torque_with_ddq, torque_no_ddq = cal_torque(pos, vel, acc, vars)

  curr = [0.0]*6
  for i in range(6):
    curr.append(torque2current(torque_with_ddq[i], vel[i], ABC[i]))

  curr[1] = -186.231185 * torque_with_ddq[1]
  if torque_with_ddq[2] > 0:
    curr[2] = 143.031415 * torque_with_ddq[2] + 179.987635
  else:
    curr[2] = 143.031415 * torque_with_ddq[2] - 179.987635
  curr[3] = 146.598336 * torque_with_ddq[3]
  curr[4] = 297.804331 * torque_with_ddq[4]
#  print(torque_with_ddq)

def set_jnt(jnt, pub):
  pub.publish(jnt)
  return True;
  
def callback(data):
  global arm_state
  arm_state = data
  check_tq()

if __name__ == '__main__':
  main()
