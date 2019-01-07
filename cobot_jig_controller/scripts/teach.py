#!/usr/bin/env python

import rospy
import time

from sensor_msgs.msg import JointState
from dynamic import *

arm_state = JointState()
stat = JointState()
pub_status = rospy.Publisher("/cobot/status", JointState, queue_size=10)
ABC = []
curr = []

def main():
  rospy.init_node('teach', anonymous=True)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  sub = rospy.Subscriber("/joint_states", JointState, callback)

  global ABC, curr
  rate = rospy.Rate(50)

  ABC = [ [ 355.0, 1240.0,  245.0],
          [-177.0, -690.0, -650.0],
          [ 180.0,  564.0,  206.0],
          [ 256.0,  689.0,  232.0],
          [ 241.0,   22.0,   58.0],
          [   0.0,    0.0,    0.0] ]
  
  jntHome = JointState()
  jntHome.name = ['J1','J2','J3','J4','J5','J6']
  jntHome.position = [0.0,0.0,0.0,0.0,0.0,0.0]
  jntHome.velocity = [0.25,0.25,0.25,0.25,0.25,0.25]
  jntHome.effort = []
  
  t_start = time.time()
#  while not rospy.is_shutdown():
  while (time.time()-t_start) < 5.0 and not rospy.is_shutdown():
    set_jnt(jntHome, pub)
    rate.sleep()
#  sub.unregister()
  rospy.loginfo("step 1 : OK")

  t_start = time.time()
  while not rospy.is_shutdown():
    a=0
    rate.sleep()
  sub.unregister()
  rospy.loginfo("step 11 : OK")

  jntSend = JointState()
  jntSend.name = ['J1','J2','J3','J4','J5','J6']
  jntSend.position = []
  jntSend.velocity = []
#  jntSend.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  jntSend.effort = curr
  t_start = time.time()
  while (time.time()-t_start) < 5.0 and not rospy.is_shutdown():
    set_jnt(jntSend, pub)
    rate.sleep()
  rospy.loginfo("step 2 : OK")

  pub.unregister()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

def check_tq():#(t_from_start):
  global arm_state, ABC, stat, pub_status, curr

  pos = np.array(arm_state.position)
  vel = np.zeros(pos.shape)
  acc = np.zeros(pos.shape)

  vars = eq.get_vars(arm_state.position) # load equation parameters for calculating torque
  torque_with_ddq, torque_no_ddq = cal_torque(pos, vel, acc, vars)

  curr = []
  for i in range(6):
    curr.append(torque2current(torque_with_ddq[i], vel[i], ABC[i]))

  stat.position = curr
  stat.velocity = arm_state.effort
  stat.header.stamp = arm_state.header.stamp
  stat.effort = []
  pub_status.publish(stat)

def set_jnt(jnt, pub):
  pub.publish(jnt)
  return True;
  
def callback(data):
  global arm_state
  arm_state = data
  check_tq()

if __name__ == '__main__':
  main()
