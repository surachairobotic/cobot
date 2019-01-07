#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState
from cobot_dynamixel_driver.srv import *

def callback_return(msg):
  rospy.loginfo("recv : position %lf", msg.position[0])

if __name__ == '__main__':
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=1000)
  rospy.init_node('tester2_python', anonymous=True)
#  sub = rospy.Subscriber("/cobot_dynamixel_driver/joint_states_return", JointState, callback_return)

  print('A')
#  rospy.wait_for_service('cobot_dynamixel_driver/set_acc')  
  rospy.wait_for_service('cobot_dynamixel_driver/up_one')
  try:
    up = rospy.ServiceProxy('cobot_dynamixel_driver/up_one', up_one)
    resp1 = up(5)
    print('out = %d' % resp1.out)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  print('B')

  jnt = JointState()
  jnt.name = []
  jnt.position = []
  jnt.velocity = []
  jnt.effort = []

  rate = rospy.Rate(20)
  i = 0
  while True:
    rospy.loginfo("i = %d", i)
    jnt.header.frame_id = str(i)
    i = i+1
    pub.publish(jnt)
    rate.sleep()
    if i>=10:
      break
  
  rate = rospy.Rate(1)

#  for i in range():
  rate.sleep()
  
#  jnt.position.resize(1)
  rate = rospy.Rate(20)
  i = 10
  while True:
    rospy.loginfo("i = %d", i)
    jnt.header.frame_id = str(i)
    i = i+1
    pub.publish(jnt)
    rate.sleep()
    if i>20:
      break

  rate = rospy.Rate(1)
  for i in range(5):
    rate.sleep()

  rate = rospy.Rate(20)
  i = 100
  while True:
    rospy.loginfo("i = %d", i)
    jnt.header.frame_id = str(i)
    i = i+1
    pub.publish(jnt)
    rate.sleep()
    if i>120:
      break

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  pub.unregister()
  
