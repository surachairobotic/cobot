#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time
from sensor_msgs.msg import JointState

def callback_return(msg):
  rospy.loginfo("recv : position %lf", msg.position[0])

if __name__ == '__main__':
  rospy.init_node('tester2_python', anonymous=True)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  sub = rospy.Subscriber("/cobot_dynamixel_driver/joint_states_return", JointState, callback_return)

  jnt = JointState()
  jnt.name = []
  jnt.position = []
  jnt.velocity = []
  jnt.effort = []

  rate = rospy.Rate(100)
#  jnt.position.resize(1)
  i = 0
  while True:
    rospy.loginfo("i = %d", i)
    jnt.position = [i]
    i = i+1
    pub.publish(jnt)
#    ros::spinOnce();
    rate.sleep()
    if i>=100:
      break

  sub.unregister()
  pub.unregister()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
  
