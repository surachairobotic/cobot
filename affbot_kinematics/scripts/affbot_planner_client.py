#!/usr/bin/env python

import sys
import os
import rospy
import math
import my_kinematics as kinematics
import plot_plan
from affbot_kinematics.srv import *
from geometry_msgs.msg import Pose
from affbot_kinematics.msg import *

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


srv_planning = None
joint_names = kinematics.joint_names


def planning(start_pose, end_pose):
  global srv_planning, joint_names
  try:
    req = AffbotPlanRequest()
    start_pose = start_pose
    end_pose = end_pose
    type = "p2p"
    max_velocity = 1

    res = srv_planning(joint_names=joint_names
    , start_joints=[0,0,0,0,0]
    , end_joints=[0.2,0.3,0.7,0.2,0.1]
    , start_pose=start_pose
    , end_pose=end_pose
    , type="line"
    , max_velocity=3
    , max_acceleration=3
    , step_time=0.1)
    return res
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


if __name__ == "__main__":
  print("waiting 'affbot_planning'")
  rospy.wait_for_service('affbot_planning')
  srv_planning = rospy.ServiceProxy('affbot_planning', AffbotPlanning)
  kinematics.init()
  
  print('start')
  start_pose =   kinematics.get_pose([0,0,0,0,0])
  end_pose = kinematics.get_pose([0.2,0.3,0.7,0.2,0.1])
  
  res = planning(start_pose, end_pose)
  if res.error_code==0:
    plot_plan.plot(res.points)
  else:
    print('error : '+str(res.error_code))
    
  
