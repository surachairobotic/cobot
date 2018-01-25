#!/usr/bin/env python

import sys
import os
import rospy
import math
from affbot_kinematics.srv import *
from geometry_msgs.msg import Pose
from affbot_kinematics.msg import *

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState



compute_fk = None
compute_ik = None
srv_planning = None
JOINT_NAME = ['J1', 'J2', 'J3', 'J4', 'J5']

def get_pose(ang):
  global JOINT_NAME, compute_fk
  header = Header()
  header.frame_id = 'base_link'
  robot_state = RobotState()
  robot_state.joint_state.name = JOINT_NAME
  robot_state.joint_state.position = ang
  c = compute_fk(header, ['tool0'], robot_state)
  if c.error_code.val!=1:
    raise 'compute_fk error : error_code = ' + str(c.error_code.val)
  return c.pose_stamped[0].pose

def get_joints(xyz, xyzw=None):
  global compute_ik
  target = geometry_msgs.msg.PoseStamped()
  target.header.frame_id = 'world'
  if type(xyz) is list:
    target.pose.position.x = xyz[0]
    target.pose.position.y = xyz[1]
    target.pose.position.z = xyz[2]
    target.pose.orientation.x = xyzw[0]
    target.pose.orientation.y = xyzw[1]
    target.pose.orientation.z = xyzw[2]
    target.pose.orientation.w = xyzw[3]
  elif type(xyz) is geometry_msgs.msg._Pose.Pose:
    target.pose = xyz

  
  ik_request = moveit_msgs.msg.PositionIKRequest()
  ik_request.group_name = 'arm'
  ik_request.ik_link_name = 'tool0'
  ik_request.pose_stamped = target
  ik_request.timeout.secs = 0.1
  ik_request.avoid_collisions = False
  try:
    resp = compute_ik(ik_request = ik_request)
    if resp.error_code.val!=1:
      print('error : ' + str(resp.error_code.val))
    return resp.solution.joint_state.position
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e 


def planning(start_pose, end_pose):
  global srv_planning
  try:
    req = AffbotPlanRequest()
    joint_names = JOINT_NAME
    start_pose = start_pose
    end_pose = end_pose
    type = "p2p"
    max_velocity = 1
    res = srv_planning(JOINT_NAME, [0,0,0,0,0], [0.2,0.3,0.7,0.2,0.1]
      , start_pose, end_pose, "p2p", 1)
    return res
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


if __name__ == "__main__":
  print("waiting 'affbot_planning'")
  rospy.wait_for_service('affbot_planning')
  srv_planning = rospy.ServiceProxy('affbot_planning', AffbotPlanning)
  print("waiting 'compute_fk'")
  rospy.wait_for_service('compute_fk')
  compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
  print("waiting 'compute_ik'")
  rospy.wait_for_service('compute_ik')
  compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
  print('start')
  start_pose = get_pose([0,0,0,0,0])
  end_pose = get_pose([0.2,0.3,0.7,0.2,0.1])
  
  res = planning(start_pose, end_pose)
  print(res)
