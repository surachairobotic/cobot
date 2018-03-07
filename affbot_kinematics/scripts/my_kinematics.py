#!/usr/bin/env python

import sys
import os
import rospy
import math

import geometry_msgs.msg
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

joint_names = ['J1', 'J2', 'J3', 'J4', 'J5']

compute_fk = None
compute_ik = None


def get_pose(ang):
  global JOINT_NAME, compute_fk
  header = Header()
  header.frame_id = 'base_link'
  robot_state = RobotState()
  robot_state.joint_state.name = joint_names
  robot_state.joint_state.position = ang
  c = compute_fk(header, ['tool0'], robot_state)
  if c.error_code.val!=1:
    raise Exception('compute_fk error : error_code = ' + str(c.error_code.val))
   # rospy.logerr('compute_fk error : error_code = ' + str(c.error_code.val))
   # return None
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
      raise Exception('error : ' + str(resp.error_code.val))
      #print('error : ' + str(resp.error_code.val))
    return resp.solution.joint_state.position
  except rospy.ServiceException, e:
    raise Exception("Service call failed: %s"%e)
    #print "Service call failed: %s"%e
    #return None

def init():
  global compute_ik, compute_fk
  
  print("waiting 'compute_fk'")
  rospy.wait_for_service('compute_fk')
  compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
  print("waiting 'compute_ik'")
  rospy.wait_for_service('compute_ik')
  compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

