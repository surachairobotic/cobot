#!/usr/bin/env python

## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import random
import math
import tf
import numpy as np
from pprint import pprint
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import OrientationConstraint
from sensor_msgs.msg import JointState
#from test_dynamixel.msg import Goal
from time import sleep
from tf.transformations import *
#euler_from_quaternion, quaternion_from_euler, quaternion_multiply

import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String


compute_fk = None
compute_ik = None
JOINT_NAME = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
#JOINT_NAME = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


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
  if type(xyz) is list or type(xyz) is numpy.ndarray:
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

if __name__=='__main__':
  try:
    rospy.init_node('test_kinematics',
                    anonymous=True)
    rospy.wait_for_service('compute_fk')
    compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    #xyz  = [pose.position.x, pose.position.y, pose.position.z]
    #xyzw = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    xyz1  = np.array([0.025311, 0.200823, 0.140278])
    xyzw1 = [0.022495, -0.999138, 0.017588, 0.030124]
    #
    # 0.303, -1.267, 1.282, 0.302, -0.999, 2.525

    #xyz0  = np.array([0.28184637474657914, 0.061241947945060118, 0.060507878329037673])
    #xyzw0 = [-0.60082954411511069, -0.76567319730943562, -0.19786849378194782, 0.11660391506650959]
    #js = get_joints(xyz0, xyzw0)
    #print('JS')
    #print(np.array(js)*180/3.14)
    #print(np.array(js))

    js = get_joints(xyz1, xyzw1)
    # print('JS')
    # print(np.array(js)*180/3.14)
    # print(np.array(js))

    js = [0.303, -1.267, 1.282, 0.302, -0.999, 2.525]
    pose = get_pose(js)
    print('PS')
    print(pose)

    #print( np.linalg.norm(xyz0-xyz1))

  except rospy.ROSInterruptException:
    pass
