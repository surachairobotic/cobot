#!/usr/bin/env python

import sys
import os
import rospy
import math
import my_kinematics as kinematics
import numpy as np
import random
import lib_controller
import line_planner2


if __name__ == "__main__":
  rospy.init_node('test_kinematics')
  kinematics.init()
  
  
  joints = [None]*5
  origin_xyz = kinematics.get_origin_xyz()
  lib_controller.JOINT_MIN_ANG[4] = -math.pi
  lib_controller.JOINT_MAX_ANG[4] =  math.pi
  lib_controller.JOINT_MIN_ANG[1] = -10*math.pi/180.0
  lib_controller.JOINT_MAX_ANG[1] =  45*math.pi/180.0
  lib_controller.JOINT_MIN_ANG[2] = -30*math.pi/180.0
  lib_controller.JOINT_MAX_ANG[2] =  30*math.pi/180.0
  for i in range(10000):
    for j in range(len(joints)):
      joints[j] = random.uniform( lib_controller.JOINT_MIN_ANG[j], lib_controller.JOINT_MAX_ANG[j])
    pose = kinematics.get_pose(joints)
    j2 = kinematics.get_joints(pose)
    xyz = kinematics.pos2list(pose.position)
    quat = kinematics.quat2list(pose.orientation)
    ab = line_planner2.quat2ab( xyz, quat, origin_xyz )
    quat2 = line_planner2.ab2quat( xyz, ab, origin_xyz )
      
    err_j = np.sum( np.array(joints) - np.array(j2) )
    err_q = np.sum( np.array(quat) - np.array(quat2) )
    
    if err_j>0.001:
      print(np.array(joints)*180.0/math.pi)
      print(np.array(j2)*180.0/math.pi)
      print(err_j)
      exit()
      
    if err_q>0.001:
      print(quat)
      print(quat2)
      print(err_q)
      exit()
      

    
  
