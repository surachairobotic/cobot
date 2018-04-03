#!/usr/bin/env python

import sys
import os
import math
import numpy as np
import rospy
import tf.transformations
import line_planner2
import my_kinematics as kinematics
import random


if __name__ == "__main__":

  rospy.init_node('test_quat_ab')
  kinematics.init()
  origin_xyz = kinematics.get_origin_xyz()
  
  lim = [
    [-1.571,1.571]
   ,[-0.269,1.571]
   ,[-1.571,1.05]
   ,[-3.50,0.5236]
   ,[-3.50,3.5236]
  ]
  random.seed()
  for i in range(10000):
    j = []
    for i in range(len(lim)):
      j.append( random.uniform(lim[i][0],lim[i][1] ) )
    #j = [0*0.7201157320713183, 1.4435302438139055, -0.13005808701056165, -2.884811556826161, 0.5498283222939966]
    #j = [0,1.44,-0.13,-2.88 + math.pi,0.5]
    pose = kinematics.get_pose(j) # [0,0,0,0.9,3.2])

    xyz = [pose.position.x, pose.position.y, pose.position.z]
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    
    if xyz[1] - origin_xyz[1] >= 0.01:
      continue
    
    ab = line_planner2.quat2ab( xyz, quat, origin_xyz )
    quat2 = line_planner2.ab2quat( xyz, ab, origin_xyz )
    
    for k in range(len(quat)):
      if abs(quat[k] - quat2[k])>0.00001:# or 1:
        print('joints: ' + str(j))
        
        print('xyz : ' + str(np.array(xyz) - np.array(kinematics.get_origin_xyz())))
        print('ab : ' + str(ab))
        print('quat: ' + str(quat))
        print('quat2: ' + str(quat2))
        print('dquat : ' + str(np.array(quat) - np.array(quat2)))
        exit()
    #exit()
