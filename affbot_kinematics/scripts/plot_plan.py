#!/usr/bin/env python

import sys
import os
import rospy
import math
import numpy as np
import my_kinematics as kinematics

from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
import affbot_planner
'''
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from pointsectory_msgs.msg import JointpointsectoryPoint
import tf.transformations
'''
import matplotlib.pyplot as plt



def save(file_name, points):
  with open(file_name, 'wt') as f:
    for p in points:
      s = str(p.time_from_start.to_sec()) + ' '
      for v in p.positions:
        s+= str(v) + ' '
      for v in p.velocities:
        s+= str(v) + ' '
      for v in p.accelerations:
        s+= str(v) + ' '
      f.write(s + '\n')
    print('\'plan.txt\' saved.')


def plot(points):
  pos = []
  velo = []
  acc = []
  t = []
  xyz = []
  line_dis = []
  
  p1 = kinematics.get_pose(points[0].positions)
  p1 = np.array(affbot_planner.pos2list(p1.position))
  p2 = kinematics.get_pose(points[-1].positions)
  p2 = np.array(affbot_planner.pos2list(p2.position))
  l0 = (p2-p1)/np.linalg.norm(p2-p1)
  for i in range(len(points)):
    pos.append(points[i].positions[:])
    velo.append(points[i].velocities[:])
    acc.append(points[i].accelerations[:])
    t.append(points[i].time_from_start.to_sec())
    pose = kinematics.get_pose(points[i].positions)
    p = [pose.position.x ,pose.position.y, pose.position.z]
    xyz.append(p)
    p = np.array(p)
    v = np.cross(p-p1, l0)
    line_dis.append( np.linalg.norm(v) )
    
  f, axarr = plt.subplots(4, sharex=True)
  axarr[0].hold(True)
  axarr[1].hold(True)
  axarr[2].hold(True)
  
  pos = np.array(pos)
  velo = np.array(velo)
  acc = np.array(acc)
  xyz = np.array(xyz)
  for i in range(5):
    axarr[0].plot(t, pos[:,i], '+-')
    axarr[1].plot(t, velo[:,i], '+-')
    axarr[2].plot(t, acc[:,i], '+-')
#  axarr[3].plot(t, xyz[:,0], '+-', t, xyz[:,1], '+-', t, xyz[:,2], '+-')
  axarr[3].plot(t, line_dis, '+-')
  plt.show()


