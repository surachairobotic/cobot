#!/usr/bin/env python

import sys
import os
import math
import numpy as np
import rospy
import my_kinematics as kinematics
import tf.transformations
#import affbot_planner
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint


def plan(req, joint_limits, joint_names):
  # use joint angles
  if len(req.joint_names)>0:
    if req.joint_names!=joint_names:
      raise kinematics.MyException('Joint names does not match  : {0}'.format(req.joint_names))
      
    for i in range(len(req.joint_names)):
      if req.joint_names[i] not in joint_names:
        raise kinematics.MyException('Unknown joint name : ' + req.joint_names[i])
    start_joints = req.start_joints
    end_joints = req.end_joints
  # use pose
  else:
    start_joints = kinematics.get_joints(req.start_pose)
    if start_joints is None:
      raise kinematics.MyExceptionr('Invalid start pos')
    end_joints = kinematics.get_joints(req.end_pose)
    if end_joints is None:
      raise kinematics.MyException('Invalid end pos')
  res_points = []
  n = len(joint_names)
  pos = [None]*n
  velo = [None]*n
  acc = [None]*n
  time = [None]*n
  direct = [None]*n
  for i in range(n):
    lim = joint_limits[i]
    if lim.has_velocity and lim.velocity<req.max_velocity:
      velo[i] = lim.velocity
    else:
      velo[i] = req.max_velocity
    if lim.has_acceleration and lim.acceleration<req.max_acceleration:
      acc[i] = lim.acceleration
    else:
      acc[i] = req.max_acceleration
    
    pos[i] = abs(start_joints[i] - end_joints[i])
    time[i] = (pos[i] + (velo[i]**2)/acc[i])/velo[i]
    ta = velo[i]/acc[i]
    if time[i]<ta*2:
      time[i] = math.sqrt(pos[i]*4.0/acc[i])
      velo[i] = acc[i]*time[i]*0.5
      err = abs(pos[i] - 0.25*acc[i]*time[i]**2)
    else:
      err = abs(pos[i] - 0.5*ta*velo[i]*2 - velo[i]*(time[i]-2*ta))
    if err>0.0001:
      raise kinematics.MyException('cal time failed : err = ' + str(err))
    if start_joints[i]<end_joints[i]:
      direct[i] = 1
    else:
      direct[i] = -1

  # find the joint that use the longest time
  index_max_time = max(xrange(len(time)), key=time.__getitem__)
  t_max = time[index_max_time]
  
  # start_pose = end_pose
  if t_max<0.1:
    p = JointTrajectoryPoint()
    p.positions = start_joints
    p.velocities = [0]*n
    p.accelerations = [0]*n
    p.effort = [0]*n
    res_points.append(p)
    p = JointTrajectoryPoint()
    p.positions = end_joints
    p.velocities = [0]*n
    p.accelerations = [0]*n
    p.effort = [0]*n
    res_points.append(p)
#    check_plan(res_points, joint_limits)
    return res_points
  
  # make all joints end at the same time
  for i in range(n):
    if i==index_max_time:
      continue
    # v = (t+math.sqrt(t**2-4/a*th))/(2/a))
    
    dd = t_max**2-4*pos[i]/acc[i]
    if dd<0:
      dd = 0
#      raise kinematics.MyException('cannot cal velo for axis [%d]\nt_max : %f\npos = %f\nacc = %f\ndd = %f\n' % (i, t_max, pos[i], acc[i], dd))
      
    velo[i] = (t_max-math.sqrt(dd)) * acc[i] * 0.5
    err = abs(velo[i]**2/acc[i] - velo[i]*t_max + pos[i])
    if err>0.0001:
      raise kinematics.MyException('recal velo failed : err = ' + str(err))
  
  step = int(math.ceil(t_max / req.step_time))
  step_time = t_max / step
  time = [i*step_time for i in range(step+1)]

  t_acc = []
  for i in range(n):
    t2 = velo[i]/acc[i]
    if t2>t_max*0.5:
      t_acc.append([t_max*0.5,t_max*0.5])
    else:
      t_acc.append([t2, t_max-t2])
    velo[i]*= direct[i]
    acc[i]*= direct[i]
  
  for i in range(len(time)):
    p = JointTrajectoryPoint()
    p.positions = [0]*n
    p.velocities = [0]*n
    p.accelerations = [0]*n
    p.effort = [0]*n
    for j in range(n):
      if time[i]<t_acc[j][0]:
        p.velocities[j] = acc[j]*time[i]
        p.positions[j] = start_joints[j] + 0.5 * p.velocities[j] * time[i]
        p.accelerations[j] = acc[j]
      elif time[i]>t_acc[j][1]:
        p.velocities[j] = acc[j]*(t_max - time[i])
#          p.positions[j] = start_joints[j] + 0.5*acc[j]*t_acc[j][0]**2 \
#            + velo[j]*(t_acc[j][1]-t_acc[j][0]) \
#            + (velo[j] + velo[j]-acc[j]*(time[i]-t_acc[j][1]))*0.5*(time[i]-t_acc[j][1])
        p.positions[j] = end_joints[j] - 0.5 * p.velocities[j] * (t_max - time[i])
        p.accelerations[j] = -acc[j]
      else:
        p.velocities[j] = velo[j]
        p.positions[j] = start_joints[j] + 0.5*acc[j]*t_acc[j][0]**2 + (time[i]-t_acc[j][0])*velo[j]
        p.accelerations[j] = 0
    
    p.time_from_start = rospy.rostime.Duration(time[i])
    res_points.append(p)
  return res_points

