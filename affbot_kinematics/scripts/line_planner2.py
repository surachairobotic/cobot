#!/usr/bin/env python

import sys
import os
import math
import numpy as np
import rospy
import my_kinematics as kinematics
import tf.transformations
import affbot_planner
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint


def quat2ab(xyz, quat, origin_xyz):
  b = math.atan2( origin_xyz[0] - xyz[0], xyz[1] - origin_xyz[1] )
  
  mat = tf.transformations.quaternion_matrix(quat)
  x1 = mat[:,2]
  x = mat[0,2]
  y = mat[1,2]
  z = mat[2,2]
  print(mat[:,2])
  a = math.atan2( math.sqrt(x**2+y**2), abs(z) )
  if z>0:
    a = math.pi - a
  if x*(xyz[0] - origin_xyz[0]) + y*(xyz[1] - origin_xyz[1]) >=0:
    a = -a
  return a,b
  
def ab2quat(xyz, ab, origin_xyz):
  x = xyz[0] - origin_xyz[0]
  y = xyz[1] - origin_xyz[1]
  z = xyz[2] - origin_xyz[2]
  
  '''
  qz = math.atan2( -x, y )
  qx = ab[1] - qz
  qy = ab[0] + math.pi*0.5
  '''
  qz = math.atan2(y,x)
  qx = ab[1] - qz
  qy = ab[0] + math.pi*0.5
  
  cx = math.cos(qx)
  sx = math.sin(qx)
  cy = math.cos(qy)
  sy = math.sin(qy)
  cz = math.cos(qz)
  sz = math.sin(qz)
  rx = np.array( [[1,0,0,0],[0, cx,-sx,0],[0,sx,cx,0],[0,0,0,1]] )
  rz = np.array( [[cz, -sz, 0,0],[sz, cz, 0,0],[0,0,1,0],[0,0,0,1]] )
  ry = np.array( [[cy, 0, sy,0],[0, 1, 0,0],[-sy, 0, cy,0],[0,0,0,1]] )
#  mat = np.matmul( rx , np.matmul(ry,rz))
  mat = np.matmul( ry , np.matmul(rz,rx))
  vx = np.array([1,0,0,1])
  v = mat.dot(vx)
  
  print(rx)
  print(ry)
  print(rz)
  
  print(v)
  return tf.transformations.quaternion_from_matrix(mat)

def plan(req, joint_limits, joint_names, origin_xyz):
  # use joint angles
  if len(req.joint_names)>0:
    if req.joint_names!=joint_names:
      raise affbot_planner.MyException('Joint names does not match  : {0}'.format(req.joint_names))
    '''
    for i in range(len(req.joint_names)):
      if req.joint_names[i] not in joint_names:
        raise affbot_planner.MyException('Unknown joint name : ' + req.joint_names[i])
    '''
    start_pose = kinematics.get_pose(req.start_joints)
    if start_pose is None:
      raise affbot_planner.MyExceptionr('Invalid start joints')
    end_pose = kinematics.get_pose(req.end_joints)
    if end_pose is None:
      raise affbot_planner.MyExceptionr('Invalid end joints')
    
  # use pose
  else:
    start_pose = req.start_pose
    end_pose = req.end_pose
      
      
  max_velo = req.max_velocity
  max_acc = req.max_acceleration
  n = len(joint_names)
  p = start_pose.position
  dis = math.sqrt( (end_pose.x-p.x)**2 + (end_pose.y-p.y)**2 + (end_pose.z-p.z)**2 )
  #dis = math.sqrt( p.x**2 + p.y**2 + p.z**2 )
  n_time = 0
  while n_time<10:
    velo = max_velo
    acc = max_acc
    b_replan = False
    points = []
    
    
    # time
#    t_max = (dis + (velo**2)/acc)/velo
    t = velo/acc          # time used for acceleration
    s = (velo*0.5)*t      # distance for acceleration
    if s<dis*0.5:
      # dis*0.5 = 0.5*acc*t^2
      t = math.sqrt(dis/acc)
      t_acc = [t, t]
      velo = acc * t
      t_max = t * 2
      err = abs(dis*0.5 - 0.5*acc*t**2)
    else:
      t = velo / acc
      t_const = (dis - 2*0.5*acc*t**2) / velo
      t_max = t*2 + t_const
      t_acc = [t, t+t_const]
      err = abs(dis - 0.5*acc*t**2 - velo*(t_max-2*t))
      
    if err>0.0001:
      raise affbot_planner.MyException('cal time failed : err = ' + str(err))
    
    step = int(math.ceil(t_max / req.step_time))
    step_time = t_max / step
    time = [i*step_time for i in range(step+1)]
    
    # tip angle
    q1 = affbot_planner.quat2list(start_pose.orientation)
    q2 = affbot_planner.quat2list(end_pose.orientation)
    rz1 = tf.transformations.quaternion_matrix(q1)[0:3,2]
    rz2 = tf.transformations.quaternion_matrix(q2)[0:3,2]
    ang1 = affbot_planner.get_tip_direction( origin_xyz, start_pose.position )
    ang2 = affbot_planner.get_tip_direction( origin_xyz, end_pose.position )
    ang0 = affbot_planner.get_diff_ang(start_pose.position, end_pose.position )
    
    tr, tp, ty = tf.transformations.euler_from_quaternion(q1)
    rpy1 = [tr,tp,ty]
    tr, tp, ty = tf.transformations.euler_from_quaternion(q2)
    rpy2 = [tr,tp,ty]
    
    dts = [0]
    for i in range(len(time)-1):
      dts.append(time[i+1]-time[i])
    for i in range(len(time)):
      p = JointTrajectoryPoint()
      p.positions = [0]*n
      p.velocities = [0]*n
      p.accelerations = [0]*n
      p.effort = [0]*n
      p.time_from_start = rospy.rostime.Duration(time[i])
      
      if time[i]<t_acc[0]:
        v = acc*time[i]
        d = 0.5 * v * time[i]
        a = acc
      elif time[i]>t_acc[1]:
        v = acc*(t_max - time[i])
        d = dis - 0.5 * v * (t_max - time[i])
        a = -acc
      else:
        v = velo
        d = 0.5*acc*t_acc[0]**2 + (time[i]-t_acc[0])*v
        a = 0
        
      ratio = d / dis
      pose = geometry_msgs.msg._Pose.Pose()
      pose.position.x = start_pose.position.x + (end_pose.position.x - start_pose.position.x)*ratio
      pose.position.y = start_pose.position.y + (end_pose.position.y - start_pose.position.y)*ratio
      pose.position.z = start_pose.position.z + (end_pose.position.z - start_pose.position.z)*ratio
      
      ang = affbot_planner.get_tip_direction( origin_xyz, pose.position )
      if ang1==ang2:
        q_ratio = 0
      else:
        q_ratio = (ang-ang1)/(ang2-ang1)
      q_ratio = affbot_planner.get_diff_ang( pose.position, start_pose.position ) / ang0
      '''
      rpy = [0,0,0]
      for k in range(3):
        rpy[k] = rpy1[k] + (rpy2[k] - rpy1[k])*q_ratio
      q = tf.transformations.quaternion_from_euler( rpy[0], rpy[1], rpy[2] )
      '''
      q = tf.transformations.quaternion_slerp( q1, q2, q_ratio)
      link_pos = [pose.position.x - origin_xyz[0],pose.position.y - origin_xyz[1],pose.position.z - origin_xyz[2]]
      mat = tf.transformations.quaternion_matrix(q)
      tip_pos = mat[0:3,2]
  #      print('q_ratio : ' + str(q_ratio))
  #      print('ratio : ' + str(ratio))
  #      print('link : ' + str(math.atan2(link_pos[1], link_pos[0])))
  #      print('tip  : ' + str(math.atan2(tip_pos[1], tip_pos[0])))
  #      exit()
  #      continue
      pose.orientation = affbot_planner.list2quat(q)
      p.positions = kinematics.get_joints(pose)
      
      if i<len(time)-1 and i>0:
        new_dt = 0
        p1 = points[i-1]
        dt = dts[i]
        for j in range(n):
          v = (p.positions[j] - p1.positions[j]) / dt
          p.velocities[j] = 2*v - p1.velocities[j]
          p.accelerations[j] = (p.velocities[j] - p1.velocities[j]) / dt
          if joint_limits[j].has_acceleration and abs(p.accelerations[j]) > joint_limits[j].acceleration:
            max_acc *= joint_limits[j].acceleration / abs(p.accelerations[j])
            b_replan = True
            break
          if joint_limits[j].has_velocity and abs(p.velocities[j]) > joint_limits[j].velocity:
            max_velo *= joint_limits[j].velocity / abs(p.velocities[j])
            b_replan = True
            break
        if b_replan:
          break
      points.append(p)
    if not b_replan:
      return points
    else:
      n_time+=1
  raise affbot_planner.MyException('Line planning took too long time')

