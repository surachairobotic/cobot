#!/usr/bin/env python

# use xyzab to interpolate

import sys
import os
import math
import numpy as np
import rospy
import my_kinematics as kinematics
import tf.transformations
import time
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint


def Rx(q):
  cx = math.cos(q)
  sx = math.sin(q)
  return np.array( [[1,0,0,0],[0, cx,-sx,0],[0,sx,cx,0],[0,0,0,1]] )


def Ry(q):
  cy = math.cos(q)
  sy = math.sin(q)
  return np.array( [[cy, 0, sy,0],[0, 1, 0,0],[-sy, 0, cy,0],[0,0,0,1]] )

def Rz(q):
  cz = math.cos(q)
  sz = math.sin(q)
  return np.array( [[cz, -sz, 0,0],[sz, cz, 0,0],[0,0,1,0],[0,0,0,1]] )
  

def quat2ab(xyz, quat, origin_xyz):
  mat = tf.transformations.quaternion_matrix(quat)
  b = 0
  
  qx = mat[0:3,0]
  qy = mat[0:3,1]
  qz = mat[0:3,2]
  
  q_xyz = np.array([ xyz[0] - origin_xyz[0], xyz[1] - origin_xyz[1], 0])
#  print('qz : ' + str(qz))
#  print('q_xyz : ' + str(q_xyz))
  
  if abs(qz[2])<0.001:
    if qz.dot( q_xyz )>0:
      v2 = np.array([0,0,1])
    else:
      v2 = np.array([0,0,-1])
    #v2 =  # q_xyz
    #print('xxxxx')
  else:
    if qz[2]<0:
      v1 = np.cross( qz, q_xyz )
    else:
      v1 = np.cross( q_xyz, qz )
    v1/= math.sqrt( v1[0]**2 +  v1[1]**2 +  v1[2]**2 )
    v2 = np.cross( v1, qz )
  
  ac = v2.dot( qy )
  if ac>1:
    ac = 1
  elif ac<-1:
    ac = -1
  b = math.acos(ac)
  sin_b = v2.dot( qx )
  if sin_b>0:
    b = -b
  b+= math.atan2( q_xyz[1], q_xyz[0] ) + math.pi*0.5
  a = math.atan2( math.sqrt(qz[0]**2+qz[1]**2), abs(qz[2]) )
#  b = math.atan2(qy[1], qy[0]) + math.pi*0.5
  
#  print(mat)
  
  '''
  print(mat)
  print('a : ' + str(a))
  print('b : ' + str(b))
  '''
  
  if qz[2]>0:
    a = math.pi - a
  if qz[0]*(xyz[0] - origin_xyz[0]) + qz[1]*(xyz[1] - origin_xyz[1]) >=0:
    a = -a
    
#  if abs(qy[0])<0.01 && abs(qy[1])<0.01:
#    b = 
    
  return [a,b]
  
def ab2quat(xyz, ab, origin_xyz):
  x = xyz[0] - origin_xyz[0]
  y = xyz[1] - origin_xyz[1]
  z = xyz[2] - origin_xyz[2]
  
  qq = math.atan2(y,x) + math.pi*0.5
  qz = -ab[1] + qq - math.pi*0.5
  qy = math.pi - ab[0]
  qz2 = math.pi*0.5 + qq
  
  mat = np.matmul( Rz(qz2) , np.matmul(Ry(qy),Rz(qz)))
  vx = np.array([1,0,0,1])
  v = mat.dot(vx)

  quat = tf.transformations.quaternion_from_matrix(mat)
  return quat

def plan(req, joint_limits, joint_names, origin_xyz):
  # use joint angles
  t_start = time.time()
  if len(req.joint_names)>0:
    if req.joint_names!=joint_names:
      raise kinematics.MyException('Joint names does not match  : {0}'.format(req.joint_names))
    '''
    for i in range(len(req.joint_names)):
      if req.joint_names[i] not in joint_names:
        raise kinematics.MyException('Unknown joint name : ' + req.joint_names[i])
    '''
    start_pose = kinematics.get_pose(req.start_joints)
    if start_pose is None:
      raise kinematics.MyException('Invalid start joints')
    end_pose = kinematics.get_pose(req.end_joints)
    if end_pose is None:
      raise kinematics.MyException('Invalid end joints')
    
  # use pose
  else:
    start_pose = req.start_pose
    end_pose = req.end_pose
  
  
      
  
  max_velo = req.max_velocity
  max_acc = req.max_acceleration
  n = len(joint_names)
  p = start_pose.position
  dis = math.sqrt( (end_pose.position.x-p.x)**2 
    + (end_pose.position.y-p.y)**2 
    + (end_pose.position.z-p.z)**2 )

  start_ab = quat2ab( kinematics.pos2list(start_pose.position)
    , kinematics.quat2list(start_pose.orientation)
    , origin_xyz)
    
  end_ab = quat2ab( kinematics.pos2list(end_pose.position)
    , kinematics.quat2list(end_pose.orientation)
    , origin_xyz)

  n_time = 0
  while n_time<100:
    velo = max_velo
    acc = max_acc
    b_replan = False
    points = []
    
#    print('velo : %f, acc : %f' % (velo, acc))
    
    # time
#    t_max = (dis + (velo**2)/acc)/velo
    t = velo/acc          # time used for acceleration
    dis_acc = (velo*0.5)*t      # distance for acceleration
    if dis_acc > dis*0.5:
      # dis*0.5 = 0.5*acc*t^2
      t = math.sqrt(dis/acc)
      t_acc = [t, t]
      velo = acc * t
      t_max = t * 2
      err = abs(dis*0.5 - 0.5*acc*t**2)
      if velo > max_velo:
        raise kinematics.MyException('velo increased : ')
      max_velo_linear = velo
    else:
#      t = velo / acc
      t_const = (dis - 2*0.5*acc*t**2) / velo
      t_max = t*2 + t_const
      t_acc = [t, t+t_const]
      err = abs(dis - acc*t**2 - velo*(t_max-2*t))
      max_velo_linear = velo
      
    if err>0.0001:
      print('dis_acc = {0}\ndis={1}\nt = {2}\nt_acc = {3}\nt_max = {4}'.format(dis_acc,dis,t,t_acc,t_max))
      raise kinematics.MyException('cal time failed : err = ' + str(err))
    
    step = int(math.ceil(t_max / req.step_time))
    step_time = t_max / step
    times = [i*step_time for i in range(step+1)]
    
    # tip angle
    '''
    q1 = kinematics.quat2list(start_pose.orientation)
    q2 = kinematics.quat2list(end_pose.orientation)
    rz1 = tf.transformations.quaternion_matrix(q1)[0:3,2]
    rz2 = tf.transformations.quaternion_matrix(q2)[0:3,2]
    ang1 = kinematics.get_tip_direction( origin_xyz, start_pose.position )
    ang2 = kinematics.get_tip_direction( origin_xyz, end_pose.position )
    ang0 = kinematics.get_diff_ang(start_pose.position, end_pose.position )
    
    tr, tp, ty = tf.transformations.euler_from_quaternion(q1)
    rpy1 = [tr,tp,ty]
    tr, tp, ty = tf.transformations.euler_from_quaternion(q2)
    rpy2 = [tr,tp,ty]
    '''
    
    dts = [0]
    for i in range(len(times)-1):
      dts.append(times[i+1]-times[i])
    for i in range(len(times)):
      p = JointTrajectoryPoint()
      p.positions = [0]*n
      p.velocities = [0]*n
      p.accelerations = [0]*n
      p.effort = [0]*n
      p.time_from_start = rospy.rostime.Duration(times[i])
      
      if times[i]<t_acc[0]:
        v = acc*times[i]
        d = 0.5 * v * times[i]
        a = acc
      elif times[i]>t_acc[1]:
        v = acc*(t_max - times[i])
        d = dis - 0.5 * v * (t_max - times[i])
        a = -acc
      else:
        v = velo
        d = 0.5*acc*t_acc[0]**2 + (times[i]-t_acc[0])*v
        a = 0
        
      ratio = d / dis
      pose = geometry_msgs.msg._Pose.Pose()
      pose.position.x = start_pose.position.x + (end_pose.position.x - start_pose.position.x)*ratio
      pose.position.y = start_pose.position.y + (end_pose.position.y - start_pose.position.y)*ratio
      pose.position.z = start_pose.position.z + (end_pose.position.z - start_pose.position.z)*ratio
      '''
      ang = kinematics.get_tip_direction( origin_xyz, pose.position )
      if ang1==ang2:
        q_ratio = 0
      else:
        q_ratio = (ang-ang1)/(ang2-ang1)
      q_ratio = kinematics.get_diff_ang( pose.position, start_pose.position ) / ang0
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
      pose.orientation = kinematics.list2quat(q)
      '''
      
      ab = []
      for j in range(2):
        ab.append( start_ab[j] + (end_ab[j] - start_ab[j]) * ratio )
      pose.orientation = kinematics.list2quat(
        ab2quat( kinematics.pos2list(pose.position), ab, origin_xyz)
      )
      p.positions = kinematics.get_joints(pose)
      for v in p.positions:
        if math.isnan(v):
          print('invalid pos : ')
          print(p.positions)
          print(pose)
          raise kinematics.MyException('invalid pos : ')
      '''
      if i<len(time)-1 and i>0:
        new_dt = 0
        p1 = points[i-1]
        dt = dts[i]
        
        new_acc = max_acc
        new_velo = max_velo_linear
        for j in range(n):
          v = (p.positions[j] - p1.positions[j]) / dt
          v = 2*v - p1.velocities[j]
          a = (p.velocities[j] - p1.velocities[j]) / dt
          a = 2*a - p1.accelerations[j]
          p.velocities[j] = v
          p.accelerations[j] = a
          a = abs(a)
          v = abs(v)
          if joint_limits[j].has_acceleration and a > joint_limits[j].acceleration:
            a = max_acc * joint_limits[j].acceleration / a
            if a < new_acc:
              new_acc = a
              #print('[%d] acc ex [%d] : %f / %f = %f' % (n_time, j, p.accelerations[j], joint_limits[j].acceleration, a))
              b_replan = True

          if joint_limits[j].has_velocity and v > joint_limits[j].velocity:
            v = max_velo_linear * joint_limits[j].velocity / v
            if v < new_velo:
              #print('[%d] vel ex [%d] : %f / %f => %f' % (n_time, j, p.velocities[j], joint_limits[j].velocity, v))
              b_replan = True
              new_velo = v

        if b_replan:
          max_velo = new_velo
          max_acc = new_acc
          break
      '''
      points.append(p)
    
    new_acc = max_acc
    new_velo = max_velo_linear
    i = 1
    while i<len(points)-1:
      p = points[i]
      new_dt = 0
      p1 = points[i-1]
      dt = dts[i]
      for j in range(n):
        v = (p.positions[j] - p1.positions[j]) / dt
#        v = 2*v - p1.velocities[j]
        a = (p.velocities[j] - p1.velocities[j]) / dt
#        a = 2*a - p1.accelerations[j]
        p.velocities[j] = v
        p.accelerations[j] = a
        a = abs(a)
        v = abs(v)
        
        if joint_limits[j].has_acceleration and a > joint_limits[j].acceleration:
          a = max_acc * joint_limits[j].acceleration / a
          if a < new_acc:
            new_acc = a
            b_replan = True
            #break
        if joint_limits[j].has_velocity and v > joint_limits[j].velocity:
          v = max_velo_linear * joint_limits[j].velocity / v
          if v < new_velo:
            new_velo = v
            b_replan = True
            #break
        
      i+=1
    
#    b_replan = False
#    print('[%d] [i=%d] vel ex : %f => %f' % (n_time, i, max_velo, new_velo))
#    print('[%d] [i=%d] acc ex : %f => %f' % (n_time, i, max_acc, new_acc))
    if b_replan:
#      print('vl : %f' % (max_velo_linear))
      max_velo = new_velo
      max_acc = new_acc
    
    if not b_replan:
      print('attempt : %d, time = %f' % (n_time, time.time()-t_start))
      print('max_velo : %f , max_acc : %f' % (max_velo, max_acc))
      return points
    else:
      n_time+=1
  raise kinematics.MyException('Line planning took too long time')

