#!/usr/bin/env python

import sys
import os
import rospy
import math
import numpy as np
import my_kinematics as kinematics

#from AffbotPlanningRequest.msg import *
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import tf.transformations
import matplotlib.pyplot as plt
import line_planner

robot_desc = None

joint_names = kinematics.joint_names
joint_limits = []

def quat2list(quat):
  return [ quat.x, quat.y, quat.z, quat.w ]

def list2quat(l):
  q = geometry_msgs.msg._Quaternion.Quaternion()
  q.x = l[0]
  q.y = l[1]
  q.z = l[2]
  q.w = l[3]
  return q

def pos2list(pos):
  return [pos.x, pos.y, pos.z]
  
def list2pos(l):
  p = geometry_msgs.msg._Position.Position()
  p.x = l[0]
  p.y = l[1]
  p.z = l[2]
  return p


def get_tip_direction(origin_xyz, xyz):
  x = xyz.x - origin_xyz[0]
  y = xyz.y - origin_xyz[1]
  return math.atan2(y,x)

def get_diff_ang(pos1, pos2):
  p1 = np.array([pos1.x, pos1.y, pos1.z])
  p2 = np.array([pos2.x, pos2.y, pos2.z])

  return math.acos( np.dot( p1, p2 ) / (np.linalg.norm(p1) * np.linalg.norm(p2)) )

def myplot(traj):
  pos = []
  velo = []
  acc = []
  t = []
  xyz = []
  line_dis = []
  
  p1 = kinematics.get_pose(traj[0].positions)
  p1 = np.array(pos2list(p1.position))
  p2 = kinematics.get_pose(traj[-1].positions)
  p2 = np.array(pos2list(p2.position))
  l0 = (p2-p1)/np.linalg.norm(p2-p1)
  for i in range(len(traj)):
    pos.append(traj[i].positions[:])
    velo.append(traj[i].velocities[:])
    acc.append(traj[i].accelerations[:])
    t.append(traj[i].time_from_start.to_sec())
    pose = kinematics.get_pose(traj[i].positions)
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


class cJointLimit:
  def __init__(self):
    self.position = [0,0]
    self.velocity = 0
    self.acceleration = 0

    self.has_position = False
    self.has_velocity = False
    self.has_acceleration = False
    
  def __str__(self):
    s = ' position : '
    if self.has_position:
      s+= str(self.position[0]) + ' , ' + str(self.position[1])
    else:
      s+= 'None'
    s+= '\n velocity : '
    if self.has_velocity:
      s+= str(self.velocity)
    else:
      s+= 'None'
    s+= '\n acceleration : '
    if self.has_acceleration:
      s+= str(self.acceleration)
    else:
      s+= 'None'
    return s+'\n'
  def __repr__(self):
    return self.__str__()





def handle_planning(req):
  global robot_desc, joint_limits, joint_names
  res = AffbotPlanResponse()
  # use joint angles
  if len(req.joint_names)>0:
    if req.joint_names!=joint_names:
      rospy.logerr('Joint names does not match  : {0}'.format(req.joint_names))
      res.error_code = -1
      return res
      
    for i in range(len(req.joint_names)):
      if req.joint_names[i] not in joint_names:
        rospy.logerr('Unknown joint name : ' + req.joint_names[i])
        res.error_code = -1
        return res
    start_joints = req.start_joints
    end_joints = req.end_joints
  # use pose
  else:
    start_joints = kinematics.get_joints(req.start_pose)
    if start_joints is None:
      rospy.logerr('Invalid start pos')
      res.error_code = -1
      return res
    end_joints = kinematics.get_joints(req.end_pose)
    if end_joints is None:
      rospy.logerr('Invalid end pos')
      res.error_code = -1
      return res
  
  n = len(joint_names)
  if req.type=='p2p':
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
        rospy.logerr('cal time failed : err = ' + str(err))
        res.error_code = -1
        return res
      if start_joints[i]<end_joints[i]:
        direct[i] = 1
      else:
        direct[i] = -1

    # find the joint that use the longest time
    index_max_time = max(xrange(len(time)), key=time.__getitem__)
    t_max = time[index_max_time]
    
    # start_pose = end_pose
    if t_max==0:
      p = JointTrajectoryPoint()
      p.positions = start_joints
      p.velocities = [0]*n
      p.accelerations = [0]*n
      p.effort = [0]*n
      res.points.append(p)
      p = JointTrajectoryPoint()
      p.positions = end_joints
      p.velocities = [0]*n
      p.accelerations = [0]*n
      p.effort = [0]*n
      res.points.append(p)
      return res
    
    # make all joints end at the same time
    for i in range(n):
      if i==index_max_time:
        continue
      # v = (t+math.sqrt(t**2-4/a*th))/(2/a))
      velo[i] = (t_max-math.sqrt(t_max**2-4*pos[i]/acc[i])) * acc[i] * 0.5
      err = abs(velo[i]**2/acc[i] - velo[i]*t_max + pos[i])
      if err>0.0001:
        rospy.logerr('recal velo failed : err = ' + str(err))
        res.error_code = -1
        return res
    
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
      res.points.append(p)
    
  elif req.type=='line':
    res.points = line_planner.plan_line(req, joint_limits, joint_names, origin_xyz)
    '''
    velo = req.max_velocity
    acc = req.max_acceleration
    p = req.start_pose.position
    dis = math.sqrt( p.x**2 + p.y**2 + p.z**2 )
    
    t_max = (dis + (velo**2)/acc)/velo
    t_acc = velo/acc
    if t_max<t_acc*2:
      t_max = math.sqrt(dis*4.0/acc)
      velo = acc*t_max*0.5
      t_acc = [t_max*0.5, t_max*0.5]
      err = abs(dis - 0.25*acc*t_max**2)
    else:
      t_acc = [t_acc, t_max - t_acc]
      err = abs(dis - 0.5*t_acc[0]*velo*2 - velo*(t_max-2*t_acc[0]))
    if err>0.0001:
      rospy.logerr('cal time failed : err = ' + str(err))
      res.error_code = -1
      return res
    
    step = int(math.ceil(t_max / req.step_time))
    step_time = t_max / step
    time = [i*step_time for i in range(step+1)]
    
    q1 = quat2list(req.start_pose.orientation)
    q2 = quat2list(req.end_pose.orientation)
#    print('q1 : '+str(tf.transformations.quaternion_matrix(q1)[0:3,2] ))
#    print('q2 : '+str(tf.transformations.quaternion_matrix(q2)[0:3,2] ))
    rz1 = tf.transformations.quaternion_matrix(q1)[0:3,2]
    rz2 = tf.transformations.quaternion_matrix(q2)[0:3,2]
    ang1 = get_tip_direction( req.start_pose.position )
    ang2 = get_tip_direction( req.end_pose.position )
    ang0 = get_diff_ang(req.start_pose.position, req.end_pose.position )
    
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
      pose.position.x = req.start_pose.position.x + (req.end_pose.position.x - req.start_pose.position.x)*ratio
      pose.position.y = req.start_pose.position.y + (req.end_pose.position.y - req.start_pose.position.y)*ratio
      pose.position.z = req.start_pose.position.z + (req.end_pose.position.z - req.start_pose.position.z)*ratio
      
      
      ang = get_tip_direction( pose.position )
      if ang1==ang2:
        q_ratio = 0
      else:
        q_ratio = (ang-ang1)/(ang2-ang1)
      q_ratio = get_diff_ang( pose.position, req.start_pose.position ) / ang0
      
      
      rpy = [0,0,0]
      for k in range(3):
        rpy[k] = rpy1[k] + (rpy2[k] - rpy1[k])*q_ratio
      q = tf.transformations.quaternion_from_euler( rpy[0], rpy[1], rpy[2] )
      
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
      pose.orientation = list2quat(q)
      p.positions = kinematics.get_joints(pose)
#      print('i = ' + str(i))
      
      if i<len(time)-1 and i>0:
        new_dt = 0
        p1 = res.points[i-1]
        dt = dts[i]
        for j in range(n):
          v = (p.positions[j] - p1.positions[j]) / dt
          p.velocities[j] = 2*v - p1.velocities[j]
          p.accelerations[j] = (p.velocities[j] - p1.velocities[j]) / dt
          
          if joint_limits[j].has_acceleration and abs(p.accelerations[j]) > joint_limits[j].acceleration:
            if p.accelerations[j]>0:
              a = joint_limits[j].acceleration
            else:
              a = -joint_limits[j].acceleration
            s = p.positions[j] - p1.positions[j]
            # s = v*t + 0.5*a*t^2
            # 0.5*a*t^2 + vt - s = 0
            # t = (-v+sqrt(v^2 + 4*0.5*a*s))/(2*0.5*a)
            
            if abs(a)>30:
              print(a)
              raise 'xx'
            rt = v**2 + 2*a*s
            if rt<0:
              a = (-v**2)/(2*s)
            if abs(a)>30:
              print(a)
              raise 'xx2'
              
            dt = (-v + math.sqrt(v**2 + 2*a*s))/a
            err = abs( s - v*dt - 0.5*a*dt**2 )
            if err>0.0001:
              rospy.logerr('cal new_dt failed : err = ' + str(err))
              res.error_code = -1
              return res
            if dt<dts[i]:
              print('old a : ' + str(p.accelerations[j]))
              print('new a : ' + str(a))
              raise 'small dt'
            if dt>new_dt:
              new_dt = dt
        print('new_dt : ' + str(new_dt))
        if new_dt>0:
          for j in range(n):
            s = p.positions[j] - p1.positions[j]
            v = p1.velocities[j]
            a = (s - v*new_dt) / (0.5*new_dt**2)
            if abs(a)>30:
              print('s : '+str(s))
              print('v : '+str(v))
              print('a : '+str(a))
              print('dt : '+str(new_dt))
              raise 'xx3'
            err = abs( s - v*new_dt - 0.5*a*new_dt**2 )
            if err>0.0001:
              rospy.logerr('cal new_dt 2 failed : err = ' + str(err))
              res.error_code = -1
              return res
            p.velocities[j] = v + a * new_dt
            p.accelerations[j] = a
          dts[i] = new_dt
      #p.time_from_start = rospy.rostime.Duration(time[i])
      res.points.append(p)
    print(dts)
    t = 0
    for i in range(len(res.points)):
      t+= dts[i]
      res.points[i].time_from_start = rospy.rostime.Duration(t)
    '''
  else:
    rospy.logerr('Unknown planning type : ' + req.type)
    res.error_code = -1
    return res  
  res.error_code = 0
  return res


def set_joint_limits(robot_desc):
  global joint_names, joint_limits
  for i in range(len(joint_names)):
    j = robot_desc.joints[i]
    if joint_names[i]!=j.name:
      rospy.logerr('Invalid joint name in description : ' + j.name)
      exit()
    jl = cJointLimit()
    if hasattr(j, 'limit') and hasattr(j.limit, 'lower') and hasattr(j.limit, 'upper'):
      jl.position[0] = j.limit.lower
      jl.position[1] = j.limit.upper
      jl.has_position = True
    
    if j.name not in joint_names:
      rospy.logerr('Joint name does not exist in description_planning : ' + j.name)
      exit()
    
    lim = velo_limits[j.name]
    if lim['has_velocity_limits']:
      jl.has_velocity = True
      jl.velocity = lim['max_velocity']
    if lim['has_acceleration_limits']:
      jl.has_acceleration = True
      jl.acceleration = lim['has_acceleration_limits']
    joint_limits.append(jl)


if __name__ == "__main__":

  rospy.init_node('affbot_planner')
  robot = URDF.from_xml_string("<robot name='myrobot'></robot>")
  kinematics.init()
  
  try:
    #robot_desc = URDF.from_xml_string(rospy.get_param("/robot_description"))
    robot_desc = URDF.from_parameter_server()
    velo_limits = rospy.get_param("/robot_description_planning")['joint_limits']
  except:
    rospy.logerr('Cannot load robot description')
    exit()
    
  set_joint_limits(robot_desc)
#  print(str(joint_limits))
  
  if hasattr(robot_desc.joints[0], 'origin') and hasattr(robot_desc.joints[0].origin, 'xyz'):
    origin_xyz = robot_desc.joints[0].origin.xyz
  else:
    rospy.logerr('Cannot find joints[0] origin : ')
    exit()
  
  class cReq:
    def __init__(self):
      self.joint_names=joint_names
      self.start_joints=[0,0,0,0,0]
      self.end_joints=[0.2,-0.3,-0.7,0.2,0.0]
      self.start_pose=kinematics.get_pose(self.start_joints)
      self.end_pose=kinematics.get_pose(self.end_joints)
      self.type="line"
      self.max_velocity=10
      self.max_acceleration=10
      self.step_time=0.1
      
      self.joint_names=[]
  for jl in joint_limits:
    jl.acceleration = 2
    jl.has_acceleration = True
  
  res = handle_planning(cReq())
  myplot(res.points)
  exit()
  
  print(robot_desc)
  s = rospy.Service('affbot_planning', AffbotPlanning, handle_planning)
  print "Start ..."
  rospy.spin()

