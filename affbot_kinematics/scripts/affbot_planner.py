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
import matplotlib.pyplot as plt

robot_desc = None

joint_names = kinematics.joint_names
joint_limits = []



def myplot(traj):
  pos = []
  velo = []
  acc = []
  t = []
  for i in range(len(traj)):
    pos.append(traj[i].positions[:])
    velo.append(traj[i].velocities[:])
    acc.append(traj[i].accelerations[:])
    t.append(traj[i].time_from_start.to_sec())
    
  f, axarr = plt.subplots(3, sharex=True)
  axarr[0].hold(True)
  axarr[1].hold(True)
  axarr[2].hold(True)
  
  pos = np.array(pos)
  velo = np.array(velo)
  acc = np.array(acc)
  for i in range(5):
    axarr[0].plot(t, pos[:,i], '+-')
    axarr[1].plot(t, velo[:,i], '+-')
    axarr[2].plot(t, acc[:,i], '+-')
  plt.show()


class cJointLimit:
  def __init__(self):
    self.position = [0,0]
    self.velocity = [0,0]
    self.acceleration = [0,0]

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
  
  if req.type=='p2p':
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

    print(velo)
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
    
  else:
    pass
  
  res.error_code = 0
  return res

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
    
#  print(type(robot_desc.joints[0]))
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
#  print(str(joint_limits))
  
  
  class cReq:
    def __init__(self):
      self.joint_names=joint_names
      self.start_joints=[0,0,0,0,0]
      self.end_joints=[0.2,-0.3,-0.7,0.2,0.0]
      self.start_pose=kinematics.get_pose(self.start_joints)
      self.end_pose=kinematics.get_pose(self.end_joints)
      self.type="p2p"
      self.max_velocity=math.pi
      self.max_acceleration=math.pi
      self.step_time=0.1
      
      self.joint_names=[]
      
  
  res = handle_planning(cReq())
  myplot(res.points)
  print('xx')
  exit()
  
  print(robot_desc)
  s = rospy.Service('affbot_planning', AffbotPlanning, handle_planning)
  print "Start ..."
  rospy.spin()

