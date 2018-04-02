#!/usr/bin/env python

import sys
import os
import rospy
import math
import numpy as np
import copy
import my_kinematics as kinematics

#from AffbotPlanningRequest.msg import *
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import tf.transformations
import matplotlib.pyplot as plt
import line_planner
import p2p_planner
import plot_plan

robot_desc = None

joint_names = kinematics.joint_names
joint_limits = []
display_pub = None
last_error_code = -1
last_plan = None

class MyException(Exception):
    def __init__(self, msg, err_code=-1):
        Exception.__init__(self)
        self.msg = msg
        self.err_code = err_code
        

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



def check_plan(points, joint_limits):
  for p in points:
    for i in range(len(p.positions)):
      jl = joint_limits[i]
      if jl.has_position and \
        (p.positions[i]<jl.position[0] or p.positions[i]>jl.position[1]):
        raise MyException('Joint angle exceeds the limit [{0}] : {1}'.format(i, p.positions[i]), -2)
      if jl.has_velocity and abs(p.velocities[i])>jl.velocity:
        raise MyException('Joint velocity exceeds the limit [{0}] : {1}'.format(i, p.velocities[i]), -2)
      if jl.has_acceleration and abs(p.accelerations[i])>jl.acceleration:
        raise MyException('Joint acceleration exceeds the limit [{0}] : {1}'.format(i, p.accelerations[i]), -2)


def display_plan(points):
  global joint_names, display_pub
  j = JointTrajectory()
  j.points = points
  j.joint_names = joint_names
  j.header = Header()
  j.header.stamp = rospy.Time.now()
  
  r = RobotTrajectory()
  r.joint_trajectory = j
  
  d = DisplayTrajectory()
  d.model_id = ""
  d.trajectory = [r]
  display_pub.publish(d)


def handle_get_last_plan(req):  
  global last_error_code, last_plan
  if last_error_code==0:
    return last_plan, 0
  else:
    rospy.logerr("No plan found")
    return [], last_error_code
  

def handle_planning(req):
  global joint_limits, joint_names, last_error_code, last_plan
  try:
    last_error_code = 1
    last_plan = None
    res_points = []
    res_error_code = 0
    
    if len(req.joint_names)>0 and len(req.start_joints)==0:
      req.start_joints = kinematics.get_current_joints()
      print('start joints')
      print(req.start_joints)
    if req.type=='p2p':
      res_points = p2p_planner.plan(req, joint_limits, joint_names)
    elif req.type=='line':
      res_points = line_planner.plan(req, joint_limits, joint_names, origin_xyz)
    else:
      raise MyException('Unknown planning type : ' + req.type)
    check_plan(res_points, joint_limits)
    display_plan(res_points)
    
    last_plan = copy.deepcopy(res_points)
    last_error_code = 0
    return res_points, 0
  except MyException as e:
    rospy.logerr(e.msg)
    return [], e.err_code

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
      jl.acceleration = lim['max_acceleration']
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
  print(str(joint_limits))
  
  if hasattr(robot_desc.joints[0], 'origin') and hasattr(robot_desc.joints[0].origin, 'xyz'):
    origin_xyz = robot_desc.joints[0].origin.xyz
  else:
    rospy.logerr('Cannot find joints[0] origin : ')
    exit()
    
  display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)
  '''
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
    jl.velocity = 0.5
    jl.has_velocity = True
  
  res = handle_planning(cReq())
  plot_res.plot(res)
  exit()
  '''
  
  srv_planning = rospy.Service('affbot/planner/planning', AffbotPlanning, handle_planning)
  srv_get_last_plan = rospy.Service('affbot/planner/get_last_plan', AffbotGetLastPlan, handle_get_last_plan)
  print "Start ..."
  rospy.spin()

