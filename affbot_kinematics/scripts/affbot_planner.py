#!/usr/bin/env python

import sys
import os
import rospy
import math
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

robot_desc = None
compute_fk = None
compute_ik = None

joint_names = ['J1', 'J2', 'J3', 'J4', 'J5']
joint_limits = []


def get_pose(ang):
  global JOINT_NAME, compute_fk
  header = Header()
  header.frame_id = 'base_link'
  robot_state = RobotState()
  robot_state.joint_state.name = JOINT_NAME
  robot_state.joint_state.position = ang
  c = compute_fk(header, ['tool0'], robot_state)
  if c.error_code.val!=1:
    rospy.logerr('compute_fk error : error_code = ' + str(c.error_code.val))
    return None
  return c.pose_stamped[0].pose

def get_joints(xyz, xyzw=None):
  global compute_ik
  target = geometry_msgs.msg.PoseStamped()
  target.header.frame_id = 'world'
  if type(xyz) is list:
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
    return None


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
      if req.joint_names[i] not in joint_limits.keys():
        rospy.logerr('Unknown joint name : ' + req.joint_names[i])
        res.error_code = -1
        return res
    start_joints = req.start_joints
    end_joints = req.end_joints
  # use pose
  else:
    start_joints = get_pose(req.start_pose)
    if start_joints is None:
      rospy.logerr('Invalid start pos')
      res.error_code = -1
      return res
    end_joints = get_pose(req.end_pose)
    if end_joints is None:
      rospy.logerr('Invalid end pos')
      res.error_code = -1
      return res
  
  if req.type=='p2p':
    pass
  else:
    pass
  
  res.error_code = 0
  return res

if __name__ == "__main__":
  rospy.init_node('affbot_planner')
  print("waiting 'compute_fk'")
  rospy.wait_for_service('compute_fk')
  compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
  print("waiting 'compute_ik'")
  rospy.wait_for_service('compute_ik')
  compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
  
  robot = URDF.from_xml_string("<robot name='myrobot'></robot>")
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
    
    if j.name not in velo_limits.keys():
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
  print(str(joint_limits))
    
  
  print(robot_desc)
  s = rospy.Service('affbot_planning', AffbotPlanning, handle_planning)
  print "Start ..."
  rospy.spin()

