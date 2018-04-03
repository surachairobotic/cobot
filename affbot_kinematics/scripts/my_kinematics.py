#!/usr/bin/env python

import sys
import os
import rospy
import math
import time
import copy

import geometry_msgs.msg
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

joint_names = ['J1', 'J2', 'J3', 'J4', 'J5']

compute_fk = None
compute_ik = None
origin_xyz = None



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


#### IK, FK ####

def get_pose(ang):
  global JOINT_NAME, compute_fk
  header = Header()
  header.frame_id = 'base_link'
  robot_state = RobotState()
  robot_state.joint_state.name = joint_names
  robot_state.joint_state.position = ang
  c = compute_fk(header, ['tool0'], robot_state)
  if c.error_code.val!=1:
    raise Exception('compute_fk error : error_code = ' + str(c.error_code.val))
   # rospy.logerr('compute_fk error : error_code = ' + str(c.error_code.val))
   # return None
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
      raise Exception('error : ' + str(resp.error_code.val))
      #print('error : ' + str(resp.error_code.val))
      
    for v in resp.solution.joint_state.position:
      if math.isnan(v):
        print(target.pose)
        raise Exception("ik nan : " + str(resp.solution.joint_state.position))
    return resp.solution.joint_state.position
  except rospy.ServiceException, e:
    raise Exception("Service call failed: %s"%e)
    #print "Service call failed: %s"%e
    #return None


#### get origin xyz ####

def get_origin_xyz():
  global origin_xyz
  if origin_xyz is None:
    robot_desc = URDF.from_parameter_server()
    origin_xyz = robot_desc.joints[0].origin.xyz
  return origin_xyz


#### get current pose ####

sub_joint_state = None
start_joints = None

def callback_joint_state(joints):
  global sub_joint_state, start_joints
  start_joints = copy.deepcopy(joints)
  sub_joint_state.unregister()
  sub_joint_state = None


def get_current_joints():
  global sub_joint_state, start_joints
  sub_joint_state = rospy.Subscriber("/joint_states", JointState, callback_joint_state)
  t = time.time()
  while sub_joint_state is not None:
    if time.time()-t > 1.0:
      sub_joint_state.unregister()
      sub_joint_state = None
      raise Exception('get_current_pose() : time out')
  return start_joints.position

def get_current_pose():
  return get_pose(get_current_joints())

#### init ####

def init():
  global compute_ik, compute_fk
  
  print("waiting 'compute_fk'")
  rospy.wait_for_service('compute_fk')
  compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
  print("waiting 'compute_ik'")
  rospy.wait_for_service('compute_ik')
  compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)


#### main ####

if __name__ == "__main__":
  if len(sys.argv)<=1:
    exit()
  init()
  if sys.argv[1]=='fk':
    if len(sys.argv)!=7:
      print('FK param num has to be 7')
      exit()
    ang = []
    for i in range(5):
      ang.append(float(sys.argv[i+2]))
    print(get_pose(ang))
  elif sys.argv[1]=='ik':
    if len(sys.argv)!=5:
      print('IK param num has to be 5')
      exit()
    xyz = []
    for i in range(3):
      xyz.append(float(sys.argv[i+2]))
    print(get_joints(xyz))
    
  
  
