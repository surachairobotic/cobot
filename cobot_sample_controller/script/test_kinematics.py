#!/usr/bin/env python

## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import random
import math
import tf
import numpy as np
from pprint import pprint
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import OrientationConstraint
from sensor_msgs.msg import JointState
#from test_dynamixel.msg import Goal
from time import sleep
from tf.transformations import *
#euler_from_quaternion, quaternion_from_euler, quaternion_multiply

import geometry_msgs.msg 
## END_SUB_TUTORIAL

from std_msgs.msg import String


compute_fk = None
compute_ik = None
JOINT_NAME = ['J1', 'J2', 'J3', 'J4', 'J5']
#JOINT_NAME = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


def get_pose(ang):
  global JOINT_NAME, compute_fk
  header = Header()
  header.frame_id = 'base_link'
  robot_state = RobotState()
  robot_state.joint_state.name = JOINT_NAME
  robot_state.joint_state.position = ang
  c = compute_fk(header, ['tool0'], robot_state)
  if c.error_code.val!=1:
    raise 'compute_fk error : error_code = ' + str(c.error_code.val)
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

def test_fik(joints):
  pose = get_pose(joints)
  get_joints(pose)

def test_start_goal(joints):
  pose = get_pose(joints)
  print(pose)
  start = get_joints(pose)
  print('start : ' + str(start))
  pose.position.y+= 0.21
  goal = get_joints(pose)
  print('goal : ' + str(goal))

def test_fk():
  c = get_pose([0,0,0,0,0])
  print(c)

def test_ik():
  xyz = [0.248, -0.197, 0.628]
  xyzw = [0.936, -0.348, -0.032, -0.030]
  xyz = [0.10000493677,-0.19999979427,0.656249008225]
  xyzw = [0.832026040222,-0.554736575651,-5.0938709911e-06,-1.01850365978e-06]
#  xyz = [0.100, -0.200, 0.656]
#  xyzw = [0.832, -0.556, 0,0]
  
  get_joints(xyz, xyzw)

def cap_ang(q):
  while q>math.pi:
    q-= math.pi*2
  while q<-math.pi:
    q+= math.pi*2
  return q

def create_1_test_data(q):
  pi = math.pi
  pose = get_pose(q)
  s = '{{' + '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}'.format(
    pose.position.x,pose.position.y,pose.position.z
    , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    , q[0], q[1], q[2], q[3], q[4]) + '}};'
  print(s)
  
  ang, q5 = get_ang_q5(pose)
  ang = -cap_ang(pi * 2 - q[1] - q[2] - q[3])
  
  B = 0.3
  A = 0.4
#  ang = pi * 2 - q[1] - q[2] - q[3]
  x2 = A * math.cos(pi*0.5 - q[1]) + B * math.cos( - q[1] - q[2])
  z2 = A * math.sin(pi*0.5 - q[1]) + B * math.sin( - q[1] - q[2])
  C = math.sqrt(x2*x2 + z2*z2)
  cos_b = (B*B + C*C -A*A)/(2*B*C)
  cos_a = (C-B*cos_b)/A
  a = math.acos(cos_a)
  b = math.acos(cos_b)
#  if C - A*cos_a < 0.0:
#      b = math.pi - b;
  q = [0,0,0,0,0]
  q[1] = pi*0.5 - (math.atan2(z2,x2) + a)
  q[2] = pi*0.5 - (pi - a - b)
  q[3] = ang + (pi*2 - q[1] - q[2])
  
  q[4] = q5
  for i in range(len(q)):
    q[i] = cap_ang(q[i])
    
  print('ang : {0}\nC : {1}\nxz2 : {2} , {3}\nab : {4} , {5}'.format(ang,C,x2,z2,a,b))
#  print('q0 : {0}\nq1 : {1}\nq2 : {2}\nq3 : {3}\nq4 : {4}'.format(q[0],q[1],q[2],q[3],q[4]))
  print(q)
  print('cos : {0} , {1} / {2} , {3}'.format(a, cos_a,b, cos_b))
  print('cos err : {0},{1}'.format(cos_a - math.cos(a), cos_b-math.cos(b)))


  
def create_test_data(num):
  random.seed()
  pi = math.pi
  q = []
  
  print('{')
  
  for i in range(num):
    while 1:
      q = []
      q.append(0)
      q.append(random.uniform(-pi*0.4, pi*0.4))
      q.append(random.uniform(-pi*0.4, pi*0.4))
      q.append(random.uniform(-pi*0.4, pi*0.4))
      q.append(random.uniform(-pi, pi))
      pose = get_pose(q)
#      print(pose)
      if pose.position.y < -0.11:
#        print(pose.position.y)
        break
    q[0] = random.uniform(-pi, pi)
    pose = get_pose(q)
    s = '{' + '{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}'.format(
      pose.position.x,pose.position.y,pose.position.z
      , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
      , q[0], q[1], q[2], q[3], q[4]) + '}'
    if i!=num-1:
      s+= ','
    print(s)
  print('};')
  
  
  
##################

def get_ang_q5(pose):
  o = pose.orientation
  q = [o.x,o.y,o.z,o.w]
  R = quaternion_matrix(q)
  x0 = 0.1
  y0 = 0.1
  print('qt : ' + str(q))
  
  x1 = np.array(R[0:3,2])
  x2 = np.array([pose.position.x - x0, pose.position.y - y0, 0])
  x3 = np.cross(x1,x2)
  
  # when ee direction is parallel to the ground
  if abs(np.linalg.norm(x3))<0.00001:
    x4 = np.array([0.0,0.0,-1.0])
  else:
    x3/= np.linalg.norm(x3)
    x4 = np.cross(x3,x1)
    x4/= np.linalg.norm(x4)
  print('x1 : ' + str(x1))
  print('x4 : ' + str(x4))
 
  n = [0,0,-1]
  dd = (R[0][2]*n[0] + R[1][2]*n[1] + R[2][2]*n[2])
  cos_ang = R[0][2]*n[0] + R[1][2]*n[1] + R[2][2]*n[2]
  if cos_ang > 1.0 and cos_ang < 1.0001:
    cos_ang = 1
  elif cos_ang < -1.0 and cos_ang > -1.0001:
    cos_ang = -1
  ang = math.acos(cos_ang)
  c = np.dot(x4, np.array(R[0:3,0]))
  s = np.dot(x4, np.array(R[0:3,1]))
  if R[2][2]>0:
    c = -c
    s = -s
  print('cs : {0} , {1}, R22 : {2}'.format(c, s, R[2][2]))
  print(R)
  q5 = -math.atan2(c,s)
  #q5 = math.acos( np.dot(x4, np.array(R[0:3,1])))
  if (abs(x1[0])>0.0001 and abs(x2[0])>0.0001 and x1[0]*x2[0]<0) or \
     (abs(x1[1])>0.0001 and abs(x2[1])>0.0001 and x1[1]*x2[1]<0):
    ang = abs(ang)
  else:
    ang = -abs(ang)
  return ang, q5
  
  

def test_quat():
  '''
  y = 0.498589
  p = -0.029507
  r = -3.046031
  q = quaternion_from_euler (r,p,y)
  '''
  pose = get_pose([0,0,0.15,-0.7,0.1])
  o = pose.orientation
  q = [o.x,o.y,o.z,o.w]
  print(q)
  R = quaternion_matrix(q)
  print(R)
  x = R[0:3,0]
  y = R[0:3,1]
  z = R[0:3,2]
  n = [0,0,-1]
  
  x0 = 0.1
  y0 = 0.1
  
  x1 = np.array(R[0:3,2])
  x2 = np.array([pose.position.x - x0, pose.position.y - y0, 0])
  x3 = np.cross(x1,x2)
  x3/= np.linalg.norm(x3)
  x4 = np.cross(x3,x1)
  x4/= np.linalg.norm(x4)
#  print(np.cross(x4,x3) - x1)
#  print(math.acos( np.dot(x4, np.array(R[0:3,0]))))
#  print(math.acos( np.dot(x4, np.array(R[0:3,1]))))
  q4 = math.acos(R[0][2]*n[0] + R[1][2]*n[1] + R[2][2]*n[2])
  q5 = math.acos( np.dot(x4, np.array(R[0:3,1])))
  print(x1)
  print(x2)
  if (abs(x1[0])>0.0001 and abs(x2[0])>0.0001 and x1[0]*x2[0]<0) or \
     (abs(x1[1])>0.0001 and abs(x2[1])>0.0001 and x1[1]*x2[1]<0):
    q4 = q4
  else:
    q4 = -q4
  print(q4)
  print(q5)
  return
  
  
  for i in range(3):
    print(math.acos(R[0][i]*n[0] + R[1][i]*n[1] + R[2][i]*n[2]))
  
  print(euler_from_quaternion(q))
  '''
  qq = math.sqrt(q[0]*q[0] + q[2]*q[2] + q[3]*q[3])
  q = [q[0]/qq, 0, q[2]/qq, q[3]/qq]
#  q2 = quaternion_from_euler (0,p,0)
#  q = quaternion_multiply(q,q2)
  print(q)
  (roll, pitch, yaw) = euler_from_quaternion(q)
  print(roll)
  print(pitch)
  print(yaw)
  '''
##################
  

if __name__=='__main__':
  try:
    rospy.init_node('test_kinematics',
                    anonymous=True)
    rospy.wait_for_service('compute_fk')
    compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
#    test_quat()
    create_1_test_data([0.000000, 0.000000, 0.000000, -0.200000, 0.400000])
#    create_1_test_data([-2.44991581808,0.603187322526,0.854758954576,-0.37473148284,0])
#    create_1_test_data([-1.7,-0.2,0.15,-1,0])
#    create_test_data(100)
#    test_start_goal([0.1,0.2,0.7,1,0])
#    test_start_goal([-0.001,-0.511,0.339,0.000,0.000])
  except rospy.ROSInterruptException:
    pass

