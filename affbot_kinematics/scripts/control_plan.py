#!/usr/bin/env python

import sys
import os
import rospy
import math
import time
import my_kinematics as kinematics
import plot_plan
import serial
import copy
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from geometry_msgs.msg import Pose

from std_msgs.msg import Header
from sensor_msgs.msg import JointState


srv_planning = None
start_joints = None
sub_joint_state = None
ser = None
serial_buf = ''

def planning(start_pose, end_pose):
  global srv_planning, joint_names
  try:
    req = AffbotPlanRequest()
    start_pose = start_pose
    end_pose = end_pose
    type = "p2p"
    max_velocity = 1

    res = srv_planning(joint_names=joint_names
    , start_joints=[0,0,0,0,0]
    , end_joints=[0.2,0.3,0.7,0.2,0.1]
    , start_pose=start_pose
    , end_pose=end_pose
    , type="line"
    , max_velocity=3
    , max_acceleration=3
    , step_time=0.1)
    return res
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)


def serial_read():
  global serial_buf, ser
  c = ser.read()
  while len(c)>0:
    if ord(c)<128:
      c = c.decode()
      if c=='\n' or c=='\r':
        if len(serial_buf)>0:
          b = serial_buf
          serial_buf = ''
          print('READ : ' + b)
          return b
      else:
        serial_buf+= c
    c = ser.read()
  return ''


def callback_joint_state(joints):
  global sub_joint_state, start_joints
  start_joints = copy.deepcopy(joints)
  sub_joint_state.unregister()
  sub_joint_state = None
  

if __name__ == "__main__":
  try:
    if len(sys.argv)!=5:
      print('param num has to be 6 : ' + str(len(sys.argv)))
      exit()
    move_type = sys.argv[1]
    if move_type!='line' and move_type!='p2p':
      print('move type has to be \'p\' or \'l\')
      exit()
    xyz = []
    try:
      for i in range(3):
        xyz.append(float(sys.argv[i+2]))
      velo = float(sys.argv[5])
    except ValueError:
      print('invalid value : ' + str(sys.argv[2:-1]))
      exit()
    
    rospy.init_node('control_plan')
    print("waiting 'affbot_planning'")
    rospy.wait_for_service('affbot_planning')
    srv_planning = rospy.ServiceProxy('affbot_planning', AffbotPlanning)
    sub_joint_state = rospy.Subscriber("/joint_state", JointState, callback_joint_state)
    kinematics.init()
    
    target_pose = Pose()
    target_pose.position.x = xyz[0]
    target_pose.position.y = xyz[1]
    target_pose.position.z = xyz[2]
    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.orientation.w = 1
    
    while sub_joint_state is not None and not :
      rospy.sleep(0.01)
      if rospy.is_shutdown():
        exit()
    
    start_pose = kinematics.get_pose(start_joints.position)
    res = srv_planning(joint_names=[]
      , start_joints=[]
      , end_joints=[]
      , start_pose=start_pose
      , end_pose=target_pose
      , type=move_type
      , max_velocity=velo
      , max_acceleration=3
      , step_time=0.1)
    if res.error_code!=0:
      print('planning error : '+str(res.error_code))
      exit()
    
    t_prev = 0
    max_stack = 10
    t_start = time.time()
    points = res.points
    for i in range(len(points)):
      p = points[i]
      t = p.time_from_start.to_sec()

      cmd = 'p%.3f %.3f %.3f ' % (p.positions[0], p.velocities[0], t)
      t_prev= t
      ser.write((cmd + '\n').encode('ascii','ignore'))
      if i>=max_stack-1:
        while time.time()-t_start < points[i-max_stack+1].positions[0]:
          serial_read()
          time.sleep(0.01)
      else:
        serial_read()
  finally:
    if ser is not None:
      ser.close()
