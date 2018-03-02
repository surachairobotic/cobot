#!/usr/bin/env python

import sys
import os
import rospy
import math
import time
import my_kinematics as kinematics
import plot_plan
import serial
from affbot_kinematics.srv import *
from geometry_msgs.msg import Pose
from affbot_kinematics.msg import *

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState



srv_planning = None
joint_names = kinematics.joint_names


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
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


serial_buf = ''
def serial_read(ser):
  global serial_buf
  c = ser.read()
  if len(c)>0 and ord(c)<128:
    c = c.decode()
    while len(c)>0:
      if c=='\n' or c=='\r':
        if len(serial_buf)>0:
          b = serial_buf
          serial_buf = ''
          print('READ : ' + b)
          return b
      else:
        serial_buf+= c
      c = ser.read().decode()
  return ''


if __name__ == "__main__":
  rospy.init_node('control_plan')
  '''
  print("waiting 'affbot_planning'")
  rospy.wait_for_service('affbot_planning')
  srv_planning = rospy.ServiceProxy('affbot_planning', AffbotPlanning)
  kinematics.init()
  
  print('start')
  start_pose =   kinematics.get_pose([0,0,0,0,0])
  end_pose = kinematics.get_pose([0.2,0.3,0.7,0.2,0.1])
  res = planning(start_pose, end_pose)
  '''
  
  try:
    ser = serial.Serial()
    ser.port = "/dev/ttyACM0"
    ser.baudrate = 115200
    ser.timeout = 0.001
    ser.writeTimeout = 1
    ser.open()


    val_num = 17
    points = []
    with open('plan.txt', 'rt') as f:
      line = f.readline()
      while line:
        vals = line.split(' ')
        if len(vals)!=val_num:
          print('invalid val num : ' + str(len(vals)))
          exit()
        v = []
        for i in range(val_num-1):
          v.append(float(vals[i]))
        points.append(v)
        line = f.readline()
    
    t_start = rospy.Time.now()
    t_prev = 0
    for p in points:
      p[0]*=1
      print('t = {0}, pos = {1}'.format(p[0], p[1]))
      t = p[0]# - 0.05
      while t > (rospy.Time.now() - t_start).to_sec() and not rospy.is_shutdown():
        serial_read(ser)
        rospy.sleep(0.001)
      if rospy.is_shutdown():
        print('ros shutdown')
        break
      cmd = 'p{0} {1} {2} '.format(p[1], p[6], p[0] - t_prev)
      t_prev= p[0]
      ser.write(cmd + '\n')
      print(cmd)
    
    print('end')
    while not rospy.is_shutdown():
      serial_read(ser)
      rospy.sleep(0.001)
  except KeyboardInterrupt:
    print('SIGINT')
  except Exception as e:
    print(e)
  finally:
    ser.close()
    
    
    
    
