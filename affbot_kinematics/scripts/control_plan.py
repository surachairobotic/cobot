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
from convert_motor_angle import joint2motor
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from geometry_msgs.msg import Pose

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

enable_ser = True

srv_planning = None
start_joints = None
sub_joint_state = None
ser = None
serial_buf = ''

def wait_start():
  global ser
  print('wait start')
  for i in range(5):
    ser.write('who\n')
    t = time.time() + 1
    while time.time() < t:
      s = serial_read()
      if s[0:4]=='who:':
        name = s[4:]
        if name=='central_mega' or name=='controller_pos_velo':
          print('start')
          return
        else:
          print('wrong serial name : %s' % (name))
          exit()
  print('wait start failed')
  exit()

def serial_read():
  global serial_buf, ser, enable_ser
  if not enable_ser:
    return ''
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
    if len(sys.argv)<=6:
      print('param num has to be larger than 6 : ' + str(len(sys.argv)))
      exit()
    move_type = sys.argv[1]
    if move_type!='line' and move_type!='p2p':
      print('move type has to be \'p\' or \'l\'')
      exit()

    target_pose = Pose()
    end_joints = []
    joint_names = []
    target_type = sys.argv[2]
    try:
      if target_type=='joint':
        for i in range(5):
          end_joints.append(float(sys.argv[i+3]))
          joint_names.append('J'+str(i+1))
        velo = float(sys.argv[8])
      elif target_type!='xyz':
        xyz = []
        for i in range(3):
          xyz.append(float(sys.argv[i+3]))
        target_pose.position.x = xyz[0]
        target_pose.position.y = xyz[1]
        target_pose.position.z = xyz[2]
        target_pose.orientation.x = 1
        target_pose.orientation.y = 0
        target_pose.orientation.z = 0
        target_pose.orientation.w = 0
        velo = float(sys.argv[6])
      else:
        print('target type has to be \'joint\' or \'xyz\'')
        exit()
    except ValueError:
      print('invalid value : ' + str(sys.argv[2:-1]))
      exit()

    if enable_ser:
      ser = serial.Serial()
  #    ser.port = "COM5"
      ser.port = "/dev/ttyACM0"
      ser.baudrate = 57600
      ser.timeout = 0.001
      ser.writeTimeout = 1
      ser.open()
      wait_start()

    rospy.init_node('control_plan')
    print("waiting 'affbot_planning'")
    rospy.wait_for_service('affbot_planning')
    srv_planning = rospy.ServiceProxy('affbot_planning', AffbotPlanning)
    sub_joint_state = rospy.Subscriber("/joint_states", JointState, callback_joint_state)
    kinematics.init()

    print('waiting current joint state')
    while sub_joint_state is not None:
      rospy.sleep(0.01)
      if rospy.is_shutdown():
        exit()

    if target_type=='joint':
      start_joints = start_joints.position
      start_pose = Pose()
    else:
      start_pose = kinematics.get_pose(start_joints.position)
      start_joints = []
    print('start planning')
    res = srv_planning(joint_names=joint_names
      , start_joints=start_joints
      , end_joints=end_joints
      , start_pose=start_pose
      , end_pose=target_pose
      , type=move_type
      , max_velocity=velo
      , max_acceleration=1
      , step_time=0.1)
    if res.error_code!=0:
      print('planning error : '+str(res.error_code))
      exit()

    t_prev = 0
    max_stack = 10
    points = res.points
    plot_plan.save('plan.txt', points)

    print('start moving')
    t_start = time.time()
    for i in range(len(points)):
      p = points[i]
      if i>0:
        t = p.time_from_start.to_sec() - points[i-1].time_from_start.to_sec()
      else:
        t = p.time_from_start.to_sec()
#      cmd = 'p%.3f %.3f %.3f ' % ( (p.positions[0] - joint_offset[0])/joint_gear_ratio[0], p.velocities[0], t)
      cmd = 'p'
      q_motor = joint2motor(p.positions)
      for i in range(1):# range(len(p.positions)):
        cmd+= '%.3f %.3f ' % ( q_motor[i], t )
      t_prev= t
      if enable_ser:
        ser.write((cmd + '\n').encode('ascii','ignore'))
      print(cmd)
      if i>=max_stack-1:
        while time.time()-t_start < points[i-max_stack+1].positions[0]:
          serial_read()
          time.sleep(0.01)
          if rospy.is_shutdown():
            exit()
      else:
        serial_read()
      if rospy.is_shutdown():
        exit()
#    plot_plan.plot(points)

  finally:
    if ser is not None:
      ser.close()
