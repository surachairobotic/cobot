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
import lib_controller
from convert_motor_angle import joint2motor
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from geometry_msgs.msg import Pose

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

srv_planning = None
start_joints = None
sub_joint_state = None
ser = None



if __name__ == "__main__":
  try:
    #### param ####
    if len(sys.argv)<3:
      print('param num has to be larger than 2 : ' + str(len(sys.argv)))
      exit()
    move_type = sys.argv[1]
    target_pose = Pose()
    end_joints = []
    joint_names = []
    target_type = sys.argv[2]
    
    if move_type!='line' and move_type!='p2p':
      print('move type has to be \'p\' or \'l\'')
      exit()

    try:
      if target_type=='joint':
        for i in range(5):
          end_joints.append(float(sys.argv[i+3]))
          joint_names.append('J'+str(i+1))
        velo = float(sys.argv[8])
      elif target_type=='xyz':
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
      elif target_type=='home' or target_type=='zero':
        if target_type=='home':
          end_joints = lib_controller.q_start
        else:
          end_joints = [0.0,0.0,0.0,0.0,0.0]
        for i in range(5):
          joint_names.append('J'+str(i+1))
        if len(sys.argv)>=4:
          velo = float(sys.argv[3])
        else:
          velo = 0.5
      else:
        print('target type has to be \'joint\' or \'xyz\'')
        exit()
    except ValueError:
      print('invalid value : ' + str(sys.argv[2:-1]))
      exit()

    #### serial ####
    ser = lib_controller.MySerial("/dev/ttyACM0", 9600)
    ser.wait_start('central_mega')
    for i in range(5):
      ser.set_limit(i, False)
      ser.set_gear_microstep(i, False)
    ser.reset()
    
    #### ros ####
    rospy.init_node('control_plan')
    print("waiting 'affbot_planning'")
    rospy.wait_for_service('affbot/planning')
    srv_planning = rospy.ServiceProxy('affbot/planning', AffbotPlanning)
    kinematics.init()

    #### planning ####
    print('waiting current joint state')
    start_joints = kinematics.get_current_joints()

    if target_type=='xyz':
      start_pose = kinematics.get_pose(start_joints)
      start_joints = []
    else:
      start_pose = Pose()
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
    points = res.points
    plot_plan.save('plan.txt', points)
#    plot_plan.plot_pulse(points)
#    exit()

    #### move ####
    t_prev = 0
    max_stack = 10
    print('start moving')
    t_start = time.time()
    t_top = 0
    for i in range(len(points)):
      p = points[i]
      if i>0:
        t = p.time_from_start.to_sec() - points[i-1].time_from_start.to_sec()
      else:
        t = p.time_from_start.to_sec()
      
      ser.set_target(t, p.positions, False)
      if i>=max_stack-1:
        while time.time()-t_start < points[i-max_stack+1].time_from_start.to_sec():
          ser.serial_read()
          time.sleep(0.01)
          if rospy.is_shutdown():
            exit()
      else:
        ser.serial_read()
      if rospy.is_shutdown():
        exit()
#    plot_plan.plot(points)
    while not rospy.is_shutdown():
      ser.serial_read()

  finally:
    if ser is not None:
      ser.ser.close()
