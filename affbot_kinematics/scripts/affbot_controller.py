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
import actionlib
from convert_motor_angle import joint2motor
from affbot_kinematics.srv import *
from affbot_kinematics.msg import *
from geometry_msgs.msg import Pose

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

ser = None
srv_get_last_plan = None


class MyAffbotMoveLastPlanAction(object):
  # create messages that are used to publish feedback/result
  _feedback = AffbotMoveLastPlanFeedback()
  _result = AffbotMoveLastPlanResult()
  
  def __init__(self):
    self._as = actionlib.SimpleActionServer("affbot/controller/move_last_plan", AffbotMoveLastPlanAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
  
  def execute_cb(self, goal):
    global ser, srv_get_last_plan
    rospy.loginfo('execute move last plan')
    self._feedback.progress = 0.0
    self._result.error_code = -1
    res = srv_get_last_plan()
    if res is None or res.error_code!=0:
      self._result.error_code = -2
      self._as.set_succeeded(self._result)
      return
    points = res.points
    
    #### move ####
    
    t_prev = 0
    max_stack = 10
    print('start moving')
    t_start = time.time()
    t_top = 0
    for i in range(len(points)):
      # feedback
      self._feedback.progress = float(i)/len(points)
      
      # preempt
      if self._as.is_preempt_requested():
        rospy.loginfo('Preempted')
        self._as.set_preempted()
        self._result.error_code = -3
        return
      # move
      p = points[i]
      if i>0:
        t = p.time_from_start.to_sec() - points[i-1].time_from_start.to_sec()
      else:
        t = p.time_from_start.to_sec()
      
      time.sleep(0.5)
      '''
      ser.set_target(t, p.positions, False)
      if i>=max_stack-1:
        while time.time()-t_start < points[i-max_stack+1].time_from_start.to_sec():
          ser.serial_read()
          time.sleep(0.01)
          if rospy.is_shutdown():
            self._result.error_code = -4
            self._as.set_succeeded(self._result)
            return
          # preempt
          if self._as.is_preempt_requested():
            rospy.loginfo('Preempted')
            self._as.set_preempted()
            self._result.error_code = -5
            self._as.set_succeeded(self._result)
            return
      else:
        ser.serial_read()
      '''
      if rospy.is_shutdown():
        self._result.error_code = -6
        self._as.set_succeeded(self._result)
        return
    self._result.error_code = 0
    self._as.set_succeeded(self._result)
    
    

def handler_move_last_plan(goal):
  # Do lots of awesome groundbreaking robot stuff here
  self.server.set_succeeded()

def move_last_plan(req):
  global srv_get_last_plan
  res = srv_get_last_plan()
  if res is None or res.error_code!=0:
    return
  

if __name__ == "__main__":
  serial_id = '0'
  try:
    #### param ####
    if len(sys.argv)>1:
      serial_id = sys.argv[1]

    #### serial ####
    '''
    ser = lib_controller.MySerial("/dev/ttyACM" + serial_id, 9600)
    ser.wait_start('central_mega')
    for i in range(5):
      ser.set_limit(i, False)
      ser.set_gear_microstep(i, False)
    ser.reset()
    '''
    
    #### ros ####
    rospy.init_node('affbot_controller')
    print("waiting 'affbot_planning'")
    rospy.wait_for_service('affbot/planner/get_last_plan')
    srv_get_last_plan = rospy.ServiceProxy('affbot/planner/get_last_plan', AffbotGetLastPlan)
    server = MyAffbotMoveLastPlanAction()
    kinematics.init()
    print('start')
    rospy.spin()
#    while not rospy.is_shutdown():
#      ser.serial_read()

  finally:
    if ser is not None:
      ser.ser.close()
