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
from threading import Lock

ser = None
srv_get_last_plan = None
b_execute = False
b_1_motor = False
motor_id = 0
lock_serial = Lock()


class MyAffbotMoveLastPlanAction(object):
  # create messages that are used to publish feedback/result
  _feedback = AffbotMoveLastPlanFeedback()
  _result = AffbotMoveLastPlanResult()
  
  def __init__(self):
    self._as = actionlib.SimpleActionServer("affbot/controller/move_last_plan", AffbotMoveLastPlanAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
#    self._as.register_preempt_callback(self.preempt_cb)
  
#  def preempt_cb(self):
#    rospy.loginfo('preem')
    
    
  def small_loop(self):
    # read serial
    ser.serial_read()
    
    # shutdown
    if rospy.is_shutdown():
      self._result.error_code = -4
      self._as.set_succeeded(self._result)
      return False
    # preempt
    if self._as.is_preempt_requested():
      rospy.loginfo('Preempted')
      self._as.set_preempted()
      self._result.error_code = -5
      self._as.set_succeeded(self._result)
      return False
      
    return True
    
  def execute_cb(self, goal):
    global ser, srv_get_last_plan, b_execute, b_1_motor
    lock_serial.acquire()
    try:
      b_execute = True
      self._feedback.progress = 0.0
      self._result.error_code = -1
      res = srv_get_last_plan()
      if res is None or res.error_code!=0:
        rospy.logerr('Cannot get last plan')
        self._result.error_code = -2
        self._as.set_succeeded(self._result)
        return
      points = res.points
      rospy.loginfo('execute move last plan')
      
      #### move ####
      
      t_prev = 0
      max_stack = 5
      print('start moving')
      t_start = time.time()
      t_top = 0
      
      
      # test moving continuously
      ''' 
      p = [0]*5
      i = 0
      t = 0.1
      print('p : ' + str(points[0].positions[0]))
      while 1:
        i+=1
        if not self.small_loop():
          return
        
        # move
        p[0] = points[0].positions[0] + math.cos(2*math.pi*i*t*0.01) - 1
        ser.set_target(t, p, b_1_motor)
        print('%f : %f' % (i*t, p[0]))
        if i>=max_stack-1:
          while time.time()-t_start < (i-max_stack+1)*t*1.017:
            time.sleep(0.01)
            # read serial
            if len(ser.serial_read())>0:
              return
            # shutdown
            if rospy.is_shutdown():
              return
      '''
      for i in range(len(points)):
        # feedback
        self._feedback.progress = float(i)/len(points)
        self._as.publish_feedback(self._feedback)
#        print('feedback : ' + str(self._feedback.progress))
        
        
        if not self.small_loop():
          return
        
        # move
        p = points[i]
        if i>0:
          t = p.time_from_start.to_sec() - points[i-1].time_from_start.to_sec()
        else:
          t = p.time_from_start.to_sec()
        ser.set_target(t, p.positions, b_1_motor)
        if i>=max_stack-1:
          while time.time()-t_start < points[i-max_stack+1].time_from_start.to_sec()*1.017:
            time.sleep(0.01)
            if not self.small_loop():
              return
      '''
      print('end')
      while 1:
        print('who')
        rospy.sleep(1.0)
        ser.ser.write('who\r\n')
        if len(ser.serial_read())>0:
          break
      '''
      self._result.error_code = 0
      self._as.set_succeeded(self._result)
    finally:
      b_execute = False
      lock_serial.release()
    


def handler_move_last_plan(goal):
  # Do lots of awesome groundbreaking robot stuff here
  self.server.set_succeeded()

def move_last_plan(req):
  global srv_get_last_plan
  res = srv_get_last_plan()
  if res is None or res.error_code!=0:
    return

def set_zero(req):
  global ser, b_execute
  if ser is None:
    print('ser is not initialized')
  elif b_execute:
    print('robot is moving')
  else:
    print('reset_arduino')
    lock_serial.acquire()
    try:
      ser.reset_arduino()
      ser.reset_joint_state()
    finally:
      lock_serial.release()
  return []

if __name__ == "__main__":
  rospy.init_node('affbot_controller')
  serial_id = '0'
  try:
    #### param ####

    if len(sys.argv)>1:
      serial_id = sys.argv[1]

    #### serial ####
    '''
    sers = lib_controller.get_arduinos("/dev/ttyACM", 1, 9600, 'central_mega')
    ser = sers
    '''
    print('port_id = ' + serial_id)
    ser = lib_controller.MySerial("/dev/ttyACM" + serial_id, 9600)
    if b_1_motor:
      rospy.sleep(1.0)
      ser.wait_start('controller_pos_velo')
      ser.set_limit(motor_id, True)
      ser.set_gear_microstep(motor_id, True)
    else:
      ser.wait_start('central_mega')
      for i in range(5):
        ser.set_limit(i, False)
        ser.set_gear_microstep(i, False)
    ser.reset_arduino()
    
    #### ros ####
    
    print("waiting 'affbot_planning'")
    rospy.wait_for_service('affbot/planner/get_last_plan')
    srv_get_last_plan = rospy.ServiceProxy('affbot/planner/get_last_plan', AffbotGetLastPlan)
    srv_set_zero = rospy.Service('affbot/controller/set_zero', AffbotSetZero, set_zero)
    server = MyAffbotMoveLastPlanAction()
    kinematics.init()
    print('start')
    while not rospy.is_shutdown():
      rospy.sleep(0.01)
      lock_serial.acquire()
      ser.serial_read()
      lock_serial.release()

  finally:
    lock_serial.release()
    if ser is not None:
      ser.ser.close()
