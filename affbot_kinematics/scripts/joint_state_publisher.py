#!/usr/bin/env python

import sys
import os
import math
import time
import serial
import rospy
import copy
import lib_controller
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from affbot_kinematics.srv import AffbotSetZero


joint_name = ['J1', 'J2', 'J3', 'J4', 'J5']
motor_position = [0.0,0.0,0.0,0.0,0.0]
motor_velocity = [0.0,0.0,0.0,0.0,0.0]
joint_effort = [0.0,0.0,0.0,0.0,0.0]
sers = []
last_plan = None

'''
class MySerial:
  def __init__(self, port_num, name):
    self.ser = serial.Serial()
    self.ser.port = "/dev/ttyACM" + str(port_num)
    self.ser.baudrate = 115200
    self.ser.timeout = 0.001
    self.ser.writeTimeout = 1
    self.ser.open()
    self.serial_buf = ''
    self.b_start = False
    self.t_prev_wait_start = -1
    self.name = name
    
  def read(self):
    c = self.ser.read()
    while len(c)>0:
      if ord(c)<128:
        c = c.decode()
        if c=='\n' or c=='\r':
          if len(self.serial_buf)>0:
            b = self.serial_buf
            self.serial_buf = ''
  #          print('READ : ' + b)
            return b
        else:
          self.serial_buf+= c
      c = self.ser.read()
    return ''

  def wait_start(self):
    if self.b_start:
      return True
    t = time.time()
    if t > self.t_prev_wait_start:
      self.t_prev_wait_start = t + 1
      self.ser.write('who\n')
    s = self.read()
    if len(s)>0:
      if s[0]!='<':
        print('READ : '+s)
      if s[0:4]=='who:':
        name = s[4:]
        if self.name==name:
          self.b_start = True
          return True
        else:
          print('wrong serial name (%s) : %s' % (self.ser.port, name))
          exit()
    return False
    
  def set_zero(self):
    self.ser.write('reset\n')
'''

def set_zero(req):
  global sers
  for ser in sers:
    ser.reset()
  return []



if __name__ == "__main__":
  if len(sys.argv)<=1:
    print('no port num found : ')
    exit()
    
  try:
    # init serial
    for i in range(len(sys.argv)-1):
      sers.append(lib_controller.MySerial("/dev/ttyACM" + sys.argv[i+1], 115200))
      
    # init ros
    rospy.init_node('affbot_joint_state_publisher')
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    srv = rospy.Service('affbot/joint_state_publisher/set_zero', AffbotSetZero, set_zero)
    
    print('waiting start')
    try:
      for i in range(len(sys.argv)-1):
        sers[i].wait_start('get_pulse_due')
        sers[i].reset()
      '''
      while 1:
        b = True
        for ser in sers:
          if ser.wait_start()==False:
            b = False
        if b:
          break
        if rospy.is_shutdown():
          exit()
        time.sleep(0.01)
      while 1:
        b = True
        for ser in sers:
          if ser.wait_start()==False:
            b = False
        if b:
          break
        if rospy.is_shutdown():
          exit()
        time.sleep(0.01)
      '''
    except serial.serialutil.SerialException:
      exit()
    print('start')
    while not rospy.is_shutdown():
      time.sleep(0.01)
      for k in range(len(sers)):
        ser = sers[k]
        s = ser.serial_read()
        if len(s)>3:
#          print(s)
          if s[0]=='<' and s[-1]=='>':
            start_id = int(s[1])
            arr = s[3:-2].split(' ')
            if len(arr)%2!=0:
              print('Invalid data len (%d): %s' %( len(arr), s ))
              continue
            n_joint = len(arr) / 2
            if n_joint>2:
              print('Invalid join num (%d): %s' %( n_joint, s ))
              continue
            '''
            if len(arr)==4:
              n_joint = 2
              start_id = 0
            elif len(arr)==6:
              n_joint = 3
              start_id = 2
            else:
              print('Invalid data len (%d): %s' %( len(arr), s ))
              continue
            '''
            try:
              j = JointState()
              j.header.stamp = rospy.Time.now()
              j.header.frame_id = 'base_link'
              j.name = joint_name
              
              q_motor = []
              dq_motor = []
              for i in range(n_joint):
                r = 1000.0 / (lib_controller.MICROSTEP[i] * lib_controller.GEAR_RATIO[i])
                motor_position[start_id + i] = float(arr[i*2]) * r
                motor_velocity[start_id + i] = float(arr[i*2+1]) * r
              joint_position = lib_controller.motor2joint( motor_position )
              joint_velocity = lib_controller.motor2joint( motor_velocity, add_q_start=False )
              '''
              for i in range(n_joint):
                joint_position[start_id + i] = q_link[i]
                joint_velocity[start_id + i] = dq_link[i]
                #joint_position[start_id + i] = float(arr[i*2]) * joint_gear_ratio[start_id + i] + joint_offset[start_id + i]
                #joint_velocity[start_id + i] = float(arr[i*2+1]) * joint_gear_ratio[start_id + i]
              '''
              j.position = joint_position
              j.velocity = joint_velocity
              j.effort = joint_effort
              pub.publish(j)
            except ValueError:
              print('Invalid data vale : ' + s)
    print('end')
  except KeyboardInterrupt:
    print('SIGINT')
  finally:
    for ser in sers:
      ser.ser.close()
    
