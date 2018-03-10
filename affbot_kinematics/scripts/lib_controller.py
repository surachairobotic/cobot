#!/usr/bin/env python

import sys
import os
import math
import time
import serial

#!/usr/bin/env python

import math

'''
GEAR_RATIO = [1600.0/(400.0*100)
 ,1600.0/(400.0*88.0)
 ,1600.0/(400.0*37.5)
 ,1600.0/(400.0*2.0)
 ,1600.0/(400.0*2.0)]
q_start = [90.0 * math.pi/180.0
  , -15.4 * math.pi/180.0
  , 60 * math.pi/180.0
  , -200 * math.pi/180.0
  , 0 * math.pi/180.0 ]
'''

deg2rad = math.pi/180

GEAR_RATIO = [100.0, 88.0, 37.5, 2.0, 2.0]
MICROSTEP = [1600, 1600, 1600, 1600, 1600]
MOTOR_MIN_ANG = [0, 0, -150*deg2rad, 0, -360*deg2rad]
MOTOR_MAX_ANG = [ 180*deg2rad,  120*deg2rad, 0,  260*deg2rad,  360**deg2rad]
MOTOR_MAX_VELO = [ 60*deg2rad,  60**deg2rad, 60*deg2rad, 90*deg2rad, 90*deg2rad]

q_start = [90.0 * math.pi/180.0
  , -15.4 * math.pi/180.0
  , 60 * math.pi/180.0
  , -200 * math.pi/180.0
  , 0 * math.pi/180.0 ]


class MySerial():
  def __init__(self, _port_name, _baud_rate):
    self.ser = serial.Serial()
#    ser.port = "COM5"
    self.ser.port = _port_name
    self.ser.baudrate = _baud_rate
    self.ser.timeout = 0.001
    self.ser.writeTimeout = 1
    self.ser.open()
    self.serial_buf = ''

  def wait_start(self, name):
    print('wait start')
    for i in range(5):
      self.ser.write(b'who\n')
      t = time.time() + 1
      while time.time() < t:
        s = self.serial_read()
        if s[0:4]=='who:':
          _name = s[4:].split(' ')[0]
          if (type(name) is str and name==_name) or (type(name) is list and (_name in name) ):
            print('start')
            return
          else:
            raise Exception('wait_start() : wrong name : ' + _name + ' / ' + str(name))
    raise Exception('wait_start() : timeout')

  def serial_read(self):
    c = self.ser.read()
    while len(c)>0:
      if ord(c)<128:
        c = c.decode()
        if c=='\n' or c=='\r':
          if len(self.serial_buf)>0:
            b = self.serial_buf
            self.serial_buf = ''
            print('READ : ' + b)
            return b
        else:
          self.serial_buf+= c
      c = self.ser.read()
    return ''

  def print_checksum(self, msg):
    sum = 0
    for i in range(len(msg)):
      sum+= ord(msg[i])
    cs = (sum & 127) + ord('0')
    msg2 = bytes(msg.encode('ascii','ignore')) + bytes([cs, ord('\n')])
    self.ser.write(msg2)

  def set_gear_microstep(self, id, b_1_motor, gear=None, microstep=None):
    if gear is None:
      gear = GEAR_RATIO[id]
    if microstep is None:
      microstep = MICROSTEP[id]
    if b_1_motor:
      self.print_checksum('g%.3f %d ' % (gear, microstep))
    else:
      self.print_checksum('g%d %.3f %d ' % (id, gear, microstep))
    for i in range(5):
      t = time.time() + 1
      while time.time()<t:
        s = self.serial_read()
        if len(s)>2 and s[:2]=='g:':
          return
    raise Exception('set_gear_microstep() : timeout')

  def set_limit(self, id, b_1_motor, velo=None, min_ang=None, max_ang=None):
    if velo is None:
      velo = MOTOR_MAX_VELO[id]
    if min_ang is None:
      min_ang = MOTOR_MIN_ANG[id]
    if max_ang is None:
      max_ang = MOTOR_MAX_ANG[id]
    if b_1_motor:
      self.print_checksum('l%.3f %.3f %.3f ' % (velo, min_ang, max_ang))
    else:
      self.print_checksum('l%d %.3f %.3f %.3f ' % (id, velo, min_ang, max_ang))
    for i in range(5):
      t = time.time() + 1
      while time.time()<t:
        s = self.serial_read()
        if len(s)>2 and s[:2]=='l:':
          return
    raise Exception('set_limit() : timeout')

  def set_target(self, time, positions, b_1_motor):
    q_motor = joint2motor(positions)
    if b_1_motor:
      cmd = 'p%.3f %.3f ' % (time, q_motor[0])
    else:
      cmd = 'p%.3f ' % (time)
      for p in positions:
        cmd+= '%.3f ' % (p)

    self.print_checksum(cmd)
    print('set_target() : ' + cmd)


def motor2joint(q_motor, add_q_start=True):
  global GEAR_RATIO, q_start
  if len(q_motor)!=5:
    raise Exception('motor2joint : joint num is not 5 (%d)' % (len(q_motor)))
  q = [0,0,0,0,0]
  q[0] = q_motor[0]
  q[1] = q_motor[1]
  q[2] = q_motor[2] - q[1]
  q[3] = (q_motor[3] + q_motor[4]) * 0.5 - q[1] - q[2]
  q[4] = (q_motor[3] - q_motor[4]) * 0.5
  '''
  q[0] = q_motor[0] / GEAR_RATIO[0]
  q[1] = q_motor[1] / GEAR_RATIO[1]
  q[2] = q_motor[2] / GEAR_RATIO[2] - q[1]
  q[3] = (q_motor[3] + q_motor[4]) / (2.0 * GEAR_RATIO[3] ) - q[1] - q[2]
  q[4] = (q_motor[3] - q_motor[4]) / (2.0 * GEAR_RATIO[4] )
  '''
  if add_q_start:
    for i in range(5):
      q[i]+= q_start[i]
  return q


def joint2motor(q):
  global GEAR_RATIO, q_start

  q_link = [0,0,0,0,0]
  for i in range(5):
    q_link[i] = q[i] - q_start[i]

  q_motor = [0,0,0,0,0]
  '''
  q_motor[0] = q_link[0] * GEAR_RATIO[0]
  q_motor[1] = q_link[1] * GEAR_RATIO[1]
  q_motor[2] = (q_link[1] + q_link[2]) * GEAR_RATIO[2]
  q_motor[3] = (q_link[1] + q_link[2] + q_link[3] + q_link[4]) * GEAR_RATIO[3]
  q_motor[4] = (q_link[1] + q_link[2] + q_link[3] - q_link[4]) * GEAR_RATIO[4]
  '''
  q_motor[0] = q_link[0]
  q_motor[1] = q_link[1]
  q_motor[2] = (q_link[1] + q_link[2])
  q_motor[3] = (q_link[1] + q_link[2] + q_link[3] + q_link[4])
  q_motor[4] = (q_link[1] + q_link[2] + q_link[3] - q_link[4])
  return q_motor
