#!/usr/bin/env python

import sys
import os
import math
import time
import serial

serial_buf = ''
def serial_read(ser):
  global serial_buf
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

def wait_start(ser):
  print('wait start')
  while 1:
      if serial_read(ser)=='start':
        print('start')
        return

def record_result(f, msg, t):
  if len(msg)<2 or msg[0]!='<' or msg[-1]!='>':
    return None
  arr = msg[1:-1].split(' ')
  if len(arr)!=4:
    print('invalid data : '+msg)
    return None
  f.write(msg[1:-1] + '\n')
  return float(arr[0])


if __name__ == "__main__":

  try:

    val_num = 17
    points = []
    with open('plan2.txt', 'rt') as f:
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

    print(len(points))
    print(points[-1][0])
    

    ser = serial.Serial()
#    ser.port = "COM5"
    ser.port = "/dev/ttyACM0"
    ser.baudrate = 57600
    ser.timeout = 0.001
    ser.writeTimeout = 1
    ser.open()
    wait_start(ser)
    t_prev = 0
    max_stack = 10
    with open('plan_result.txt', 'wt') as fw:
      t_start = time.time()
      for i in range(len(points)):
        p = points[i]
        p[0]*=1
        # print('t = {0}, pos = {1}'.format(p[0], p[1]))
        t = p[0]# - 0.05
        '''
        while t > (time.time() - t_start):
          serial_read(ser)
          time.sleep(0.001)
        '''
        #cmd = 'p%.3f %.3f %.3f ' % (p[1], p[6], p[0])# - t_prev)
        cmd = 'p%.3f %.3f ' % (p[1], p[0])# - t_prev)
        t_prev= p[0]
        ser.write((cmd + '\n').encode('ascii','ignore'))
        # print(cmd)
        if i>=max_stack-1:
          while time.time()-t_start < points[i-max_stack+1][0]:
            record_result(fw, serial_read(ser), time.time()-t_start)
            time.sleep(0.01)
        else:
          record_result(fw, serial_read(ser), time.time()-t_start)
      while 1:
        time.sleep(0.01)
        t = record_result(fw, serial_read(ser), time.time()-t_start)
        if t is not None and t > points[-1][0] + 0.5:
          break
    print('end')
    '''
    while 1:
      serial_read(ser)
      time.sleep(0.001)
    '''
  except KeyboardInterrupt:
    print('SIGINT')
  finally:
    ser.close()
