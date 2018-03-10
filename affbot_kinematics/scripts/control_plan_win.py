#!/usr/bin/env python

import sys
import os
import math
import time
import serial
import lib_controller
import convert_motor_angle

motor_id = 0
b_1_motor = True
ser = None

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

    ser = lib_controller.MySerial("COM6", 57600)
    if b_1_motor:
      ser.wait_start('controller_pos_velo')
      ser.set_limit(motor_id, True)
      ser.set_gear_microstep(motor_id, True)
    else:
      ser.wait_start('central_mega')
      for i in range(5):
        ser.set_limit(i, False)
        ser.set_gear_microstep(i, False)

    t_prev = 0
    max_stack = 10
    print('control start')

    with open('plan_result.txt', 'wt') as fw:
      t_start = time.time()
      for i in range(len(points)):
        p = points[i]
        if i>0:
          t = p[0] - points[i-1][0]
        else:
          t = p[0]
        for i in range(5):
          p[i+1]+= lib_controller.q_start[i]
        #cmd = 'p%.3f %.3f %.3f ' % (p[1], p[6], p[0])# - t_prev)
        #cmd = 'p%.3f %.3f ' % (p[0], p[motor_id + 1] )# - t_prev)
        #ser.write((cmd + '\n').encode('ascii','ignore'))
        #q_motor = lib_controller.joint2motor(p[1:6])
        #ser.print_checksum('p%.3f %.3f ' % ( t, q_motor[motor_id]))
        ser.set_target(t, p[1:6], b_1_motor)
        t_prev= p[0]
        # print(cmd)
        if i>=max_stack-1:
          while time.time()-t_start < points[i-max_stack+1][0]:
            record_result(fw, ser.serial_read(), time.time()-t_start)
            time.sleep(0.01)
        else:
          record_result(fw, ser.serial_read(), time.time()-t_start)
      while 1:
        time.sleep(0.01)
        t = record_result(fw, ser.serial_read(), time.time()-t_start)
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
    if ser is not None:
      ser.ser.close()
