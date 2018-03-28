#!/usr/bin/env python

import sys
import os
import math
import time
import serial
import lib_controller


if __name__ == "__main__":
  ser = None
  try:
    ser = lib_controller.MySerial("/dev/ttyACM0", 9600)
    ser.wait_start('central_mega') # 'controller_pos_velo'):

    #cmd = 'l-2.3 4 4.1 '
    #ser.print_checksum(cmd)
    for i in range(5):
      ser.set_limit( i, False )
      ser.set_gear_microstep( i, False )
    ser.set_target( 1.02, [1.14, 2.04, 3.04, 4.04, 5.04], False);
    while 1:
      s = ser.serial_read()
      #if len(s)>0:
      #  break
      time.sleep(0.01)
  finally:
    if ser is not None:
      ser.ser.close()
