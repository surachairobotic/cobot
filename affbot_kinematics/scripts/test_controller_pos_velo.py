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
    ser = lib_controller.MySerial("COM6", 57600)
    ser.wait_start('controller_pos_velo')

    ser.print_checksum('reset')
#    ser.print_checksum('p1.02 0.3 ')
#    ser.print_checksum('l1.0 -3.14 3.14 ')
    ser.print_checksum('g1.0 1600 ')
    while 1:
      s = ser.serial_read()
      #if len(s)>0:
      #  break
      time.sleep(0.01)
  finally:
    if ser is not None:
      ser.ser.close()
