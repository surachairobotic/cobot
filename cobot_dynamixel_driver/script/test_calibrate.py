#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

import math
import numpy as np
import time
import copy
import fk
import calibrate


q = np.array([0.0, -0.0164058,  -0.00994873,  0.02879799, -0.00069813, -0.0])
pos = calibrate.load_pos('pos_calib_toe.txt')

print('off : '+str(q*(180.0/math.pi)))
rb_pos = np.array([0.0]*6)
score = 0.0
for j in range(len(pos)):
  print('-- %d --' % (j))

  pos2 = []
  for k in range(len(pos[j])):
    for l in range(6):
      rb_pos[l] = pos[j][k][l] + q[l]
    pos2.append(fk.fk(rb_pos))
  pos2 = np.array(pos2)
  # cal score
  mean = [0.0,0.0,0.0]
  for k in range(len(pos2)):
    for l in range(3):
      mean[l]+= pos2[k][l]
  for l in range(3):
    mean[l]/= float(len(pos2))
  sc = 0
  for k in range(len(pos2)):
    for l in range(3):
      sc+= (pos2[k][l] - mean[l])**2
    print(math.sqrt(sc))
  score+= sc / len(pos2)

print('score : '+str(score))
