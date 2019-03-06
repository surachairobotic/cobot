#!/usr/bin/env python
# -*- coding: utf-8 -*-

# lib for calibrate_chess.py

import sys

sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')
sys.path.insert(0,'/home/tong/catkin_ws/src/cobot/cobot_planner/scripts')

import os
from time import sleep
import math
import numpy as np
import time
import eq
import set_const
import copy
import multiprocessing as mp
from calibrate_chess import get_xyz


def rx(q):
  c = math.cos(q)
  s = math.sin(q)
  return np.array([
    [1, 0, 0],
    [0, c,-s],
    [0, s, c],
  ])

def ry(q):
  c = math.cos(q)
  s = math.sin(q)
  return np.array([
    [c, 0, s],
    [0, 1, 0],
    [-s, 0, c],
  ])

def rz(q):
  c = math.cos(q)
  s = math.sin(q)
  return np.array([
    [c,-s, 0],
    [s, c, 0],
    [0, 0, 1],
  ])


def get_R(off_bq):
  Rx = rx( off_bq[0] )
  Ry = ry( off_bq[1] )
  return Ry*Rx


'''
def find_err_all_thread(pos, xyz_ref, off_q, off_x, dq, dqmax, dx, dxmax, data, results):
  best_score = 999999999.0
  xyz = np.array([0.0]*3)
  
  cnt = 0
  for d in data:
    cnt+=1
    score = 0.0
    bq = d[0:3]
    q = d[3:9]
    xyz = d[9:12]
    for i in range(len(pos)):
      for p2 in pos[i]:
        e = get_xyz(p2[0:6]+q) + xyz - xyz_ref[i]
        for j in range(3):
          score+= e[j]**2
    score = math.sqrt( score/(len(pos)*len(pos[0])) )
    if score < best_score:
      best_score = score
      best_q = copy.deepcopy(q)
      best_xyz = copy.deepcopy(xyz)
      progess =  cnt / float(len(data))
      print(('%.2f : %.3f : ' % (progess, best_score)) + str(best_q)
        + ', ' + str(best_xyz))
  print('end')
  results.put([best_q, best_xyz, best_score])
'''

def find_err_all_thread(pos, xyz_ref, off_q, off_x, off_bq, dq, dqmax, dx, dxmax, dbq, dbqmax, thread_range, results):
  global b_exit
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
  nbq = int(dbqmax*2 / dbq)
  nx = int(dxmax*2 / dx)
  if nq==0:
    nq = 1
  if nbq==0:
    nbq = 1
  if nx==0:
    nx = 1

  
  cnt = 0
  n_all = 1.0/float( (thread_range[1]-thread_range[0])*nq**4*nx**3*nbq**2)
  param = np.array([0.0]*12)
  bq = param[0:3]
  q = param[3:9]
  xyz = param[9:12]
  with open('calib_result_%d_%d.txt' % (thread_range[0], thread_range[1]), 'at') as f:
    for bqx in range(nbq):
      bq[0] = dbq*bqx - dbqmax + off_bq[0]
      Rx = rx( bq[0] )
      for bqy in range(nbq):
        bq[1] = dbq*bqy - dbqmax + off_bq[1]
        Ry = ry( bq[1] )
        R = Ry*Rx
        for q1 in range(thread_range[0], thread_range[1]):
          q[0] = dq*q1 - dqmax + off_q[0]
          for q2 in range(nq):
            q[1] = dq*q2 - dqmax + off_q[1]
            for q3 in range(nq):
              q[2] = dq*q3 - dqmax + off_q[2]
              for q4 in range(nq):
                q[3] = dq*q4 - dqmax + off_q[3]
                for q5 in range(nq):
                  q[4] = dq*q5 - dqmax + off_q[4]
                  for x in range(nx):
                    xyz[0] = dx*x - dxmax + off_x[0]
                    for y in range(nx):
                      xyz[1] = dx*y - dxmax + off_x[1]
                      for z in range(nx):
                        xyz[2] = dx*z - dxmax + off_x[2]
                        cnt+=1
                        score = 0.0
                        for i in range(len(pos)):
                          for p2 in pos[i]:
                            e = R.dot( get_xyz(p2[0:6]+q) ) + xyz - xyz_ref[i]
                            for j in range(3):
                              score+= e[j]**2
                        score = math.sqrt( score/(len(pos)*len(pos[0])) )
                        if score < best_score:
                          best_score = score
                          best_q = copy.deepcopy(q)
                          best_xyz = copy.deepcopy(xyz)
                          best_bq = copy.deepcopy(bq)
                          s = ('[%d:%d] %.2f : %.3f : ' % (thread_range[0],thread_range[1], cnt*n_all, best_score)) + str(param)
                          print(s)
                          f.write( s + '\n' )
        print('prog [%d:%d] %.2f' % (thread_range[0],thread_range[1], cnt*n_all))
#  results.append( [best_q, best_xyz, best_score] )
  print('[%d:%d] end' % (thread_range[0],thread_range[1]))
  results.put([best_q, best_xyz, best_bq, best_score])


def find_err_all_multithread( pos, xyz_ref, off_q, off_x, off_bq, dq, dqmax, dx, dxmax, dbq, dbqmax):
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
  nbq = int(dbqmax*2 / dbq)
  nx = int(dxmax*2 / dx)
  if nq==0:
    nq = 1
  if nbq==0:
    nbq = 1
  if nx==0:
    nx = 1
  n_thread = 8
  if n_thread>nq:
    n_thread = nq
  n = 0
  ranges = []
  for i in range(n_thread):
    n1 = int(math.ceil( (nq-n) / float(n_thread-i) ))
    ranges.append([n,n+n1])
    n+= n1
  assert( ranges[-1][1]==nq )
  ths = []
  results = []
#  mp.set_start_method('spawn')
  queues = []
  for r in ranges:
    queue = mp.Queue()
    ths.append( mp.Process(target=find_err_all_thread
      , args=([pos, xyz_ref, off_q, off_x, off_bq, dq, dqmax, dx, dxmax, dbq, dbqmax, r, queue])) )
    queues.append(queue)

  for th in ths:
    th.start()
#  for th in ths:
#    th.join()
  while len(results)!=n_thread:
    for q in queues:
      r = q.get()
      if r:
        results.append(r)
    time.sleep(1.0)
  best_r = None
  for r in results:
    print(r)
    if best_r is None:
      best_r = r
    else:
      if best_r[3]>r[3]:
        best_r = r
#  print(best_r)
  print('score : '+str(best_r[3]))
  return best_r[0:3]



