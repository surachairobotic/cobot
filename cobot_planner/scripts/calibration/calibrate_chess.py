#!/usr/bin/env python
# -*- coding: utf-8 -*-

# calibrate robot's joints using positions of chessboard

import sys

sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')
sys.path.insert(0,'/home/tong/catkin_ws/src/cobot/cobot_planner/scripts')

'''
import rospy
from sensor_msgs.msg import JointState
'''
import os
from time import sleep
import math
import numpy as np
import time
import eq
import set_const
import copy
import matplotlib.pyplot as plt
import multiprocessing as mp
from find_err_multithread import *
from find_q2q5_multithread import find_q2q5_multithread
from find_q1xyz_multithread import find_q1xyz_multithread

#fname = 'chess-190121.txt' # 'pos_calib.txt'
fname_ref = 'chess-ref-190121.txt' # 'pos_calib.txt'
P6 = np.array( set_const.L[6]+[1.0])
b_exit = False

compute_fk = None

'''
def find_err(pos, off, dq, dqmax):
  best_score = 999999999.0
  n = int(dqmax*2 / dq)
  q = np.array([0.0]*len(off))
  with open('calib_result.txt', 'at') as f:
#    for q1 in range(n):
#      print('q1 : ' + str(q1))
#      q[0] = dq*q1 - dqmax + off[0]
    for q2 in range(n):
      q[1] = dq*q2 - dqmax + off[1]
      for q3 in range(n):
        q[2] = dq*q3 - dqmax + off[2]
        for q4 in range(n):
          q[3] = dq*q4 - dqmax + off[3]
          for q5 in range(n):
            q[4] = dq*q5 - dqmax + off[4]
#            for q6 in range(n):
#              q[5] = dq*q6 - dqmax + off[5]
            score = 0
            for p1 in pos:
              pos2 = []
              for p2 in p1:
                pos2.append(get_xyz(p2[0:6]+q))
                #for l in range(6):
                #  rb_pos[l] = p2[l] + q[l]
                #pos2.append(get_xyz(rb_pos))
              pos2 = np.array(pos2)
              score+= np.var(pos2[:,0]) + np.var(pos2[:,1]) + np.var(pos2[:,2])
            if score < best_score:
              best_score = score
              best_q = copy.deepcopy(q)
              progess = (q5*n + q4*n + q3*n**2 + q2*n**3) / float(n**4)
              print(str(progess) + ' : ' + str(best_score) + ' : ' + str(best_q))
              f.write( '%.8f %f %f %f %f %f %f\n' % (best_score \
                , q[0], q[1], q[2], q[3], q[4], q[5]))
  return best_q




def find_err_q1_pos( pos, xyz_ref, off_q, off_x, dq, dqmax, dx, dxmax):
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
  nx = int(dxmax*2 / dx)
  q = copy.deepcopy(off_q)
  xyz = np.array([0.0]*3)
  with open('calib_result.txt', 'at') as f:
    for q1 in range(nq):
      q[0] = dq*q1 - dqmax + off_q[0]
      for x in range(nx):
        xyz[0] = dx*x - dxmax + off_x[0]
        for y in range(nx):
          xyz[1] = dx*y - dxmax + off_x[1]
          for z in range(nx):
            xyz[2] = dx*z - dxmax + off_x[2]
            score = 0.0
            for i in range(len(pos)):
              for p2 in pos[i]:
                e = get_xyz(p2[0:6]+q) + xyz - xyz_ref[i]
                for j in range(3):
                  score+= e[j]**2
            score/= len(pos) * len(pos[0]) * 3
            if score < best_score:
              best_score = score
              best_q = copy.deepcopy(q)
              best_xyz = copy.deepcopy(xyz)
              progess = (z + y*nx + x*nx**2 + q1*nx**3) / float(nq*nx**3)
              print(str(progess) + ' : ' + str(best_score) + ' : ' + str(best_q)
                + '\n  ' + str(best_xyz))
              f.write( '%.8f %f, %f, %f, %f, %f, %f, %f, %f, %f\n' % (best_score \
                , q[0], q[1], q[2], q[3], q[4], q[5], xyz[0], xyz[1], xyz[2]))
  return best_q, best_xyz
           

def find_err_all( pos, xyz_ref, off_q, off_x, dq, dqmax, dx, dxmax):
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
  nx = int(dxmax*2 / dx)
  if nq==0:
    nq = 1
  if nx==0:
    nx = 1
  q = copy.deepcopy(off_q)
  xyz = np.array([0.0]*3)
  with open('calib_result.txt', 'at') as f:
    for q1 in range(nq):
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
                    score = 0.0
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
                      progess = (z + y*nx + x*nx**2 + q5*nx**3 + q4*nx**3*nq + q3*nx**3*nq**2 + q2*nx**3*nq**3 + q1*nx**3*nq**4) / float(nq**5*nx**3)
                      print(str(progess) + ' : ' + str(best_score) + ' : ' + str(best_q)
                        + '\n  ' + str(best_xyz))
                      f.write( '%.8f %f, %f, %f, %f, %f, %f, %f, %f, %f\n' % (best_score \
                        , q[0], q[1], q[2], q[3], q[4], q[5], xyz[0], xyz[1], xyz[2]))
  return best_q, best_xyz

'''

def load_data(fname):
  pos = []
  with open(fname, 'rt') as f:
    p = []
    while True:
      s = f.readline().strip()
      if not s:
        break
      elif s=='--':
        if len(p)>0:
          pos.append(p)
          p = []
      else:
        a = s.split(' ')
        if len(a)!=6:
          print('Invalid string : ' + s)
          exit()
        p.append(np.array([float(b) for b in a]))
    if len(p)>0:
      pos.append(p)
  return pos



def load_data_chess(fname):
  pos = []
  with open(fname, 'rt') as f:
    p = []
    while True:
      s = f.readline()
      if not s:
        break
      s = s.strip()
      a = s.split(',')
      if len(a)<=1:
        if len(p)>0:
          pos.append(p)
        p = []
        continue
      elif len(a)!=13:
        print('invalid data : '+s+' / '+str(len(a)))
        exit()
      p2 = np.array([float(a[i]) for i in range(9)])
      
      b_0 = True
      for p3 in p2[0:6]:
        if abs(p3)>0.001:
          b_0 = False
          break
      if b_0:
        pos.append(p)
        p = []
      else:
        # check xyz
        xyz = [float(a[i]) for i in range(6, 9)]
        xyz2 = get_xyz(np.array(p2))
        if np.linalg.norm( xyz-xyz2[0:3] )>0.00001:
          print('Too large xyz error')
          print('xyz : '+str(xyz)+'\nxyz2 : '+str(xyz2))
          exit()
        p.append(p2)
        if len(p)==5:
          pos.append(p)
          p = []
    if len(p)>0:
      pos.append(p)
      
  n = None
  for p in pos:
    if n is None:
      n = len(p)
    else:
      if len(p)!=n:
        print('sample num is not the same : %d / %d'%(n, len(p)))
        exit()
  print('sample num : '+str(n))
  return pos

def load_ref_chess(fname):
  pos = []
  with open(fname, 'rt') as f:
    while True:
      s = f.readline()
      if not s:
        break
      s = s.strip()
      if len(s)==0 or s[0]=='#':
        continue
      a = s.split(',')
      if len(a)>=3:
        pos.append(np.array([float(a[i]) for i in range(3)]))
  return pos


def print_pos(pos):
  for i in range(len(pos)):
    print('-- %d --' % (i))
    for p in pos[i]:
      print(p[0:6])


def get_xyz(q):
  global P6
  R = eq.get_R(q)
  return R[5].dot(P6)[0:3]


def test_off(pos, off_q):
  print('test off')
  max_err = [0.0]*2
  mean_err = [0.0]*2
  
  n = 0
  for i in range(len(pos)):
    # find real xyz
    n+= len(pos[i])*3
    print('-- %d --' % (i))
    off_q2 = [np.array([0.0]*6), off_q]
    for j in range(2):
      if j==0:
        print('-- no off --')
      else:
        print('-- with off --')
      sum_x = np.array([0.0]*3)
      for p2 in pos[i]:
        sum_x+= get_xyz(p2[0:6] + off_q2[j])
      mean_x = sum_x / len(pos[i])
      for p2 in pos[i]:
        x = get_xyz(p2[0:6] + off_q2[j])
        print( ('%.4f' % (np.linalg.norm(x-mean_x))) + ' : '+str(x - mean_x))
        for k in range(3):
          err = abs(x[k] - mean_x[k])
          if  err>max_err[j]:
            max_err[j] = err
          mean_err[j]+= err
  for i in range(2):
    mean_err[i]/= n
    print('max_err : %f, mean_err : %f' % (max_err[i], mean_err[i]))
  exit()


def test_off_qx(pos, xyz_ref, off_q, off_x, off_bq):
  print('test off')
  max_err = [0.0]*2
  mean_err = [0.0]*2
  
  n = 0
  for i in range(len(pos)):
    # find real xyz
    n+= len(pos[i])
    print('-- %d --' % (i))
    off_q2 = [np.array([0.0]*6), off_q]
    off_x2 = [np.array([0.0]*3), off_x]
    R = [get_R([0.0,0.0]), get_R( off_bq )]
    for j in range(2):
      if j==0:
        print('-- no off --')
      else:
        print('-- with off --')
      for p2 in pos[i]:
        x = R[j].dot(get_xyz(p2[0:6] + off_q2[j])) + off_x2[j]
#        print(x - xyz_ref[i])
        dx = x-xyz_ref[i]
        print( ('%.4f' % (np.linalg.norm(dx))) + ' : '+str(dx))
        err = 0.0
        for k in range(3):
          err+= (x[k] - xyz_ref[i][k])**2
        err = math.sqrt(err)
        if  err>max_err[j]:
          max_err[j] = err
        mean_err[j]+= err
  for i in range(2):
    mean_err[i]/= n
    print('max_err : %f, mean_err : %f' % (max_err[i], mean_err[i]))
  exit()


def check_pos_ref(pos, xyz_ref):
  if len(pos)!=len(xyz_ref):
    print('pos ref size does not match : %d / %d' % (len(pos), len(xyz_ref)))
    exit()
  
  max_err = 0.0
  for i in range(len(pos)):
    for p in pos[i]:
      xyz2 = get_xyz(np.array(p))
      e = np.linalg.norm( xyz_ref[i]-xyz2[0:3] )
      print('[%d] : %.4f' % (i, e))
      if e>0.05:
        print('[%d] Too large xyz error : %f' % (i, e))
        print('ref : '+str(xyz_ref[i])+'\nxyz : '+str(xyz2[0:3]))
        exit()
      if e > max_err:
        max_err = e
  print('check_pos_ref OK : max_err = %f' % (max_err))


def plot_pos(pos, ref, off_q, off_x, off_bq):
  xyz = []
  R = get_R( off_bq )
  for i in range(len(pos)):
    xyz2 = []
    for j in range(len(pos[i])):
      p = pos[i][j]
      xyz2.append( R.dot(get_xyz(p[0:6]+off_q)) + off_x )
    xyz.append(xyz2)
  cols = 'rgbmy'
  axis_name = 'XYZ'
  for k in range(3):
    h = None
    n_col = 0
    
    plt.figure(k+1)
    plt.hold(True)
    plt.grid(True)
    if k==0:
      nx = 0
      ny = 1
    elif k==1:
      nx = 1
      ny = 2
    else:
      nx = 0
      ny = 2
    plt.xlabel(axis_name[nx] + ' (m)')
    plt.ylabel(axis_name[ny] + ' (m)')

    for i in range(len(pos)):
      if h is None:
        h = ref[i][2]
        n_col = 0
      else:
        if h!=ref[i][2]:
          h = ref[i][2]
          n_col+=1
      x = [a[nx] for a in xyz[i]]
      y = [a[ny] for a in xyz[i]]
      plt.plot( x, y, cols[n_col] + '+')
      plt.plot( ref[i][nx], ref[i][ny], 'k+')
  plt.show()


if __name__ == "__main__":
  try:
    #pos = load_data(fname)
    deg2rad = math.pi / 180.0
    pos = []
    xyz_ref = []
    
    old_off_1 = np.array([0.0, -0.016406, -0.009949, 0.028798, -0.000698, 0.0])
    old_off_2 = np.array([-0.04363323,  0.038048,-0.00384,-0.036303,0.011519,0.0])
    for level in ['down', 'mid','up']:
#    for level in ['down', 'mid', 'up']:
      pos2 = load_data_chess('./190308/chess_'+level+'.txt')
      xyz_ref+= load_ref_chess('./190308/chess-ref_'+level+'.txt')
      '''
      if level=='down':
        for p1 in pos:
          for p2 in p1:
            p2[0:6]-= old_off_1
      else:
      '''
      for p1 in pos2:
        for p2 in p1:
          p2[0:6]-= old_off_1 + old_off_2
      pos+= pos2
          
#    print_pos(pos)
    check_pos_ref( pos, xyz_ref )
    
    off_q = np.array([0.0]*6)
    off_x = np.array([0.0]*3)
    off_bq = np.array([0.0]*3)

    
    # plot off
    '''
    off_q = np.array([-0.038397, 0.010472, -0.012218, -0.008727, 0.008727, 0.000000])
    off_bq = np.array([0.003491, 0.013963, 0.000000])
    plot_pos(pos, xyz_ref, off_q, off_x, off_bq)
    exit()
    '''
    
    '''
    print(xyz_ref)
    print(pos)
    exit()
    '''
    
    # test off_q


    '''
    off_q = np.array([0.000000, -0.008727, -0.017453, 0.000000, 0.017453, 0.000000])
    test_off(pos, off_q)
    exit()
    '''
    
    # test off_q, off_x, off_bq
    off_q = np.array([-0.038397, 0.010472, -0.012218, -0.008727, 0.008727, 0.000000])
    off_bq = np.array([0.003491, 0.013963, 0.000000])
    test_off_qx(pos, xyz_ref, off_q, off_x, off_bq)
    exit()
    
    
    # find q2-q5
    '''
    off_q = np.array([0.000000, 0.010472, -0.012218, -0.008727, 0.008727, 0.000000])
    off_q = find_q2q5_multithread(get_xyz, pos, off_q, deg2rad*0.2, deg2rad*1.0)
    '''
    
    
    # find q1, xyz, bq
    '''
    off_q = np.array([0.000000, 0.010472, -0.012218, -0.008727, 0.008727, 0.000000])
#    off_x = np.array([0.0006, 0.0002, -0.001 ])
    off_q, off_x, off_bq = find_q1xyz_multithread( get_xyz, pos, xyz_ref, off_q, off_x, off_bq
      , deg2rad*0.2, deg2rad*5.0
      , 0.002, 0.02*0
      , deg2rad*0.2, deg2rad*1.0)
    '''
    
    
    # find all
    '''
    off_q = np.array([-0.038397, 0.010472, -0.012218, -0.008727, 0.008727, 0.000000])
    off_bq = np.array([0.003491, 0.013963, 0.000000])
    off_q, off_x, off_bq = find_err_all_multithread( get_xyz, pos, xyz_ref, off_q, off_x, off_bq
      , deg2rad*0.25, deg2rad*0.5
      , 0.0005, 0.001
      , deg2rad*0.025, deg2rad*0.5 )
    '''
    
    print('q : [%f, %f, %f, %f, %f, %f]' % (off_q[0],off_q[1],off_q[2],off_q[3],off_q[4],off_q[5]))
    print(off_q * (180.0/3.14))
    print(off_x)
    print('x : [%f, %f, %f]' % (off_x[0],off_x[1],off_x[2]))
    print('b : [%f, %f, %f]' % (off_bq[0],off_bq[1],off_bq[2]))
  except KeyboardInterrupt:
    print('SIGINT')
    b_exit = True
