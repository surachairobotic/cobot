#!/usr/bin/env python

import sys
import os
import math
import rospy
import copy
import time

import matplotlib.pyplot as plt
import numpy as np

file_path = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/results/"
  
if __name__ == '__main__':
  rospy.init_node('init_torque', anonymous=True)
    
#  file_all = "traj_const_velo.txt"
#  data_all = open(file_path + file_all, "r").read().splitlines()
#  tq1 = []
#  tq2 = []
  
  s = load(file_all);
  t = s[:,1];
  q = s[:,2:7];
  dq = s[:,8:13];
  ddq = s[:,14:19];
#  %eff = s(:,20:25);

  x = s[:,20:22];
  o = s[:,23:26];
  dx = s[:,27:32];

  figure(k*2-1),subplot(3,1,1),plot(t,q[:,1],'*-',t,q[:,2],'*-',t,q[:,3],'*-',t,q[:,4],'*-',t,q[:,5],'*-',t,q[:,6],'*-');
#  grid on;
  ylabel('angle(rad)');
  xlabel('time(s)');
  legend("qt1","qt2","qt3","qt4","qt5","qt6");
  figure(k*2-1),subplot(3,1,2),plot(t,dq[:,1],'*-',t,dq[:,2],'*-',t,dq[:,3],'*-',t,dq[:,4],'*-',t,dq[:,5],'*-',t,dq[:,6],'*-');
#  grid on;
  ylabel('ang velo(rad/s)');
  xlabel('time(s)');
  legend("qt1","qt2","qt3","qt4","qt5","qt6");
  figure(k*2-1),subplot(3,1,3),plot(t,ddq[:,1],'*-',t,ddq[:,2],'*-',t,ddq[:,3],'*-',t,ddq[:,4],'*-',t,ddq[:,5],'*-',t,ddq[:,6],'*-');
#  grid on;
  ylabel('ang acc(rad/s2)');
  xlabel('time(s)');
 

  figure(k*2),subplot(3,1,1),plot(t,x[:,1],'*-',t,x[:,2],'*-',t,x[:,3],'*-');
#  grid on;
  ylabel('xyz(m)');
  xlabel('time(s)');
  legend("qt1","qt2","qt3","qt4","qt5","qt6");
  figure(k*2),subplot(3,1,2),plot(t,dx[:,1],'*-',t,dx[:,2],'*-',t,dx[:,3],'*-');
#  grid on;
  ylabel('velo (m/s)');
  xlabel('time(s)');
  legend("qt1","qt2","qt3","qt4","qt5","qt6");
  figure(k*2),subplot(3,1,3),plot(t,o[:,1],'*-',t,o[:,2],'*-',t,o[:,3],'*-',t,o[:,4],'*-');
#  grid on;
  ylabel('quaternion');
  xlabel('time(s)');

#  plt.show()
  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

