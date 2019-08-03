#!/usr/bin/env python

'''

save data realsense

'''

import sys
import os
import rospy
import tf
import tf2_ros
import time
import pickle
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


t_start = None
filename = 'realsense.pkl'
b_lock = False

def cb(msg, name):
  global topics, t_start, filename, b_lock
  if t_start is None:
    t_start = time.time()
    return
  
  if time.time()-t_start<1.0:
    return
  
  if topics[name].get('data') is None:
    topics[name]['data'] = msg
    print(name+' OK')
  b_ok = True
  for name in topics:
    t = topics[name]
    if t.get('data') is None:
      b_ok = False
      break
  if b_ok:
    if b_lock:
      return
    b_lock = True
    for name in topics:
      t = topics[name]
      t['sub'].unregister()
      del t['sub']
      del t['cb']
    with open(filename, 'wb') as f:
      pickle.dump(topics, f)
    

def cb_depth_info(msg):
  cb(msg, 'depth_info')
  
def cb_depth_image(msg):
  cb(msg, 'depth_image')
  
def cb_color_info(msg):
  cb(msg, 'color_info')
  
def cb_color_image(msg):
  cb(msg, 'color_image')
  
def cb_pointcloud(msg):
  cb(msg, 'pointcloud')


topics = {'depth_info': {'topic': '/camera/aligned_depth_to_color/camera_info', 'type': CameraInfo, 'cb': cb_depth_info},
'depth_image': {'topic': '/camera/aligned_depth_to_color/image_raw', 'type': Image, 'cb': cb_depth_image},
'color_info': {'topic': '/camera/color/camera_info', 'type': CameraInfo, 'cb': cb_color_info},
'color_image': {'topic': '/camera/color/image_raw', 'type': Image, 'cb': cb_color_image},
'pointcloud': {'topic': '/camera/depth/color/points', 'type': PointCloud2, 'cb': cb_pointcloud}}



if __name__ == "__main__":
  rospy.init_node('capture_realsense', anonymous=True)
  try:
    for name in topics:
      t = topics[name]
      t['sub'] = rospy.Subscriber(t['topic'], t['type'], t['cb'] )
    print('start ...')
    while not rospy.is_shutdown():
      b_stop = False
      for name in topics:
        t = topics[name]
        if t.get('sub') is None:
          b_stop = True
          break
      if b_stop:
        break
      rospy.sleep(0.01)
  except rospy.exceptions.ROSInterruptException:
    pass
  print('end')

