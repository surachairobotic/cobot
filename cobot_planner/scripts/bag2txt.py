#!/usr/bin/env python

'''

save data from bag to plot
/cobot/goal (sensor_msgs/JointState)
/joint_states (sensor_msgs/JointState)

'''

import sys
import os
import rospy
import tf
import tf2_ros
import time
from sensor_msgs.msg import JointState


b_callback = False
save_dir = "/home/tong/catkin_ws/src/cobot/cobot_planner/scripts/bag2txt_"
t_start = time.time()



def cb_jointstate(msg):
  data = []
  data2 = [msg.position, msg.velocity, msg.effort]
  for j in range(len(data2)):
    for i in range(len(msg.name)):
      data.append(data2[j][i])
  return data




topic_info = [
  ['/joint_states', JointState, cb_jointstate, 'joint_states.txt']
]


class cTopic():
  def __init__(self, data):
    global save_dir
    self.topic_name = data[0]
    self.topic_type = data[1]
    self.cb_get_data = data[2]
    self.fname = save_dir + data[3]
    try:
      os.remove(self.fname)
    except OSError:
      pass
    
    self.sub = rospy.Subscriber(self.topic_name, self.topic_type, self.cb )
    self.b_cb = False
  
  def cb(self, msg):
    global subs, t_start
    self.b_cb = True
    b_error = True
    try:
      data = self.cb_get_data(msg)
      with open( self.fname, 'at' ) as f:
        f.write(str(time.time()-t_start) + ' ')
        for v in data:
          f.write(str(v)+' ')
        f.write('\n')
      b_error = False
    finally:
      if b_error and self.sub is not None:
        self.sub.unregister()
        self.sub = None
      self.b_cb = False


if __name__ == "__main__":
  rospy.init_node('bag2txt', anonymous=True)
  try:
    log_file = str(rospy.get_param('~log_file', ''))
    if len(log_file)==0:
      rospy.logerr("no log file name : " + log_file)
      exit()
    if len(save_dir)==0:
      rospy.logerr("no save dir : " + save_dir)
      exit()
    
    fname = None
    for i in range(len(log_file)):
      j = len(log_file)-i-1
      if log_file[j]=='/':
        fname = log_file[j+1:-4]
        break
    if fname is None or len(fname)==0:
      rospy.logerr("invalid log file name : " + log_file)
      exit()
    
    save_dir+= fname + '/'
    if not os.path.exists(save_dir):
      os.makedirs(save_dir)
    
    topics = []
    for t in topic_info:
      topics.append(cTopic(t))
    print('start ...')
    
    while not rospy.is_shutdown():
      for t in topics:
        if t.sub is None:
          break
      rospy.sleep(0.01)
    for t in topics:
      if t.sub is not None:
        t.sub.unregister()
    
    for t in topics:
      while t.b_cb:
        rospy.sleep(0.01)
  except rospy.exceptions.ROSInterruptException:
    pass
  print('end')

