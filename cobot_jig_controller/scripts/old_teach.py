#!/usr/bin/env python

import rospy
import time

from sensor_msgs.msg import JointState
from dynamic import *
from cobot_msgs.srv import EnableNode, EnableNodeResponse, ChangeMode
from std_msgs.msg import String

arm_state = JointState()
jntSend = JointState()
ABC = []
curr = [0.0]*6
enable = False
DRIVER_KEY = ""
get_key = rospy.ServiceProxy('/cobot/cobot_core/change_mode', ChangeMode)

def main():
  rospy.init_node('cobot_teach', anonymous=True)
  pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
  sub = rospy.Subscriber("/cobot/joint_states", JointState, callback)
  sub_ = rospy.Subscriber("/cobot/cobot_teach", JointState, callback)

  global ABC, curr, arm_state, jntSend, enable, DRIVER_KEY, get_key
  rate = rospy.Rate(50)

  ABC = [ [ 355.0, 1240.0,  245.0],
          [-177.0, -690.0, -650.0],
          [ 180.0,  564.0,  206.0],
          [ 256.0,  689.0,  232.0],
          [ 241.0,   22.0,   58.0],
          [   0.0,    0.0,    0.0] ];

  jntSend.name = ['J1','J2','J3','J4','J5','J6']
  jntSend.position = [];
  jntSend.velocity = [];
  jntSend.effort = [0.0];
  eff_offset = [50.0, -300.0, 150.0, 150.0, 50.0]
  eff = [0.0]*6

  s = rospy.Service('/cobot/cobot_teach/enable', EnableNode, handle_enable_node)

  rospy.wait_for_service('/cobot/cobot_core/change_mode')

  enable = True
  t_start = time.time()
  while not rospy.is_shutdown():
#    rospy.loginfo("%s : [%s]" % (jntSend.header.frame_id, enable))
    if len(arm_state.velocity) >= 5 and enable:
        for i in range(5):
          eff[i] = curr[i];
          if arm_state.velocity[i] > 0.01:
            eff[i] = eff[i] + eff_offset[i];
          elif arm_state.velocity[i] < -0.01:
            eff[i] = eff[i] - eff_offset[i];
        jntSend.header.frame_id = DRIVER_KEY
        jntSend.effort = eff;
        set_jnt(jntSend, pub)
    rate.sleep()
  rospy.loginfo("step 2 : OK")

  sub.unregister()
  sub_.unregister()
  pub.unregister()

  print("OK !!!")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

def check_tq():#(t_from_start):
  global arm_state, ABC, curr

  pos = np.array(arm_state.position)
  vel = np.zeros(pos.shape)
  acc = np.zeros(pos.shape)

  vars = eq.get_vars(arm_state.position) # load equation parameters for calculating torque
  torque_with_ddq, torque_no_ddq = cal_torque(pos, vel, acc, vars)

  curr = [0.0]*6
  for i in range(6):
    curr.append(torque2current(torque_with_ddq[i], vel[i], ABC[i]))

  curr[0] = torque_with_ddq[0]
  curr[1] = -186.231185 * torque_with_ddq[1]
  if torque_with_ddq[2] > 0:
    curr[2] = 143.031415 * torque_with_ddq[2] + 179.987635
  else:
    curr[2] = 143.031415 * torque_with_ddq[2] - 179.987635
  curr[3] = 146.598336 * torque_with_ddq[3]
  curr[4] = 297.804331 * torque_with_ddq[4]
#  print(torque_with_ddq)

def set_jnt(jnt, pub):
  pub.publish(jnt)
  return True;

def callback(data):
  global arm_state
  arm_state = data
  check_tq()

def handle_enable_node(req):
    global enable, DRIVER_KEY
    error = String()
    error.data = "OK"
    enable = False
    rospy.loginfo("handle_enable_node(req)")
    if req.enable:
        resp = get_key("TEACH_MODE")
        if resp.error == "OK":
            enable = req.enable
            DRIVER_KEY = resp.key
            rospy.loginfo("DRIVER_KEY is %s", DRIVER_KEY)
        else:
            error.data = "ERROR"
    return EnableNodeResponse(error)

if __name__ == '__main__':
  main()
