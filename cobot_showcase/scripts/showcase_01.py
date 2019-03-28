#!/usr/bin/env python

import sys
import os
import rospy
from std_msgs.msg import Bool

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse

from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import DisplayTrajectory

# Global Declarations
robot_state = RobotState()
goal_poses = PoseArray()
state = 0
indx = 0
target = []

def tg_callback(msg):
  global state
  if state == 0 and msg.data is True:
    rospy.loginfo("tg_callback is True")
    state = 1

def done_cb(_state, result):
  global state
  if state is 1:
    state = 2

def js_callback(msg):
  global b_jntArm, state, robot_state
#  if b_jntArm is True:
#  rospy.loginfo("js_callback")
#    b_jntArm = False
#    if state is 2:
  robot_state.joint_state = msg
#      state = 3

def tj_callback(msg):
  global state
  if state is 2 or state is 5 or state is 8:
    state = state + 1

def msgs_callback(msg):
  global state, indx, target
  if state is 4:
    indx = indx+1
    if indx is len(target):
      indx = 0
      state = 0
    else:
      state = 1
  elif state is 6:
    state = 7

def current_to_target(target):
  rospy.loginfo('get Fk form robot_state')
  try:
    res = getFk(robot_state.joint_state.header, ['tool0'], robot_state)
    goal_poses.poses[0] = res.pose_stamped[0].pose
  except rospy.ServiceException as exc:
    print("Service did not process request: " + str(exc))
  123
  goal_poses.poses[1] = target
  rospy.loginfo(goal_poses)
  pub_poses.publish(goal_poses)

if __name__ == "__main__":
  try:
    rospy.init_node('cobot_showcase_01', anonymous=True)
    sub_tg = rospy.Subscriber("/cobot/trigger", Bool, tg_callback)
    sub_js = rospy.Subscriber("/joint_states", JointState, js_callback)
    sub_tj = rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, tj_callback)
    sub_ms = rospy.Subscriber("/cobot/message", Bool, msgs_callback)

    rospy.loginfo('waiting GetPositionFK server')
    rospy.wait_for_service('compute_fk')
    getFk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    rospy.loginfo('GetPositionFK server : ok')

    pub_poses = rospy.Publisher("/cobot/pose", PoseArray, queue_size=10)
    pub_exe = rospy.Publisher("/cobot/execute", Bool, queue_size=10)

    global state, goal_poses, robot_state, indx, target
    goal_poses.poses = [None] * 2
    old_state = -1

    target = []
    target.append(Pose())
    target[0].position.x = 0.25
    target[0].position.y = 0.0
    target[0].position.z = 0.080
    target[0].orientation.x = 1.0
    target[0].orientation.y = -0.000016
    target[0].orientation.z = -0.000018
    target[0].orientation.w = -0.000009
    target.append(Pose())
    target[1].position.x = 0.25
    target[1].position.y = 0.0
    target[1].position.z = 0.264
    target[1].orientation.x = 0.999999
    target[1].orientation.y = -0.000033
    target[1].orientation.z = -0.001147
    target[1].orientation.w = -0.000007
    target.append(Pose())
    target[2].position.x = 0.0
    target[2].position.y = -0.25
    target[2].position.z = 0.264
    target[2].orientation.x = -0.705372
    target[2].orientation.y = -0.708837
    target[2].orientation.z = 0.000004
    target[2].orientation.w = -0.000021
    target.append(Pose())
    target[3].position.x = 0.0
    target[3].position.y = -0.25
    target[3].position.z = 0.080
    target[3].orientation.x = -0.705372
    target[3].orientation.y = -0.708837
    target[3].orientation.z = 0.000004
    target[3].orientation.w = -0.000021

#[ -0.7054064, -0.7088031, 0.0000026, -0.0000026 ]
    while not rospy.is_shutdown():
      if old_state != state:
        rospy.loginfo("state : %d", state)
        old_state = state

        if state is 1:
          current_to_target(target[indx])
          state = 2
        elif state is 3:
          s = Bool()
          s.data = True
          pub_exe.publish(s)
          state = 4

      rospy.sleep(0.01)
  finally:
    sub_tg.unregister()
    sub_js.unregister()
    sub_tj.unregister()
    sub_ms.unregister()
    rospy.loginfo('end')

