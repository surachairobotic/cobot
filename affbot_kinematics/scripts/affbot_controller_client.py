#!/usr/bin/env python

import sys
import os
import rospy
import math
import time
import lib_controller
import actionlib


from affbot_kinematics.msg import AffbotMoveLastPlanAction
from affbot_kinematics.msg import AffbotMoveLastPlanGoal

def done_cb(state, result):
  print('done')
  print('state : ' + str(state))
  print('result : ' + str(result))

def feedback_cb(feedback):
  print('feedback : ' + str(feedback))

def active_cb():
  print('active')


if __name__ == "__main__":
  try:
    rospy.init_node('affbot_controller_client')
    client = actionlib.SimpleActionClient('affbot/controller/move_last_plan', AffbotMoveLastPlanAction)
    client.wait_for_server()
    goal = AffbotMoveLastPlanGoal()
    print('send goal')
    client.send_goal(goal=goal
      , done_cb = done_cb  # Callback that gets called on transitions to Done
      , active_cb = active_cb  # No-parameter callback that gets called on transitions to Active
      , feedback_cb = feedback_cb  # Callback that gets called whenever feedback for this goal is received
    )
    print('wait result')
    client.wait_for_result()
    print(client.get_result())
    print('end')
    time.sleep(0.1)
    print('end2')
    
  finally:
    pass

