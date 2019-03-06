#!/usr/bin/env python

import rospy
import time
import actionlib
from actionlib_msgs.msg import GoalStatus, GoalID
from cobot_pick.msg import CobotFindObjectAction
from cobot_pick.msg import CobotFindObjectGoal

b_done = False
action_client = None

def done_cb(state, result):
  global b_done
  print('done')
  print('state : ' + str(state))
  print('result : ' + str(result))
  b_done = True

def feedback_cb(feedback):
  print('feedback : ' + str(feedback))

def active_cb():
  print('active')


def shutdown_cb():
  global action_client
  action_client.cancel_goal()
  
if __name__ == "__main__":
  try:
    rospy.init_node('cobot_find_basket_client')
    action_client = actionlib.SimpleActionClient('/cobot/find_basket', CobotFindObjectAction)
    print('waiting server')
    action_client.wait_for_server()
    goal = CobotFindObjectGoal()
    print('send goal')
    action_client.send_goal(goal=goal
      , done_cb = done_cb  # Callback that gets called on transitions to Done
      , active_cb = active_cb  # No-parameter callback that gets called on transitions to Active
      , feedback_cb = feedback_cb  # Callback that gets called whenever feedback for this goal is received
    )
    rospy.on_shutdown(shutdown_cb)
    print('wait result')
#    action_client.wait_for_result()
    while not b_done:
      time.sleep(0.1)
      if rospy.is_shutdown():
        break
    if b_done:
      print(action_client.get_result())
    time.sleep(0.5)
    print('end')
    
  finally:
    pass
