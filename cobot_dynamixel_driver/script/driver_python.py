
import sys
import os
import rospy
from pprint import pprint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from cobot_dynamixel_driver.srv import get_motor_number
import time
from time import sleep
import math
import numpy as np
import traceback


import cJoint


if __name__ == "__main__":
  try:
    rospy.init_node('urg_find_ball', anonymous=True)
    joints = cJoint.init()
    sub_goal = rospy.Subscriber("cobot_dynamixel_driver/goal", JointState, callback_goal)
    pub_goal = rospy.Publisher("cobot_dynamixel_driver/joint_states", JointState, queue_size=10)
    service = rospy.Service('cobot_dynamixel_driver/get_motor_number', get_motor_number, callback_motor_number)
    while sub_urg is not None and not rospy.is_shutdown():

      rospy.sleep(0.01)
  finally:
    cJoint.terminate()
