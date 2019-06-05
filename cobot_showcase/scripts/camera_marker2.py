#!/usr/bin/env python

import sys
import os
import rospy

from cobot_pose import CobotImageHandler

if __name__ == "__main__":
    try:
        rospy.init_node('camera_marker', anonymous=True)
        rospy.loginfo('camera_marker . . .')

        rospy.loginfo('waiting camera server')
        cobot_img = CobotImageHandler()
        rospy.on_shutdown(cobot_img.terminate)
        rospy.loginfo('camera server : ok')

        cobot_img.run()
        rospy.sleep(0.01)
    finally:
        rospy.loginfo('end')
