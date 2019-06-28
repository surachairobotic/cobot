#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import rospy
from std_msgs.msg import String

# Global Declarations
str = []

def callback(msg):
    str.append(msg.data)

if __name__ == "__main__":
    try:
        rospy.init_node('cobot_text2speech', anonymous=True)
        sub_txt = rospy.Subscriber("/cobot/speech/tts", String, callback)

        cmd = '/home/mtec/catkin_ws/src/cobot/cobot_speech/src/speak_script.sh '
        rospy.loginfo('cobot_text2speech : start')
        while not rospy.is_shutdown():
            if len(str) > 0:
                rospy.logdebug(str)
                os.system(cmd+str[0])
                str.pop(0)
            rospy.sleep(0.1)

    finally:
        sub_txt.unregister()
    rospy.loginfo('cobot_text2speech : end')
