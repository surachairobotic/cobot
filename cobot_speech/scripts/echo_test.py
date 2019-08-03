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
        rospy.init_node('echo_test', anonymous=True)
        sub_txt = rospy.Subscriber("/cobot/speech/stt", String, callback)

        rospy.loginfo('echo_test : start')
        while not rospy.is_shutdown():
            if len(str) > 0:
                print(unicode(str[0], "utf-8"))
                #os.system(cmd+str[0])
                str.pop(0)
            rospy.sleep(0.1)

    finally:
        sub_txt.unregister()
    rospy.loginfo('echo_test : end')
