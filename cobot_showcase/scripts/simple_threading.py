#!/usr/bin/env python

import sys
import os
import rospy
import threading

# Global Declarations
t1 = None
t2 = None

def func1():
    while not rospy.is_shutdown():
        rospy.loginfo('func1')

def func2():
    while not rospy.is_shutdown():
        rospy.loginfo('func2')

def terminate():
    if t1.is_alive:
        t1.join()
    if t2.is_alive:
        t2.join()

if __name__ == "__main__":
    try:
        rospy.init_node('simple_threading', anonymous=True)
        rospy.loginfo('simple_threading...')
        t1 = threading.Thread(target=func1)
        t2 = threading.Thread(target=func2)
        t1.daemon = True
        t2.daemon = True
        rospy.on_shutdown(terminate)
        t1.start()
        t2.start()
        while not rospy.is_shutdown():
            rospy.loginfo('main')
    finally:
        if t1.is_alive:
            t1.join()
        if t2.is_alive:
            t2.join()
    rospy.loginfo('end')
