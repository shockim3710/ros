#! /usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock

class Clocker:
    def __init__(self):
        self.clock_sub = rospy.Subscriber('clock', Clock, self.clock_callback)
        self.now_time = 0

    def clock_callback(self, msg):
        self.now_time = msg.clock.secs