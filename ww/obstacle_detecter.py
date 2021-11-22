#!/usr/bin/env python
# coding=utf-8

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from drive import Drive_Method

class Obstacle_detecter:
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.drive_controller = Drive_Method
        self.run = False

    def scan_callback(self, msg):
        if self.run:
            drive = Drive_Method()
            angle_180 = len(msg.ranges) / 2
            angle_90 = len(msg.ranges) / 4
            angle_45 = len(msg.ranges) / 8

            # msg.ranges / 2 = range_ahead
            self.range_ahead = msg.ranges[len(msg.ranges) / 2]
            self.range_right = max(msg.ranges[len(msg.ranges) / 2:])

            print self.range_right

            # 정면 물체, 측면 물체까지의 거리 출력
            # print "range ahead : %0.2f" % self.range_ahead
            # print "range right : %0.2f" % self.range_right

            if self.range_ahead > 1.8 or self.range_right > 1.8 or \
                    ((math.isnan(self.range_ahead)) and math.isnan(self.range_right)):
                drive.go_sign()
                print('go')
            else:
                drive.stop_sign()
                print('stop')