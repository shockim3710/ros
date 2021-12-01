#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from robot_drive_controller import RobotDriveController

class ObstacleStop:
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.stop_pub = rospy.Publisher('stop_sign', Bool, queue_size=1)
        self.drive_controller = RobotDriveController()


    def scan_callback(self, msg):
        angle_180 = len(msg.ranges) / 2
        angle_90 = len(msg.ranges) / 4
        angle_45 = len(msg.ranges) / 8

        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        self.range_right = max(msg.ranges[angle_180 - angle_90: angle_180 - angle_45])


if __name__ == "__main__":
    rospy.init_node('ObstacleStop')
    obstacle_stop = ObstacleStop()

    rospy.spin()