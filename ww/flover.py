#! /usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from drive import Drive_Method

class Flover:
    def __init__(self):
        self.angle = 0.0
        self.pose_sub = rospy.Subscriber('odom', Odometry, self.pose_callback)
        self.set_pose = True
        self.target_dir = 105
        self.run = False

    def pose_callback(self, msg):
        if self.run:
            ori = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            euler = euler_from_quaternion(ori)
            self.angle = round((euler[2] * 180.0 / math.pi) + 180.0, 2) #use yaw, +-15.0
            if self.set_pose:
                drive = Drive_Method()
                turn_angle = abs(self.target_dir - self.angle)
                if self.target_dir - 2.0 < self.angle < self.target_dir + 2.0:
                    self.set_pose = False
                    drive.forceStop()
                    drive.publish()
                    return  # finish set
                else:
                    drive.forceAngle(turn_angle)
                drive.publish()
