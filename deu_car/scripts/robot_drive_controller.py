#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class RobotDriveController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.velocity = 0
        self.angular = 0


    def set_velocity(self, velocity):
        self.velocity = velocity


    def set_angular(self, angular):
        self.angular = angular


    def pause_robot(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)


    def drive(self):
        self.twist.linear.x = self.velocity
        self.twist.angular.z = self.angular
        self.cmd_vel_pub.publish(self.twist)


if __name__ == "__main__":
    rospy.init_node('RobotDriveController')
    robotDrivenController = RobotDriveController()
    robotDrivenController.set_velocity(1)
    robotDrivenController.drive()

    while not rospy.is_shutdown():
        robotDrivenController.cmd_vel_pub.publish(robotDrivenController.twist)
        robotDrivenController.rate.sleep()