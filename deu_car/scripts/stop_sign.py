#! /usr/bin/env python
# coding=utf-8

import rospy
import cv2, cv_bridge
import numpy as np
from robot_drive_controller import RobotDriveController
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class StopSign:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.block_pub = rospy.Publisher('matches/is_block', Bool, queue_size=1)
        self.drive_controller = RobotDriveController()
        self.match = False
        self.contours = []


    def image_callback(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 0, 90])
        upper_red = np.array([5, 5, 110])
        imgGray = cv2.inRange(hsv, lower_red, upper_red)

        h, w = imgGray.shape
        mask = imgGray
        mask[0:h, 0:w - w/9] = 0
        mask[h/2:h, 0:w] = 0
        mask[0:h/10, 0:w] = 0
        mask, self.contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #cv2.imshow("stop_sign", mask)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('StopSign')
    stop_sign = StopSign()

    rospy.spin()