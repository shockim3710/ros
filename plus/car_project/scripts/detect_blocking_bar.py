#! /usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from robot_drive_controller import RobotDriveController


class BlockDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.block_pub = rospy.Publisher('detect/block', Bool, queue_size=1)
        self.drive_controller = RobotDriveController()
        # 객체를 판단하기위해 사용하는 contours 리스트
        self.contours = []

    def image_callback(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 빨강 값의 최대치와 최소치의 값을 설정하여 gray_img 로 변환
        lower_red = np.array([0, 0, 90])
        upper_red = np.array([5, 5, 110])
        gray_img = cv2.inRange(origin_image, lower_red, upper_red)

        # gray_img 의 마스크 범위를 설정하는 부분
        h, w = gray_img.shape
        block_bar_mask = gray_img
        block_bar_mask[0:180, 0:w] = 0
        block_bar_mask[240:h, 0:w] = 0

        # contours 에 해당 마스크 내에서 빨간색 부분을 이진화하여 객체의 갯수를 카운트하여 contours 에 저장
        block_bar_mask, self.contours, hierarchy = cv2.findContours(block_bar_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


if __name__ == '__main__':
    rospy.init_node('test_node')
    line_finder = BlockDetector()
    rospy.spin()

