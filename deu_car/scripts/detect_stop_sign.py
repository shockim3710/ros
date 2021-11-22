#! /usr/bin/env python
# coding=utf-8

import rospy
import cv2, cv_bridge
import numpy as np
import time
from robot_drive_controller import RobotDriveController
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


MIN_MATCH_COUNT = 15
show_matched_points = True


class ImageConverter:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # 해당 영상의 특징을 모아둔 리스트를 생성.
        self.surf = cv2.xfeatures2d.SURF_create(1000)
        self.stop_sign_img = cv2.imread('stop_sign.png', cv2.IMREAD_COLOR)
        self.match_pub = rospy.Publisher("matches/is_block", Bool)
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        self.match = False
        self.robot_drive_controller = RobotDriveController()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 이미지 최적화
        cv2.useOptimized()
        cv2.setUseOptimized(True)

        # 영상의 특징의 키포인트들을 가중치들을 가져와 계산함.
        kp1, des1 = self.surf.detectAndCompute(self.stop_sign_img, None)
        kp2, des2 = self.surf.detectAndCompute(image_gray, None)

        index_params = dict(algorithm=1, trees=5)
        search_params = dict(checks=50)

        matches = None

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        good = []

        for m, n in matches:
            if m.distance < 0.6 * n.distance:
                good.append(m)

        outer_dst_pts = np.float32([])

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            outer_dst_pts = dst_pts

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matches_mask = mask.ravel().tolist()

            h, w, d = self.stop_sign_img.shape
            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)

            dst = cv2.perspectiveTransform(pts, M)

            self.match = True
            rospy.loginfo('Stop Sign detected : %s' % self.match)
        else:
            self.match = False
            # rospy.loginfo("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))

            matches_mask = None

        if self.match:
            self.robot_drive_controller.set_velocity(0)
            print("stop!")
            rospy.sleep(3)
        else:
            self.robot_drive_controller.set_velocity(1)
            print("go")

        self.robot_drive_controller.drive()

        draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=matches_mask, flags=2)

        matches_img = cv2.drawMatches(self.stop_sign_img, kp1, image, kp2, good, None, **draw_params)

        if show_matched_points:
            for pt in outer_dst_pts:
                x, y = pt[0]

                cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

        self.match_pub.publish(self.match)

        if self.match:
            time.sleep(0.5)

        cv2.imshow("stop_sign_match", matches_img)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('image_converter')
    ic = ImageConverter()

    rospy.spin()
