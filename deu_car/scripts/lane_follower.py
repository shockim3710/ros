#!/usr/bin/env python
# coding=utf-8

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from robot_drive_controller import RobotDriveController
import numpy


class LineTracer:
    def __init__(self, image_topic):
        self.bridge = cv_bridge.CvBridge()
        # 추적하는 라인에 무게중심 점을 찍는 토픽 카메라 2개로 판단.
        self.image_pub = rospy.Publisher(image_topic + "/circle", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.t = image_topic
        self.cx = 0

        self.stop_count = 0
        # 정지선 인식을 위한 중앙 카메라 사용.
        self.stop_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback2)
        # stop line 을 판단하기 위한 area 크기.
        self.area = 0

    def image_callback(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # hsv 이미지로 변환.
        hsv_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)

        # v 값을 이미지에서 추출하여 값에 넣어줌.
        _, _, v = cv2.split(hsv_image)
        # 노란색 이미지의 범위를 지정.
        v = cv2.inRange(v, 220, 255)

        # 감지한 객체의 무게 중심을 화면에 찍는 변수
        # M은 리스트 형식이다.
        M = cv2.moments(v)
        if M['m00'] > 0:
            self.cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(origin_image, (self.cx, cy), 20, (0, 0, 255), -1)
            self.cx = self.cx - 320

        # cv2로 변환했던 이미지를 imgmsg 로 재 변환.
        origin_image = self.bridge.cv2_to_imgmsg(origin_image)
        self.image_pub.publish(origin_image)





    def image_callback2(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 흰색의 최대값과 최소값의 범위를 지정하여 마스크범위로 지정.
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 마스크 범위 지정.
        h, w = mask.shape
        #search_top = 3 * h / 4
        #search_bot = 3 * h / 4 + 20
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0

        # 127값을 넘는 값들은 255(흰색), 아래는 0으로 이진화한다.
        _, thr = cv2.threshold(mask, 127, 255, 0)

        # 이진화된 영상으로부터 객체를 찾는다.
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) <= 0:
            return  # not found

        cnt = contours[0]

        # area 에 contour 값을 넣어서 정지선을 처리한다.
        self.area = cv2.contourArea(cnt)

        # 이진화된 영상으로부터 사각형의 윤곽을 얻는다.
        x, y, w, h = cv2.boundingRect(cnt)
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawContours(mask, [cnt], 0, (255, 255, 0), 1)
        #cv2.imshow("window", image)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('lane_trace')
    left_line = LineTracer('my_left_camera/rgb/image_raw')
    right_line = LineTracer('my_right_camera/rgb/image_raw')
    stop_line = LineTracer('camera/rgb/image_raw')
    drive_controller = RobotDriveController()
    rate = rospy.Rate(20)
    count = 0
    while not rospy.is_shutdown():
        cx = (left_line.cx + right_line.cx)/2
        err = -float(cx)/100

        if stop_line.area > 9000.0:
            drive_controller.set_velocity(0)
            drive_controller.set_angular(0)
            count = count + 1
            print('stop!')
            print(count)
            rospy.sleep(3)

        if count == 4:
            drive_controller.set_velocity(1)
            drive_controller.set_angular(0)
            drive_controller.drive()

        if abs(err) > 0.2 and stop_line.area < 9000.0:
            drive_controller.set_velocity(0.3)
            drive_controller.set_angular(err)
            drive_controller.drive()

        elif abs(err) <= 0.2 and stop_line.area < 9000.0:
            drive_controller.set_velocity(0.7)
            drive_controller.set_angular(err)
            drive_controller.drive()



        rate.sleep()

    rospy.spin()

