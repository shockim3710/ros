#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32


class DetectStopLine:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.stop_count = 0
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.stop_pub = rospy.Publisher('stop_sign', String, queue_size=1)
        self.count_pub = rospy.Publisher('stop_count', Int32, queue_size=1)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([0, 0, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w = mask.shape

        mask[0:h * 3 / 5, 0:w] = 0
        mask[h - (h / 8):h, 0:w] = 0
        mask[0:h, 0:w / 4] = 0
        mask[0:h, w - (w / 4):w] = 0

        ret, thr = cv2.threshold(mask, 127, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) <= 0:
            self.count_pub.publish(self.stop_count)
            return  # not found

        cnt = contours[0]

        area = cv2.contourArea(cnt)
        print(area)

        if 7000.0 < area and len(contours) == 1:
            self.stop_count += 1
            msg = "stop"
            print('stop')
            print (self.stop_count)
            self.stop_pub.publish(msg)
            rospy.sleep(4)
        x, y, w, h = cv2.boundingRect(cnt)
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.drawContours(mask, [cnt], 0, (255, 255, 0), 1)

        #cv2.imshow("image", image)
        #cv2.imshow("hsv", hsv)
        #cv2.imshow("window", mask)
        cv2.waitKey(3)
        self.count_pub.publish(self.stop_count)


if __name__ == "__main__":
    rospy.init_node('white_finder')
    detect_stop_line = DetectStopLine()
    rospy.spin()
    # END ALL
