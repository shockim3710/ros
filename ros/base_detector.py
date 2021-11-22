#!/usr/bin/env python
# coding=utf-8

import abc
import cv_bridge
from sensor_msgs.msg import Image
import rospy


class BaseDetector:
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.current_focus_pub = rospy.Publisher('current_focus_image', Image, queue_size=1)
        self.rate = rospy.Rate(20)

    @abc.abstractmethod
    def image_callback(self, msg):
        raise NotImplementedError()
