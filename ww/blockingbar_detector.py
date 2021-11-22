#! /usr/bin/env python

import numpy
import cv2
from scan_image import Scan_image
from drive import Drive_Method


class BlockingBar_Detector(Scan_image):
    def __init__(self):
        Scan_image.__init__(self, 'center', 0)
        self.drive = Drive_Method()
        self.detect = True
        self.run = False

    def image_callback(self, msg):
        if self.run:
            Scan_image.image_callback(self, msg)
            lower_red = numpy.array([0, 0, 90])
            upper_red = numpy.array([5, 5, 110])
            img = cv2.inRange(self.image, lower_red, upper_red)

            h, w = img.shape
            img[0:180, 0:w] = 0
            img[240:h, 0:w] = 0

            _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                self.drive.go_sign()
                self.detect = False
            else:
                self.drive.stop_sign()
                self.detect = True