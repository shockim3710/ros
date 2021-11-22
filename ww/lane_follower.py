#! /usr/bin/env python
import math

import rospy
import cv2
from scan_image import Scan_image
from drive import Drive_Method


class Lane_follower(Scan_image):
    def __init__(self, lane):
        Scan_image.__init__(self, 'center', 0)
        self.left = Scan_image('left', 2)
        self.right = Scan_image('right', 1)
        self.t_flag = True
        self.lane = lane
        self.run = False

    def image_callback(self, msg):
        Scan_image.image_callback(self, msg)
        if self.run:
            drive = Drive_Method()
            err = 0

            if self.lane == 'center':
                err = (self.left.cx + self.right.cx - 640) / 2
                err = -float(err) / 43
            elif self.lane == 'left':
                err = self.left.cx - 45 - (640 / 2)
                err = -float(err) / 120
            elif self.lane == 'right':
                err = self.right.cx - 45 - (640 / 2)
                err = -float(err) / 120

            if abs(err) > 5.0:
                drive.setTurn(0)
            else:
                drive.setTurn(err)
            if abs(err) > 0.5:
                drive.setSpeed(0.6)
            else:
                drive.straight()
            drive.publish()

            if self.lane == 'right' or self.lane == 'left':
                return #finish

            print "left", self.left.cx
            print "right", self.right.cx
            # if self.left.cx - 320 < 0 and self.right.cx - 320 > 0 and not self.t_flag:
            if self.left.cx - 320 > -120 and self.right.cx - 320 < 150 and not self.t_flag:
                self.t_flag = True
                drive.stop_sign()
                return  # finish
