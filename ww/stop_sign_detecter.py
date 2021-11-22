#! /usr/bin/env python
# -*- coding:utf-8

import rospy
import cv2
from scan_image import Scan_image
import os

class Stop_sign_detecter(Scan_image):
    def __init__(self):
        Scan_image.__init__(self, 'center', 0)
        self.surf = cv2.xfeatures2d.SURF_create(1000)
        file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "./stop_sign.png")
        self.stop_sign = cv2.imread(file, 0)
        self.res = None
        self.match = False
        self.run = False

    def image_callback(self, msg):
        if self.run:
            Scan_image.image_callback(self, msg)
            img2 = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            height, width = img2.shape
            img2[:height / 6, :] = 0
            img2[height / 2:height, :] = 0
            img1 = self.stop_sign
            # Initiate SIFT detector
            sift = cv2.xfeatures2d.SIFT_create()
            # find the keypoints and descriptors with SIFT
            kp1, des1 = sift.detectAndCompute(img1, None)
            kp2, des2 = sift.detectAndCompute(img2, None)
            # FLANN parameters
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=50)  # or pass empty dictionary
            try:
                flann = cv2.FlannBasedMatcher(index_params, search_params)
                matches = flann.knnMatch(des1, des2, k=2)
            except Exception as ex:
                print('knnMatch error')
                return

            print "matches: ", len(matches)

            '''
            # Need to draw only good matches, so create a mask
            matchesMask = [[0, 0] for i in xrange(len(matches))]
            # ratio test as per Lowe's paper
            for i, (m, n) in enumerate(matches):
                if m.distance < 0.7 * n.distance:
                    matchesMask[i] = [1, 0]
            draw_params = dict(matchColor=(0, 255, 0),
                               singlePointColor=(255, 0, 0),
                               matchesMask=matchesMask,
                               flags=0)
            img3 = cv2.drawMatchesKnn(img1, kp1, img2, kp2, matches, None, **draw_params)
            
            cv2.imshow("parking", img3)
            cv2.waitKey(3)
            '''
