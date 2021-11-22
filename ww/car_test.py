#! /usr/bin/env python

import rospy
from lane_follower import Lane_follower
from blockingbar_detector import BlockingBar_Detector
from stop_line_detector import Stop_line_detactor
from flover import Flover
from obstacle_detecter import Obstacle_detecter
from clocker import Clocker
from drive import Drive_Method

follower_center = Lane_follower('center')
follower_right = Lane_follower('right')
follower_left = Lane_follower('left')
blocking_bar = BlockingBar_Detector()
stop_line_detector = Stop_line_detactor()
flover = Flover()
obstacle_detecter = Obstacle_detecter()
clocker = Clocker()

course = 11
time = 0

if __name__ == "__main__":
    global course, follower_center, follower_right, follower_left, blocking_bar, stop_line_detector, flover, obstacle_detecter, clocker
    drive = Drive_Method()
    rospy.init_node('car_test')
    while not rospy.is_shutdown():
        if course == 1: #blocking bar
            follower_center.run = True
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = True
            stop_line_detector.run = False
            flover.run = False
            obstacle_detecter.run = False
            if not blocking_bar.detect:
                stop_line_detector.position = [[0.0, 0.0]]
                course = 2 #go next course
        elif course == 2:   #drive linear
            follower_center.run = True
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = True
            flover.run = False
            obstacle_detecter.run = False
            if len(stop_line_detector.position) >= 4:
                follower_center.run = True
                course = 0
                print "finish curve course!"
        '''
        elif course == 3:   #drive intersection first time
            follower_center.run = True
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = True
            flover.run = False
            obstacle_detecter.run = False
            if time == 0:
                time = clocker.now_time
            elif time + 8 < clocker.now_time:
                follower_center.run = False
                follower_left.run = True
            if len(stop_line_detector.position) >= 5:
                time = 0
                course = 4
        elif course == 4:   #drive intersection first time
            if time == 0:
                time = clocker.now_time
            elif time + 4 > clocker.now_time:
                drive.straight()
                drive.publish()
            else:
                course = 5
        elif course == 5:   #drive linear
            follower_center.run = False
            follower_right.run = True
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = True
            flover.run = False
            obstacle_detecter.run = False
        '''
        if course == 10:    #lead to T course
            follower_center.run = True
            follower_center.t_flag = False
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = True
            flover.run = False
            obstacle_detecter.run = False
            if len(stop_line_detector.position) >= 2:
                course = 11
        elif course == 11:  #start T course
            follower_center.run = True
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = False
            flover.run = False
            obstacle_detecter.run = False
            if time == 0:
                time = clocker.now_time
                follower_center.t_flag = False
            if clocker.now_time > time and follower_center.t_flag:
                time = 0
                course = 12
        elif course == 12:
            drive.go_sign()
            follower_center.run = False
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = False
            flover.run = True
            obstacle_detecter.run = False
            if not flover.set_pose:
                time = 0
                course = 13
        elif course == 13:
            follower_center.run = True
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = False
            flover.run = False
            obstacle_detecter.run = False
            if time == 0:
                time = clocker.now_time
                follower_center.run = False
            if time + 3 > clocker.now_time:
                follower_center.run = False
                drive.setSpeed(1)
                drive.setTurn(0.1)
                drive.publish()
            if time + 3 == clocker.now_time:
                drive.forceAngle(45)
                drive.publish()


        if course == 20:
            follower_center.run = True
            follower_right.run = False
            follower_left.run = False
            blocking_bar.run = False
            stop_line_detector.run = False
            flover.run = False
            obstacle_detecter.run = True

    rospy.spin()
