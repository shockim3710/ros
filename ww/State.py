#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import smach_ros
import smach
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
drive = Drive_Method()

class BlockingBarState(smach.State):
    global blocking_bar, follower_center
    def __init__(self):
        smach.State.__init__(self, outcomes=['blockingbar', 'normalDriving'])

    def execute(self, userdata):
        if not blocking_bar.detect:
            blocking_bar.run = False
            follower_center.run = False
            print('BlockingBarState -> normalDrivingState')
            return 'normalDriving'
        blocking_bar.run = True
        follower_center.run = True
        print('BlockingBarState -> BlockingBarState')
        return 'blockingbar'

class NormalDrivingState(smach.State):
    global stop_line_detector, follower_center, stop_line_detector
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving', 'leftLineDriving','goStraight','finish'])
        self.count = 0

    def execute(self, userdata):
        if self.count == 0:
            if stop_line_detector.detect and len(stop_line_detector.position) == 4:
                self.count = self.count + 1
                follower_center.run = False
                stop_line_detector.run = False
                print('NormalDrivingState -> NormalDrivingState')
                return 'leftLineDriving'
            follower_center.run = True
            stop_line_detector.run = True
            print('NormalDrivingState -> NormalDrivingState')
            return 'normalDriving'


        elif self.count == 1:   #첫코너 후 사거리 직전
            if stop_line_detector or len(stop_line_detector.position) == 5:
                self.count = self.count + 1
                follower_center.run = False
                stop_line_detector.run = False
                print('NormalDrivingState -> StarightState')
                return 'goStraight'
            follower_center.run = True
            stop_line_detector.run = True
            print('NormalDrivingState -> NormalDrivingState')
            return 'normalDriving'
        else:
            return 'finish'

class LeftLineDrivingState(smach.State):
    global clocker, follower_left
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving', 'leftLineDriving'])
        self.start = 0
        self.count = 0

    def execute(self, userdata):
        if self.count == 0: # 첫코스 지나고 코너
            if self.start == 0:
                self.start = clocker.now_time
                follower_left.run = True
            if self.start + 3.0 < clocker.now_time:
                follower_left.run = False
                self.start = 0
                print('LeftLineDrivingState -> NormalDrivingState')
                return 'normalDriving'
            follower_left.run = True
            print('LeftLineDrivingState -> LeftLineDrivingState')
            return 'leftLineDriving'

class StarightState(smach.State):
    global clocker, drive
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving', 'goStraight'])
        self.start = 0

    def execute(self, userdata):
        if self.start == 0:
            self.start = clocker.now_time
        if self.start + 3.0 < clocker.now_time:
            drive.forceStop()
            drive.publish()
            self.start = 0
            print('StarightState -> NormalDrivingState')
            return 'normalDriving'
        drive.setTurn(0)
        drive.straight()
        drive.publish()
        return 'goStraight'

if __name__ == '__main__':
    rospy.init_node('tset')
    sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
    with sm:
        smach.StateMachine.add('BLOCKINGBAR', BlockingBarState(), transitions={'normalDriving': 'NORMALDRIVING', 'blockingbar':'BLOCKINGBAR'})
        smach.StateMachine.add('NORMALDRIVING', NormalDrivingState(), transitions={'normalDriving': 'NORMALDRIVING', 'leftLineDriving' : 'LEFTLINEDRIVINGSTATE', 'goStraight' : 'STRAIGHT', 'finish': 'outcome4'})
        smach.StateMachine.add('LEFTLINEDRIVINGSTATE', LeftLineDrivingState(), transitions={'normalDriving': 'NORMALDRIVING', 'leftLineDriving' : 'LEFTLINEDRIVINGSTATE'})
        smach.StateMachine.add('STRAIGHT', StarightState(), transitions={'normalDriving': 'NORMALDRIVING', 'goStraight' : 'STRAIGHT'})
    sm.execute()
    print('FINISH')
