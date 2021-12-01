#!/usr/bin/env python
# coding=utf-8

import rospy
from smach import State
import time
import math
from robot_drive_controller import RobotDriveController
from line_tracer import LineTracer
from blocking_bar import BlockingBar
from stop_sign import StopSign
from obstacle_stop import ObstacleStop

class SettingLine(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        pass

        return 'success'


class Bar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, ud):
        bar = BlockingBar()
        countfloor = 0

        while True:
            print(len(bar.contours))

            if len(bar.contours) < 4 and countfloor > 3:
                bar.drive_controller.set_velocity(1)

                print('GO')
                start_time = time.time() +3

                while True:
                    bar.drive_controller.drive()
                    if time.time() - start_time > 0:
                        break

                bar.drive_controller.set_velocity(0)

                return 'success'

            elif len(bar.contours) > 3 and countfloor > 3:
                bar.drive_controller.set_velocity(0)
                print('BAR_STOP')

            else:
                print('FINDING')

            countfloor = countfloor + 1
            rospy.sleep(0.3)


class Line(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])


    def execute(self, ud):
        left_line = LineTracer('my_left_camera/rgb/image_raw')
        right_line = LineTracer('my_right_camera/rgb/image_raw')
        stop_line = LineTracer('camera/rgb/image_raw')
        drive_controller = RobotDriveController()

        rate = rospy.Rate(10)
        count = 0

        while not rospy.is_shutdown():
            cx = (left_line.cx + right_line.cx) / 2
            err = -float(cx) / 100

            if stop_line.area > 9000.0:
                drive_controller.set_velocity(0)
                drive_controller.set_angular(0)
                count = count + 1
                print('LINE_STOP')
                print(count)

                rospy.sleep(3)

            if count == 4:
                drive_controller.set_velocity(0)
                drive_controller.set_angular(-0.2)

                start_time = time.time() + 1.5

                while True:
                    drive_controller.drive()

                    if time.time() - start_time > 0:
                        count = count + 1

                        break

                drive_controller.set_velocity(1)
                drive_controller.set_angular(0)
                drive_controller.drive()

                start_time = time.time() + 5

                while True:
                    drive_controller.drive()

                    if time.time() - start_time > 0:
                        count = count + 1

                        break

                print("END")

                drive_controller.set_velocity(0)

            if abs(err) > 0.14:
                drive_controller.set_velocity(0.4)
                drive_controller.set_angular(err)
                drive_controller.drive()

            elif abs(err) < 0.14:
                drive_controller.set_velocity(0.5)
                drive_controller.set_angular(err)
                drive_controller.drive()

            rate.sleep()

        return 'success'


class Sign(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])


    def execute(self, ud):
        sign = StopSign()

        if len(sign.contours) == 0:

            sign.drive_controller.set_velocity(0)
            print('SIGN_STOP')

        rospy.sleep(3)
        print('GO')
        sign.drive_controller.set_velocity(1)

        return 'success'


class Obstacle(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])


    def execute(self, ud):
        obstacle = ObstacleStop()
        drive_controller = RobotDriveController()

        # 정면에 없고 오른쪽 대각선 방향에도 없을경우, 각 값이 2 이하일 경우 주행 유지
        if obstacle.range_ahead > 1 or obstacle.range_right > 1 or \
                ((math.isnan(obstacle.range_ahead)) and math.isnan(obstacle.range_right)):
            value = False
            obstacle.stop_pub.publish(value)

            print('GO')
        # 아니면 정지 토픽 발행
        else:
            value = True
            obstacle.stop_pub.publish(value)
            drive_controller.set_velocity(0)

            print('OBSTACLE_STOP')
            rospy.sleep(4)

        return 'success'