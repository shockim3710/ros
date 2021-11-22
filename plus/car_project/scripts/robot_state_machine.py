#!/usr/bin/env python

import rospy
from smach import StateMachine
from define_robot_state import SettingLane, DetectBlockingBar, LaneTrace, AvoidObstacle, DetectStopSign, \
    RightAngleParking, ParallelParking

if __name__ == "__main__":
    rospy.init_node('test_node')
    driving_test_site = StateMachine(outcomes=['success'])
    with driving_test_site:
        StateMachine.add('SETTING_LANE', SettingLane(), transitions={'success': 'DETECT_BLOCKING_BAR'})
        StateMachine.add('DETECT_BLOCKING_BAR', DetectBlockingBar(), transitions={'success': 'LANE_TRACE1'})
        StateMachine.add('LANE_TRACE1', LaneTrace(), transitions={'success': 'DETECT_STOP_SIGN1'})
        # StateMachine.add('RIGHT_ANGLE_PARKING', RightAngleParking(), transitions={'success': 'DETECT_OBSTACLE'})
        StateMachine.add('DETECT_STOP_SIGN1', DetectStopSign(), transitions={'success': 'LANE_TRACE2'})
        StateMachine.add('LANE_TRACE2', LaneTrace(), transitions={'success': 'DETECT_OBSTACLE'})
        StateMachine.add('DETECT_OBSTACLE', AvoidObstacle(), transitions={'success': 'LANE_TRACE3'})
        StateMachine.add('LANE_TRACE3', LaneTrace(), transitions={'success': 'DETECT_STOP_SIGN2'})
        # StateMachine.add('PARALLEL_PARKING', ParallelParking(), transitions={'success': 'DETECT_STOP_SIGN'})
        StateMachine.add('DETECT_STOP_SIGN2', DetectStopSign(), transitions={'success': 'success'})
        # StateMachine.add('') ....

    driving_test_site.execute()
    rospy.spin()
