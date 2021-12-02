#!/usr/bin/env python

import rospy
from smach import StateMachine
from robot_state import SettingLine, Bar, Line, Sign, Obstacle

if __name__ == "__main__":
    rospy.init_node('test_node')
    driving_test_site = StateMachine(outcomes=['success'])
    with driving_test_site:
        StateMachine.add('SettingLine', SettingLine(), transitions={'success': 'Bar'})
        StateMachine.add('Bar', Bar(), transitions={'success': 'Line1'})
        StateMachine.add('Line1', Line(), transitions={'success': 'Sign1'})
        StateMachine.add('Sign1', Sign(), transitions={'success': 'Obstacle'})
        StateMachine.add('Obstacle', Obstacle(), transitions={'success': 'Line2'})
        StateMachine.add('Line2', Line(), transitions={'success': 'Sign2'})
        StateMachine.add('Sign2', Sign(), transitions={'success': 'success'})

    driving_test_site.execute()
    rospy.spin()