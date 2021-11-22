#!/usr/bin/env python

import rospy
import smach_ros
import smach
from time import sleep

class BlockingBarState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

    def execute(self, userdata):
        print('BlockingBarState -> StopState' )
        sleep(1)
        return 'stop'

class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving'])

    def execute(self, userdata):
        print('StopState -> NormalDrivingState' )
        sleep(1)
        return 'normalDriving'

class NormalDrivingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving','goStraight','finish'])
        self.count = 0

    def execute(self, userdata):
        if self.count == 0 :
            print('NormalDrivingState -> NormalDrivingState' )
            sleep(1)

            self.count = self.count + 1
            a = str(self.count)
            print(a)

            return 'normalDriving'
        elif self.count == 1 :
            print('NormalDrivingState -> StarightState')
            sleep(1)

            self.count = self.count + 1
            a = str(self.count)
            print(a)

            return 'goStraight'
        else :

            self.count = self.count + 1
            a = str(self.count)
            print(a)

            print('NormalDrivingState -> finsih' )
            sleep(1)
            return 'finish'

class StarightState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving'])

    def execute(self, userdata):
        print('StarightState -> NormalDrivingState')
        sleep(1)
        return 'normalDriving'

if __name__ == '__main__':
    sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
    with sm:
        smach.StateMachine.add('BLOCKINGBAR', BlockingBarState(), transitions={'stop': 'STOP'})
        smach.StateMachine.add('STOP', StopState(), transitions={'normalDriving': 'NORMALDRIVING'})
        smach.StateMachine.add('NORMALDRIVING', NormalDrivingState(), transitions={'normalDriving': 'NORMALDRIVING', \
                                                                                   'goStraight' : 'STRAIGHT',
                                                                                   'finish': 'outcome4'})
        smach.StateMachine.add('STRAIGHT', StarightState(), transitions={'normalDriving': 'NORMALDRIVING'})


    sm.execute()

    print('FINISH')

    '''
    #!/usr/bin/env python

import rospy
import smach_ros

from stop_line_detector import Stop_line_detactor
from lane_follower import Lane_follower
from blockingbar_detector import BlockingBar_Detector
import smach
from time import sleep
from std_msgs.msg import String


class BlockingBarState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving'])
        self.stop_line_detector = Stop_line_detactor()
        self.turefalse = False

    def execute(self, userdata):
        print('BlockingBarState -> NormalDrivingState' )
        sleep(1)
        follower = Lane_follower()
        bar_detector = BlockingBar_Detector()
        #while(~self.turefalse):
        #    self.turefalse = self.stop_line_detector.getTf()


        return 'normalDriving'

class NormalDrivingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normalDriving','goStraight','finish'])
        self.count = 0

    def execute(self, userdata):
        if self.count == 0 :
            print('NormalDrivingState -> NormalDrivingState' )
            sleep(1)

            self.count = self.count + 1
            a = str(self.count)
            print(a)

            return 'normalDriving'
        elif self.count == 1 :
            print('NormalDrivingState -> StarightState')
            sleep(1)

            self.count = self.count + 1
            a = str(self.count)
            print(a)

            return 'goStraight'
        else :

            self.count = self.count + 1
            a = str(self.count)
            print(a)

            print('NormalDrivingState -> finsih' )
            sleep(1)
            return 'finish'



class StarightState(smach.State):
    def __init__(self):
        rospy.init_node('car_test')
        smach.State.__init__(self, outcomes=['normalDriving'])

    def execute(self, userdata):
        print('StarightState -> NormalDrivingState')
        sleep(1)
        return 'normalDriving'

def monitor_cb(msg):
    a = 0
    if(msg == 'true'):
        if( a == 0 ):
            print(msg)
            a = a + 1
            return 'stop'
    else :
        print(msg)

if __name__ == '__main__':

    sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
    with sm:
        smach.StateMachine.add('BLOCKINGBAR', BlockingBarState(), transitions={'normalDriving': 'NORMALDRIVING'})
        smach.StateMachine.add('NORMALDRIVING', NormalDrivingState(), transitions={'normalDriving': 'NORMALDRIVING', \
                                                                                   'goStraight' : 'STRAIGHT',
                                                                                   'finish': 'outcome4'})
        #smach.StateMachine.add('STRAIGHT', smach_ros.MonitorState("/stop_sign", String, monitor_cb), transitions={'normalDriving': 'NORMALDRIVING'})


    sm.execute()

    print('FINISH')
    rospy.spin()
    
    '''