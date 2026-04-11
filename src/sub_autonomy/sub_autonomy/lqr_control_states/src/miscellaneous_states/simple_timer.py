#!/usr/bin/env python3


import rospy
from flexbe_core import EventState


'''
Created on 12.11.2020

@author: Thomas Germain
'''


class SimpleTimer(EventState):
    '''
    Implements a state that can be used to move the AUV forward.

    -- time_to_wait 	float	 time to wait in second.

    <= finished					Indicates the AUV completed the movement.

    '''

    def __init__(self, time_to_wait):
        super().__init__(outcomes=['finished'])
        self.wait_time = rospy.Duration(time_to_wait)

    def execute(self, userdata):
        # exit condition
        if (rospy.Time.now() - self.start_time) > self.wait_time:
            return 'finished'

    def on_enter(self, userdata):
        # start timer
        self.start_time = rospy.Time.now()

    def on_exit(self, userdata):
       Logger.loginfo('SimpleTimer reached an outcome and is exiting')

    def on_stop(self):
        pass
