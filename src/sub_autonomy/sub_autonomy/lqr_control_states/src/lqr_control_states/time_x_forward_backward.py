#!/usr/bin/env python3


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import Vector3


'''
Created on 13.10.2020

@author: Thomas Germain
'''


class TimeXForwardBackward(EventState):
    '''
    Implements a state that can be used to move the AUV back or forward.

    -- throttle 	float	 motor speed - for down + for up.

    -- moving_time  float   number of seconds the movement is applied

    <= finished					Indicates the AUV completed the movement.

    '''

    def __init__(self, throttle, moving_time):
        super().__init__(outcomes=['finished'])
        self._pub = ProxyPublisher({'control/linear_thrust': Vector3, 'control/relative_angle': Vector3})
        self._throttle = throttle
        self.moving_time = rospy.Duration(moving_time)

    def execute(self, userdata):
        # exit conditions
        if (rospy.Time.now() - self.start_time) > self.moving_time:
            return 'finished'

    def on_enter(self, userdata):
        # set throttle for AUV
        thrust = Vector3()
        thrust.x = self._throttle
        self._pub.publish('control/linear_thrust', thrust)

        # start timer
        self.start_time = rospy.Time.now()

    def on_exit(self, userdata):
        # movement is done, set movement back to 0
        thrust = Vector3()
        thrust.x = 0
        self._pub.publish('control/linear_thrust', thrust)
        Logger.loginfo('TimeXForwardBackward reached an outcome and is exiting')

    def on_stop(self):
        thrust = Vector3()
        thrust.x = 0
        self._pub.publish('control/linear_thrust', thrust)
