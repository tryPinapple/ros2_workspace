#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import Imu
import rospy
import rosnode


class ImuNodeDx(EventState):
    """
    This state allows to diagnostic if the IMU node is active and if messages are published on the given topic.

    -- max_time           float    	      Time in seconds allowed to achieve the goal

    <= completed		    IMU node is active and data are received on IMU topic
    <= failed				IMU node is not active or Imu node is active but no data is received on IMU topic

    """

    def __init__(self, max_time):
        super().__init__(outcomes=['completed', 'failed'])

        self._imu_topic = 'vectornav/IMU'
        self._imu_node = '/vectornav'

        # Proxy Subscriber
        self._sub = ProxySubscriberCached({self._imu_topic: Imu})

        # Parameters
        self.max_time = rospy.Duration(max_time)

        self.fail = False

    def execute(self, userdata):
        # Fail condition
        if self.fail:
            return 'failed'
        else:
            return 'completed'

    def on_enter(self, userdata):
        self.fail = True  # Assume failure and clear the flag if all checks passed with success

        # Start timer
        start_time = rospy.Time.now()

        # Check if IMU node is active
        if self._imu_node not in rosnode.get_node_names():
            Logger.loginfo('[ImuNodeDx] Imu node is not active!')
            return

        # Wait until max_time or until a message was received
        while not self._sub.has_msg(self._imu_topic) and (rospy.Time.now() - start_time) < self.max_time:
            pass

        # Get the latest msg. If there's none, check failed.
        try:
            msg = self._sub.get_last_msg(self._imu_topic)
        except KeyError:
            Logger.loginfo('[ImuNodeDx] No data was received on IMU topic!')
            self.fail = True
            return

        # Msg sanity check (optional)
        if not self.message_sanity_check(msg):
            Logger.loginfo('[ImuNodeDx] Received msg failed sanity check!')
            return

        self.fail = False

    @staticmethod
    def message_sanity_check(msg):
        # Validate message content
        return True
