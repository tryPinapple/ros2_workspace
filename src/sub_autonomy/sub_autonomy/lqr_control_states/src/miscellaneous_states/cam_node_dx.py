#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import Image
import rospy
import rosnode


class CamNodeDx(EventState):
    """
    This state allows to diagnostic if the CAM node is active
    and if data are received on CAM topic for a given time.

    -- max_time     float       Time in seconds allowed to achieve the goal

    <= completed		        CAM node is active and data are received on CAM topic
    <= failed				    CAM node is not active or CAM node is active but no data
                                is received on CAM topic
    """

    def __init__(self, max_time):
        super().__init__(outcomes=['completed', 'failed'])

        self._cam_topic = 'raw'
        self._cam_node = '/video_source'

        # Proxy Subscriber
        self._sub = ProxySubscriberCached({self._cam_topic: Image})

        # Parameters
        self.max_time = rospy.Duration(max_time)

        self.fail = False

    def execute(self, userdata):
        # Fail condition
        if self.fail:
            return 'failed'
        else:
            Logger.loginfo(f'[CameraNodeDx] {self._cam_node} node is active'
                           f' & data was received on {self._cam_topic} topic!')
            return 'completed'

    def on_enter(self, userdata):
        # Assume failure and clear the flag if all checks passed with success
        self.fail = True
        # Start timer
        start_time = rospy.Time.now()

        # Check if CAM node is active
        if self._cam_node not in rosnode.get_node_names():
            Logger.loginfo(f'[CameraNodeDx] {self._cam_node} node is not'
                           f' active')
            return

        while self._cam_node in rosnode.get_node_names() and self.max_time > (rospy.Time.now() - start_time):
            if not self._sub.has_msg(self._cam_topic):
                Logger.loginfo(f'[CameraNodeDx] {self._cam_node} node is active'
                               f' but no data was received on {self._cam_topic} topic!')
                return

        # Get the latest msg. If there's none, check failed.
        try:
            msg = self._sub.get_last_msg(self._cam_topic)
        except KeyError:
            Logger.loginfo(f'[CameraNodeDx] No data was received on'
                           f' {self._cam_topic} topic!')
            return

            # Msg sanity check (optional)
        if not self.message_sanity_check(msg):
            Logger.loginfo('[CameraNodeDx] Received msg failed sanity check!')
            return

        self.fail = False

    @staticmethod
    def message_sanity_check(msg):
        # Validate message content
        return True
