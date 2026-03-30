#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from asuqtr_power_node.msg import pod_actuator_cmd.msg

# Servo motor position in degrees
DEFAULT_POS = 90
TORPEDO1_POS = 80
TORPEDO2_POS = 100


class TorpedoState(EventState):
    '''
    Implements a state that will be used to command torpedo from PCB pod.

    -- release_action 	int	        Torpedo release action command:
                                    set 1 to release torpedo 1,
                                    set 2 to release torpedo 2,
                                    set 0 to return to default position

    <= done					        Indicates that the torpedo action command has been sent
    <= failed                       Indicates that user entered parameter is wrong
    '''

    def __init__(self, release_action):
        super().__init__(outcomes=['failed', 'done'])

        # Proxy Publisher
        self._torpedo_topic = '/pod_node/torpedo_helper'
        self._pub = ProxyPublisher({self._torpedo_topic: pod_actuator_cmd})

        # Parameters
        self.release = release_action
        self.torpedo_msg = pod_actuator_cmd()
        self.torpedo_msg.actuator = 'torpedo'

        self.fail = False

    def execute(self, userdata):
        # Fail condition
        if self.fail:
            return 'failed'
        else:
            return 'done'

    def on_enter(self, userdata):
        self.fail = True  # Assume failure and clear the flag if all checks passed with success

        # Torpedo action command to publish
        if self.release == 1:
            self.torpedo_msg.cmd = TORPEDO1_POS
            self._pub.publish(self._torpedo_topic, self.torpedo_msg)
            Logger.loginfo('[TorpedoState] torpedo 1 release cmd is sent')
        elif self.release == 2:
            self.torpedo_msg.cmd = TORPEDO2_POS
            self._pub.publish(self._torpedo_topic, self.torpedo_msg)
            Logger.loginfo('[TorpedoState] torpedo 2 release cmd is sent')
        elif self.release == 0:
            self.torpedo_msg.cmd = DEFAULT_POS
            self._pub.publish(self._torpedo_topic, self.torpedo_msg)
            Logger.loginfo('[TorpedoState] default position cmd is sent')
        else:
            Logger.loginfo('[TorpedoState] entered parameter is wrong')
            return

        self.fail = False
