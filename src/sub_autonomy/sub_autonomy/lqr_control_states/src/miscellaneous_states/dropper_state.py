#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from asuqtr_power_node.msg import pod_actuator_cmd

# Servo position in degrees
DEFAULT_POS = 0
DROPPER1_POS = 180
DROPPER2_POS = 180


class DropperState(EventState):
    '''
    Implements a state that will be used to command dropper from PCB pod.

    -- release_action 	int	        Torpedo release action command:
                                    set 1 to release dropper 1,
                                    set 2 to release dropper 2,
                                    set 0 to return to default position

    <= done					        Indicates that the dropper action command has been sent
    <= failed                       Indicates that user entered parameter is wrong
    '''

    def __init__(self, release_action):
        super().__init__(outcomes=['failed', 'done'])

        # Proxy Publisher
        self._dropper_topic = '/pod_node/dropper_driver'
        self._pub = ProxyPublisher({self._dropper_topic: pod_actuator_cmd})

        # Parameters
        self.release = release_action
        self.dropper_msg = pod_actuator_cmd()

        self.fail = False

    def execute(self, userdata):
        # Fail condition
        if self.fail:
            return 'failed'
        else:
            return 'done'

    def on_enter(self, userdata):
        self.fail = True  # Assume failure and clear the flag if all checks passed with success

        # Dropper action command to publish
        if self.release == 1:
            self.dropper_msg.actuator = 'dropper 1'
            self.dropper_msg.cmd = DROPPER1_POS
            self._pub.publish(self._dropper_topic, self.dropper_msg)
            Logger.loginfo('[DropperState] dropper 1 release cmd is sent')
        elif self.release == 2:
            self.dropper_msg.actuator = 'dropper 2'
            self.dropper_msg.cmd = DROPPER2_POS
            self._pub.publish(self._dropper_topic, self.dropper_msg)
            Logger.loginfo('[DropperState] dropper 2 release cmd is sent')
        elif self.release == 0:
            self.dropper_msg.actuator = 'dropper 1'
            self.dropper_msg.cmd = DEFAULT_POS
            self._pub.publish(self._dropper_topic, self.dropper_msg)
            self.dropper_msg.actuator = 'dropper 2'
            self.dropper_msg.cmd = DEFAULT_POS
            self._pub.publish(self._dropper_topic, self.dropper_msg)
            Logger.loginfo('[DropperState] default position cmd is sent for both dropper')
        else:
            Logger.loginfo('[DropperState] entered parameter is wrong')
            return

        self.fail = False
