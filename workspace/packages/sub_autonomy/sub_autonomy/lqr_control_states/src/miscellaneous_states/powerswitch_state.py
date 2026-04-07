#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Bool


class PowerSwitchState(EventState):
    '''
    Implements a state that will be used to power ON/OFF the power switch output
    from IO NODE.

    -- release_output 	bool	    Power switch output:
                                    set 1 to power ON,
                                    set 0 to power OFF

    <= done					        Indicates that the power switch output has been sent
    <= failed                       Indicates that user entered parameter is wrong
    '''

    def __init__(self, release_output):
        super().__init__(outcomes=['failed', 'done'])

        # Proxy Publisher
        self._power_switch_topic = '/io/power_switch'
        self._pub = ProxyPublisher({self._power_switch_topic: Bool})

        # Parameters
        self.output = release_output

        self.fail = False

    def execute(self, userdata):
        # Fail condition
        if self.fail:
            return 'failed'
        else:
            return 'done'

    def on_enter(self, userdata):
        self.fail = True  # Assume failure and clear the flag if all checks passed with success

        # Power switch output to publish
        if self.output == 1:
            self._pub.publish(self._power_switch_topic, True)
            Logger.loginfo('[PowerSwitchState] power switch output ON is sent')
        elif self.output == 0:
            self._pub.publish(self._power_switch_topic, False)
            Logger.loginfo('[PowerSwitchState] power switch output OFF is sent')
        else:
            Logger.loginfo('[PowerSwitchState] entered parameter is wrong')
            return

        self.fail = False
