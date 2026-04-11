#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from asuqtr_io_node.msg import mission_led_cmd


#COLOR DICTIONARY
color_list = {"red": 0xFF0000,
              "blue": 0x0000FF,
              "green": 0x00FF00,
              "yellow": 0xFFFF00,
              "white": 0xFFFFFF,
              "none": 0x000000}

# MODE DICTIONARY
mode_list = ["blink", "dim", "steady"]


class MissionLedState(EventState):
    '''
    Implements a state that will be used to command mission LED from main pod.

    -- led_mode 	str 	        LED mode options in '':
                                    'blink',
                                    'dim',
                                    'steady'

    -- led_color    str             Color option shown by LED in '':
                                    'red',
                                    'blue',
                                    'green',
                                    'yellow',
                                    'white',
                                    'none'

    <= done					        Indicates that LED cmd is sent
    <= failed                       Indicates that user entered parameter is wrong
    '''

    def __init__(self, led_mode, led_color):
        super().__init__(outcomes=['failed', 'done'])

        # Proxy Publisher
        self._mission_led_topic = '/io/mission_indicator_led'
        self._pub = ProxyPublisher({self._mission_led_topic: mission_led_cmd})

        # Parameters
        self.mode = str(led_mode)
        self.color = str(led_color)
        self.mission_led_msg = mission_led_cmd()

        self.fail = False

    def execute(self, userdata):
        # Fail condition
        if self.fail:
            return 'failed'
        else:
            return 'done'

    def on_enter(self, userdata):
        self.fail = True  # Assume failure and clear the flag if all checks passed with success

        # Color
        if self.color not in color_list:
            Logger.loginfo('[MissionLedState] entered color parameter is wrong')
            return
        # Mode
        if self.mode not in mode_list:
            Logger.loginfo('[MissionLedState] entered mode parameter is wrong')
            return

        # Mission led command to publish
        self.mission_led_msg.mode = self.mode
        self.mission_led_msg.color = color_list[self.color]
        self._pub.publish(self._mission_led_topic, self.mission_led_msg)
        Logger.loginfo(f'[MissionLedState] {self.mission_led_msg.mode}'
                       f'mode & {self.mission_led_msg.color} color code is sent')

        self.fail = False