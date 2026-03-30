#!/usr/bin/env python3
import board
import rospy
import typing
from asuqtr_indicator_node.msg import LedCommand
from threading import Thread
import neopixel_spi as neopixel

PIXEL_ORDER = neopixel.GRB # Some NeoPixels have red and green reversed (GRB vs RGB)
COLOR_DEFAULT = 0xFFFFFF
COLOR_ERROR = 0xFF0000
COLOR_NONE = 0x000000
MAX_COLOR_BRIGHTNESS = 1.0
DEFAULT_COLOR_BRIGHTNESS = 0.5
BLINK_DELAY = 1
DIM_DELAY = 0.1


class AsuqtrIndicatorLeds(neopixel.NeoPixel_SPI):
    """ Allows to display light patterns for visual feedback of ASUQTR' AUV while in autonomous mode"""
    MAX_LOOP_HZ = 30
    NB_LED = 10
    def __init__(self, spi_bus, nb_led=NB_LED, pixel_order=PIXEL_ORDER,
                 auto_write=True):
        super().__init__(spi_bus, nb_led, pixel_order=pixel_order, brightness=1.0,
                         auto_write=auto_write)
        self.cmd: typing.Dict = {'special_mode_active': False,
                            'special_mode_id': 0,
                            'RGB_color': 0xFF00FF,
                            'blink': False,
                            'blink_frequency': 0,
                            'dim': False,
                            'dim_cycle_time': 0,
                            'brightness': 1.0
                                 }
        self.nb_led = nb_led
        self.led_pattern_task = Thread(target=self.display_led_pattern, daemon=True)
        self.led_pattern_task.start()
        self.special_modes: typing.Dict = {1:self.police_mode,
                                          2:self.party_mode,
                                          3:self.rainbow_mode,
                                          4:self.plane_blink_mode
                                          }

    def display_led_pattern(self):
        """ Execute patterns according to current led command received on ROS network. See neopixel pattern
        example here: https://learn.adafruit.com/neopixels-on-raspberry-pi/python-usage """
        # Pattern parameters
        special_mode = None
        rate = rospy.Rate(self.MAX_LOOP_HZ) # locks this thread at MAX_LOOP_HZ maximum
        loop_time = 1/self.MAX_LOOP_HZ # used to estimate time increment, fails if cpu bottleneck (I can live with this)
        next_color = ()
        blink_lapse = 0
        dim_lapse = 0
        brigh_lvls = 20
        bright_lvl_increment = 1/brigh_lvls
        bright_decreasing = True

        while not rospy.is_shutdown():
            # Process special mode request
            if self.cmd['special_mode_active']:
                try:
                    special_mode = self.special_modes.get(self.cmd['special_mode_id'])
                    special_mode()
                except (RuntimeError, Exception) as error:
                    if isinstance(error, RuntimeError):
                        rospy.logerr(f'Indicator {special_mode.__name__}  had a runtime error')
                    else:
                        rospy.logerr(f'special mode error happened:\n{str(error)}')
                continue

            # Process blinking parameters
            if self.cmd['blink']:
                if blink_lapse >= 1/(2*self.cmd['blink_frequency']):
                    blink_lapse = 0
                    self.fill(next_color)
                    next_color = (0,0,0) if next_color == self.cmd['RGB_color'] else self.cmd['RGB_color']

            # Process dimming parameters
            elif self.cmd['dim']:
                if dim_lapse >= self.cmd['dim_cycle_time']/brigh_lvls:
                    dim_lapse = 0
                    self.brightness += -bright_lvl_increment if bright_decreasing else bright_lvl_increment
                    if self.brightness <= 0:
                        bright_decreasing = False

            else:
                self.brightness = MAX_COLOR_BRIGHTNESS
                self.fill(self.cmd['RGB_color'])
            blink_lapse += loop_time
            dim_lapse += loop_time
            rate.sleep()

    def police_mode(self):
        pass

    def party_mode(self):
        pass

    def rainbow_mode(self):
        pass

    def plane_blink_mode(self):
        pass

    def handle_ros_command(self, cmd: LedCommand):
        """
        Method to be used as a callback for ROS subscriber events
        :param cmd: LedCommand Ros message to apply new displaying parameters to LEDs
        """
        # switch grb to RBG with bitshift
        grb = cmd.GRB_hex_color
        rgb = (((grb&0xFF00)>>2),((grb&0xFF0000)>>4),grb&0xFF)
        self.cmd = {'special_mode_active': cmd.special_mode_active,
                            'special_mode_id': cmd.special_mode_id,
                            'RGB_color': rgb,
                            'blink': cmd.blink,
                            'blink_frequency': cmd.blink_frequency,
                            'dim': cmd.dim,
                            'dim_cycle_time': cmd.dim_cycle_time,
                            'brightness': cmd.brightness/100  # % value to range(0, 1.0) value
                    }


def indicator_node():
    # Init ROS asuqtr_indicator_node
    rospy.init_node('indicator_node')
    # Init SPI NeoPixel Leds
    indicator_leds = AsuqtrIndicatorLeds(board.SPI())
    # Initiate ROS topic subscribers
    rospy.Subscriber('/indicator/led_command', LedCommand, indicator_leds.handle_ros_command)
    rospy.spin()


if __name__ == '__main__':
    indicator_node()
