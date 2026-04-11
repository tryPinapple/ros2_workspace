#!/bin/bash
##############################################################################
# This script takes care of installing python dependencies and setting       #
# the correct Ubuntu user permissions for gpio. It does not manage ROS       #
# dependencies                                                               #
##############################################################################
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Install/Update python3's pip
sudo apt-get install python3-pip -y
python3 -m pip install --upgrade pip
# Install the Adafruit required drivers and NeoPixel SPI package for asuqtr_led_node
python3 -m pip install adafruit-blinka adafruit-circuitpython-busdevice adafruit-circuitpython-pypixelbuf \
                       adafruit-circuitpython-neopixel-spi
# Set Ubuntu user permissions on Jetson Device
sudo cp "$SCRIPT_DIR"/99-gpio.rules /etc/udev/rules.d/
# Activate new rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Do this magic trick found on NVIDIA forum to create spi device in /dev/
sudo modprobe spidev