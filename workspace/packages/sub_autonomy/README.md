ASUQTR Mission Management
====================

The main entry point of all the ROS package for ASUQTR. It contains launch files designed to start 2 possible runtime 
setups :
* Jetson device setup  _(ideal setup, requires a jetson device)_
* Standard PC setup  _(alternative setup when you do not have access to a jetson device)_

This package also contains various nodes to help achieve management of missions for the RoboSub competition.

QuickStart Guide
----------------

####Requirements:
1. A Nvidia Jetson device or standard PC with ROS for ASUQTR installed:
[How to Install ROS for ASUQTR](https://confluence.asuqtr.com/display/SUBUQTR/How+to+install+ROS+for+ASUQTR)

#### Standard PC setup (without nodes requiring hardware buses):

* <pre><code>roslaunch asuqtr_mission_management pc.launch</code></pre>
* **Ctrl+C** to quit

#### Jetson device setup:
* <pre><code>roslaunch asuqtr_mission_management jetson.launch</code></pre>
* **Ctrl+C** to quit


ASUQTR-Dashboard
--------
When using any of those 2 setups, you'll want to use the ASUQTR-Dashboard to gain access well presented feedback
from components of the whole AUV's system. To do so :

1. Reach the ASUQTR-Dashboard by typing **localhost** or your **machine's IP address** in your web browser
2. Plug in your Xbox/Logitech/PS4 controller and have fun

_Note: As you will see on the Dashboard, there is an alternative (and more convinient) way to start the whole ASUQTR 
ROS system, which is the ROS START button_

Overview 
--------

#### manual_mode node
This node acts as a manager for the Xbox controller commands sent through the ASUQTR-dashboard. 
It allows multiple useful functions for pool testing :
* Enable/Disable LQR control for tuning and safety kill switch
* Allow raw throttle commands when LQR control is disabled
* Kill switch for the currently running autonomous behavior

#### mission_management node
*not implemented yet, shall be used as a top layer to launch mission_behaviors*

#### testing_mode.yaml
This file contains parameters for the node which are loaded on runtime. Usually, you only need to change these values in real life testing with the AUV.

Example values contained in this file are :

* behavior running on launch (Bool)

* angular speed factor for manual mode (coefficient)

* joystick dead zone for controller error (coefficient)
