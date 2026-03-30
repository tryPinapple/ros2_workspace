ASUQTR ROS Power Node
====================

A ROS node for ASUQTR measuring battery packs tensions via serial, from Arduinos to Nvidia Jetson Xavier.

This package serves the purpose reading serialized data coming from the Arduinos. Those 2 Arduinos are both connected 
to a single battery pack. They read and convert analogic tensions to digital before sending on the Nvidia Jetson Xavier. 
From there, the tension values are sent as messages on the ROS network. 


QuickStart Guide
----------------


####Requirements:
1. A Nvidia Jetson device OR a PC with ROS for ASUQTR installed:
[How to Install ROS for ASUQTR](https://confluence.asuqtr.com/display/SUBUQTR/How+to+install+ROS+for+ASUQTR)
2. Two Arduinos programmed with the arduino code in this package and plugged to your machine via USB.
3. Both Arduinos shall be respectively named **/dev/ttyACM0** and **/dev/ttyACM1**. To configure Ubuntu in a way
that it remembers those devices name, follow [this tutorial from ASUQTR's documentation](https://confluence.asuqtr.com/display/SUBUQTR/Associate+a+name+to+a+USB+device+in+Ubuntu)

#### Run for standalone testing :

* <pre><code>roslaunch asuqtr_power_node tunning_lqr.launch</code></pre>
* **Ctrl+C** to quit

#### Run with the whole ASUQTR system:
_On a Jetson device_
* <pre><code>roslaunch asuqtr_mission_management jetson.launch</code></pre>
_On a standard PC_
* <pre><code>roslaunch asuqtr_mission_management pc.launch</code></pre>
* **Ctrl+C** to quit

Overview 
--------

#### Power node

This node is basically an adapter which parses messages from serial JSON data to ROS messages on the ROS network. 
It provides information about the cell tensions, leaks, etc. of both battery.

The protocol used to deserialize data from the arduino reads data until a specific TERMINATOR is read. After that,
 a JSON strings are converted to the correct ROS messages format and sent ont the network. Both battery packs are 
 implemented as independent threads to allow a minimum of fault tolerance if, for any reason, one of the two Arduinos
  fails.

#### power_node.launch

This launch file starts the node with the required ROS params if needed. If the needs for the project change, you should modify those values. 

#### Arduino
This directory contains the JSON and LEDs arduino libraries used in the pcbpod_arduino.ino file. This last files serves the
 purpose of reading analog inputs and giving LED visual feedback from the battery pods.