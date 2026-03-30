#!/usr/bin/env python3
import json
import rospy
import rostopic
import roslib
from contextlib import ExitStack
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32
from asuqtr_power_node.msg import pod_actuator_cmd

import serial
from serial.threaded import LineReader, ReaderThread

# Port names need to be defined in function of arduino names
DRIVER_PORT = '/dev/ttyACM0'
HELPER_PORT = '/dev/ttyACM1'


class PcbPodArduino(LineReader):
    """
    Implement a protocol for reading & write serial data from/to ASUQTR's
    PCB pod arduino. This protocol must be used by the ReaderThread class
    in conjunction with a defined serial port
    """

    def connection_made(self, transport):
        super().connection_made(transport)
        self.name = self.transport.serial.name
        rospy.loginfo("Serial port %s successfully opened!", self.name)

    def handle_line(self, line):
        """
        Process one line - to be overridden by monkey patching
        """
        pass

    def connection_lost(self, exc):
        if exc:
            rospy.logerr("Serial port %s lost connection!", self.name)


def json_to_ros_msg(data):
    """
     Dict of ROS msgs which will be publish on a specified topic
     """
    json_data = json.loads(data)
    pod_ros_msgs = {}
    pod_ros_msgs['battery'] = BatteryState(
        cell_voltage=json_data["cell_voltage"])
    pod_ros_msgs['leak'] = Bool(
        data=json_data["leak_sensor"][0])
    pod_ros_msgs['temp'] = Float32(
        data=json_data["temp_sensor"][0])

    return pod_ros_msgs


def ros_msg_to_json(cmd, pod_actuator):
    """
    Dict of JSON strings which will be write to the Arduino
    """
    pod_json_msgs = {}
    pod_json_msgs['pod actuator'] = pod_actuator
    pod_json_msgs['actuator cmd'] = cmd

    return json.dumps(pod_json_msgs)


def open_ports(port_names):
    """
    Try to open every serial ports in the list 'port_names' and return a
    dict of opened ports
    """
    opened_ports = {}
    for name in port_names:
        try:
            port = serial.Serial(name)
            opened_ports[name] = port
        except serial.serialutil.SerialException as e:
            rospy.logerr("pod_node: Serial port %s couldn't be opened", name)
            rospy.logerr(e)

    return opened_ports


def pod_node():
    rospy.init_node('pod_node')

    # Pod_node publishers
    driver_battery_topic = rospy.Publisher('pod_node/battery_volt_driver',
                                         BatteryState,
                                         queue_size=1)
    helper_battery_topic = rospy.Publisher('pod_node/battery_volt_helper',
                                         BatteryState,
                                         queue_size=1)
    driver_leak_topic = rospy.Publisher('pod_node/battery_leak_driver',
                                      Bool,
                                      queue_size=10)
    helper_leak_topic = rospy.Publisher('pod_node/battery_leak_helper',
                                      Bool,
                                      queue_size=10),
    driver_temp_topic = rospy.Publisher('pod_node/battery_temp_driver',
                                      Float32,
                                      queue_size=10)
    helper_temp_topic = rospy.Publisher('pod_node/battery_temp_helper',
                                      Float32,
                                      queue_size=10)

    # Callback for Pod actuator topic subscriber
    def actuator_cb(msg):
        json_string = ros_msg_to_json(msg.cmd, msg.actuator)
        protocol.write_line(json_string)

    # Data handler functions for serial arduino comm
    def handle_driver_arduino(data):
        pod_ros_msgs = json_to_ros_msg(data)
        driver_battery_topic.publish(pod_ros_msgs['battery'])
        driver_leak_topic.publish(pod_ros_msgs['leak'])
        driver_temp_topic.publish(pod_ros_msgs['temp'])

    def handle_helper_arduino(data):
        pod_ros_msgs = json_to_ros_msg(data)
        helper_battery_topic.publish(pod_ros_msgs['battery'])
        helper_leak_topic.publish(pod_ros_msgs['leak'])
        helper_temp_topic.publish(pod_ros_msgs['temp'])

    handlers = {DRIVER_PORT: handle_driver_arduino,
                HELPER_PORT: handle_helper_arduino}

    topic_actuators = {DRIVER_PORT: '/pod_node/dropper_driver',
                       HELPER_PORT: '/pod_node/torpedo_helper'}

    # Open serial ports. Shut node if no port opened
    opened_ports = open_ports([DRIVER_PORT, HELPER_PORT])

    if not opened_ports:
        rospy.logwarn("Shutting down pod_node, no serial connection detected")
        return

    with ExitStack() as stack:
        # Start reader threads and monkey patch handler for opened ports
        for port, handler in handlers.items():
            if port in opened_ports:
                protocol = stack.enter_context(ReaderThread(opened_ports[port], PcbPodArduino))
                protocol.handle_line = handler
                msg_type = roslib.message.get_message_class(
                            rostopic.get_topic_type(topic_actuators.get(port))[0])
                rospy.Subscriber(topic_actuators.get(port), msg_type, actuator_cb)

        rospy.spin()


if __name__ == '__main__':
    pod_node()
