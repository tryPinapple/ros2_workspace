#!/usr/bin/env python3
import concurrent
import logging
import queue
import struct
import threading
from concurrent.futures.thread import ThreadPoolExecutor
from time import sleep

import serial

from matplotlib.pyplot import colorbar, figure, grid, imshow, pcolormesh, plot, show, subplot
from mpl_toolkits.mplot3d import Axes3D
from numpy import array, empty, linspace, pi, meshgrid
from serial_data import NotAcknowledgeError, PingDeviceData, PingDeviceInfo, PingNotAck, PingProtocolVersion, SerialData
import rospy
from sensor_msgs.msg import MultiEchoLaserScan, Image

# Message IDs
ID_NOT_ACK = 2
ID_DEVICE_INFO = 4
ID_HANDSHAKE = 5
ID_DEVICE_ID = 2000
ID_DEVICE_DATA = 2300
ID_RESET = 2600
ID_TRANSDUCER = 2601
ID_AUTO_TRANSMIT = 2602
ID_MOTOR_OFF = 2903

# Ping360 default values for 50m range in salty water
DEFAULT_FREQUENCY = 740  # kHz
DEFAULT_GAIN = 0  # 0 : Low      1 : Normal      2 : High
DEFAULT_TRANSMIT_DURATION = 500  # microseconds
DEFAULT_SAMPLE_PERIOD = 2560 # 64 microseconds
DEFAULT_NUMBER_SAMPLES = 1024

# Serial port constant definitions
PORT = '/dev/ttyS3'
BAUDRATE = 115200

class SerialData:
    def __init__(self, header):
        self.payload_length = (header[3] << 8) + header[2]
        self.message_id = (header[5] << 8) + header[4]
        self.raw_data = list(header)
        logging.debug(
            f'Packet header: Length = {self.payload_length}, ID = {self.message_id}'
        )

    def read_payload(self, serial_object):
        rest = serial_object.read(self.payload_length + 2)
        self.payload = rest[:-2]
        self.checksum = rest[-2:]
        self.raw_data.extend(list(rest[:-2]))
        self._validate_checksum()

    def _validate_checksum(self):
        sum = 0
        for value in self.raw_data:
            sum += value
        chk = int(unpack('<H', self.checksum)[0])
        if chk != sum:
            logging.debug("Checksum failed")
            raise ChecksumError(
                "Checksum validation of incoming serial data failed")
        else:
            logging.debug("Checksum passed")
            return True


class PingProtocolVersion:
    def __init__(self, payload_data):
        self.version_major = payload_data[0]
        self.version_minor = payload_data[1]
        self.version_patch = payload_data[2]

    def __str__(self):
        return f'Found a device using Ping protocol {self.version_major}.{self.version_minor}.{self.version_patch}'


class PingDeviceInfo:
    def __init__(self, payload_data):
        if payload_data[0] == 1:
            self.device_type = "Ping Echosounder"
        elif payload_data[0] == 2:
            self.device_type = "Ping360"
        else:
            self.device_type = "Unknown"
        self.device_revision = payload_data[1]
        self.firmware_version_major = payload_data[2]
        self.firmware_version_minor = payload_data[3]
        self.firmware_version_patch = payload_data[4]

    def __str__(self):
        return f'Connected to {self.device_type} revision {self.device_revision} firmware {self.firmware_version_major}.{self.firmware_version_minor}.{self.firmware_version_patch}'


class PingDeviceData:
    def __init__(self, payload_data):
        if payload_data[1] == 0:
            self.gain_setting = "Low"
        elif payload_data[1] == 1:
            self.gain_setting = "Normal"
        elif payload_data[1] == 2:
            self.gain_setting = "High"
        self.angle = (payload_data[3] << 8) + (payload_data[2])
        self.transmit_duration = (payload_data[5] << 8) + (payload_data[4])
        self.sample_period = ((payload_data[7] << 8) +
                              (payload_data[6])) * (25.0 / 1000.0)
        self.transmit_frequency = (payload_data[9] << 8) + (payload_data[8])
        self.nb_samples = (payload_data[11] << 8) + (payload_data[10])
        self.data_length = (payload_data[13] << 8) + (payload_data[12])
        self.data = list()
        for i in range(self.data_length):
            self.data.append(payload_data[i + 13])
            # if i == self.data_length - 1:
            #     self.data = array(self.data)

    def __str__(self):
        return f'------Device Data------\n\t  Gain setting : \t{self.gain_setting}\n\t  Angle : \t\t{self.angle} gradians\n\t  Transmit Duration : \t{self.transmit_duration} microseconds\n\t  Sample Period : \t{self.sample_period} microseconds\n\t  Transmit Frequency : \t{self.transmit_frequency} kHz\n\t  Number of Samples : \t{self.nb_samples}\n\t  Data Length : \t{self.data_length} bytes'


class PingNotAck:
    def __init__(self, payload_data):
        self.id = (payload_data[1] << 8) + payload_data[0]
        self.message = struct.unpack("%ds" % (len(payload_data) - 2),
                                     payload_data[2:])[0].decode("utf-8")

    def __str__(self):
        return f'ID {self.id}: {self.message}'


class ChecksumError(Exception):
    def __init__(self, message):
        self.message = message


class NotAcknowledgeError(Exception):
    def __init__(self, message):
        self.message = message

class PingSerial:
    def __init__(self, port, baudrate):
        self.heatmap = list()
        rospy.loginfo(f'Opening serial port {port} at {baudrate} bauds'
                     )  # rospy.loginfo()
        self.com_object = serial.Serial(port, baudrate)
        self.data = list()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, value, traceback):
        self.com_object.close()
        if exc_type:
            print(exc_type, value)
        return True

    def read_producer(self, queue, event):
        while not event.is_set():
            serial_data = SerialData(self.com_object.read(8))
            serial_data.read_payload(self.com_object)
            rospy.logdebug("Read producer got message: %s",
                          serial_data.payload)  # rospy.logdebug()
            queue.put(serial_data)

    def read_consumer(self, queue, event):
        while not event.is_set():
            serial_data = queue.get()
            rospy.logdebug(
                "Read consumer storing message: %s (size=%d) (ID=%d)",
                serial_data.payload, queue.qsize(),
                serial_data.message_id)  # rospy.logdebug()
            if serial_data.message_id == ID_HANDSHAKE:
                self.protocol = PingProtocolVersion(serial_data.payload)
                rospy.loginfo(self.protocol)  # rospy.loginfo()
            elif serial_data.message_id == ID_DEVICE_INFO:
                self.device_info = PingDeviceInfo(serial_data.payload)
                rospy.loginfo(self.device_info)  # rospy.loginfo()
            elif serial_data.message_id == ID_DEVICE_DATA:
                self.current_device_data = PingDeviceData(serial_data.payload)
                if self.append_data:
                    self.data.append(self.current_device_data.data)
                rospy.loginfo(self.current_device_data)  # rospy.loginfo()
            elif serial_data.message_id == ID_NOT_ACK:
                not_ack = PingNotAck(serial_data.payload)
                rospy.logwarn(not_ack)  # rospy.logwarn()
                raise NotAcknowledgeError(not_ack)

        rospy.logdebug(
            "Read consumer received event. Exiting")  # rospy.logdebug()

    def handshake(self):
        self._get(ID_HANDSHAKE, 4)

    def get_device_info(self):
        self._get(ID_DEVICE_INFO, 6)

    def get_device_data(self):
        self._get(ID_DEVICE_DATA, 14)

    def reset(self):
        self._request = [66, 82, 2, 0, 0, 2]
        self._request = self._request[:4] + list(struct.pack(
            '<H', ID_RESET)) + self._request[4:]
        self._request.extend([0, 0])
        self._request = bytearray(self._request)
        self._write_serial_payload()
        rospy.loginfo(f'Resetted the {self.device_info.device_type} device'
                     )  # rospy.loginfo()

    def set_device_ID(self):
        pass

    def _device_ID_callback(self):
        pass

    def transducer_adjust(self, gain, angle, transmit_duration, sample_period,
                          transmit_frequency, nb_samples):
        self._request = [66, 82, 14, 0, 0, 2]
        self._request = self._request[:4] + list(
            struct.pack('<H', ID_TRANSDUCER)) + self._request[4:]
        self._request.extend([1, gain])
        self._request.extend(struct.pack('<H', angle))
        self._request.extend(struct.pack('<H', transmit_duration))
        self._request.extend(struct.pack('<H', sample_period))
        self._request.extend(struct.pack('<H', transmit_frequency))
        self._request.extend(struct.pack('<H', nb_samples))
        self._request.extend([1, 0])
        self._request = bytearray(self._request)
        self._write_serial_payload()

    '''
    Auto Transmit message (2602) is not implemented yet on Ping360

    def auto_transmit(self, start_angle, stop_angle, num_steps, delay):
        self._request = [66, 82, 6, 0, 0, 2]
        self._request = self._request[:4] + list(
            struct.pack('<H', ID_AUTO_TRANSMIT)) + self._request[4:]
        self._request.extend(struct.pack('<H', start_angle))
        self._request.extend(struct.pack('<H', stop_angle))
        self._request.extend([num_steps, delay])
        self._request = bytearray(self._request)
        self._calc_checksum()
        self.com_object.write(self._request)
        logging.debug(f'Auto transmit write: {list(self._request)}')  # rospy.logdebug()
    '''

    def motor_off(self):
        self._get(ID_MOTOR_OFF, 1)

    def _get(self, message_id, length):
        self._request = bytearray([66, 82, 2, 0, 6, 0, 0, 2])
        self._request.extend(struct.pack('<H', message_id))
        self._write_serial_payload()

    def _write_serial_payload(self):
        self._calc_checksum()
        self.com_object.write(self._request)
        rospy.logdebug(f'Writing {self._request}')  # rospy.logdebug()

    def _calc_checksum(self):
        sum = 0
        for value in self._request:
            sum += value
        self._request.extend(struct.pack('<H', sum))

    def periphery_transmit(self, start_angle, stop_angle, increment, delay):
        for deg in range(start_angle, stop_angle, increment):
            self.append_data = True
            self.transducer_adjust(self._gain, deg, self._transmit_duration,
                                   self._sample_period,
                                   self._transmit_frequency, self._nb_samples)
            rospy.sleep(delay)

    def service(self, req):
        periphery_transmit(req.angle_min, req.angle_max, req.angle_increment, req.time_increment)
        return self.data

    def set_range(self, range_in_meters):
        self.range = range_in_meters  # Range = samplePeriod * nb_samples * speed_of_sound/2

    def set_speed_of_sound(self, speed_of_sound):
        self.speed_of_sound = speed_of_sound

    def update_transmit_parameters(self):
        self._gain = DEFAULT_GAIN
        self._transmit_duration = DEFAULT_TRANSMIT_DURATION
        self._sample_period = DEFAULT_SAMPLE_PERIOD
        self._transmit_frequency = DEFAULT_FREQUENCY
        self._nb_samples = DEFAULT_NUMBER_SAMPLES

    def get_time_between_control(self):
        time_seconds = (self._nb_samples * (self._sample_period * 25 / 1000) / 1000000)
        return time_seconds + (time_seconds * 0.1) # Adding 10% to required delay


def degrees_to_gradians(degrees):
    return degrees / 0.9


def sonar_node():
    rospy.init_node("sonar_node", log_level=rospy.DEBUG)
    sonar_message = MultiEchoLaserScan()
    image_message = Image()
    raw_data_pub = rospy.Publisher('sonar_echo', MultiEchoLaserScan, queue_size=5)
    image_pub = rospy.Publisher('image_sonar_echo', Image, queue_size=1)
    image_mode = rospy.get_param('~image_send_mode')
    raw_data_mode = rospy.get_param('~raw_data_send_mode')
    mode = rospy.get_param('~mode')

    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
    pipeline = queue.Queue(maxsize=200)
    event = threading.Event()

    with PingSerial(PORT, BAUDRATE) as ping:
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            executor.submit(ping.read_producer, pipeline, event)
            executor.submit(ping.read_consumer, pipeline, event)

            ping.set_speed_of_sound(rospy.get_param('~speed_of_sound'))  # Default : 1531m/s
            ping.set_range(rospy.get_param('~ping_range'))  # Default : 50m

            rospy.loginfo(
                "Searching for Ping protocol devices...")  # rospy.loginfo()
            ping.handshake()
            rospy.sleep(10)
            ping.get_device_info()
            rospy.sleep(10)
            ping.update_transmit_parameters()
            ping.append_data = False
            ping.transducer_adjust(0, 0, 500, 20000, 740, 1024)
            rospy.sleep(4)

            if mode == "service":
                sonar_service = rospy.Service('sonar_scan',
                                              asuqtr_sonar_node.srv.sonar_scan,
                                              ping.service)
                rospy.spin()
            elif mode == "automatic":
                angle_end = degrees_to_gradians(360)
                increment = 2
                while not rospy.is_shutdown:
                    ping.data = list()
                    ping.periphery_transmit(start_angle=0,
                                            stop_angle=angle_end,
                                            increment=increment,
                                            delay=get_time_between_control())
                    if raw_data_send_mode == True:
                        sonar_message.angle_min = 0
                        sonar_message.angle_max = 2 * pi
                        sonar_message.angle_increment = increment
                        sonar_message.time_increment = get_time_between_control()
                        sonar_message.intensities = ping.data
                        raw_data_pub.publish(sonar_message)
                    if image_send_mode == True:
                        rad = linspace(0, 10, ping._nb_samples)
                        a = linspace(0, angle_end * pi / 200, angle_end)
                        r, th = meshgrid(rad, a)
                        subplot(projection="polar")
                        pcolormesh(th, r, array(ping.data))
                        plot(a, r, ls='none', color='k')
                        grid()
                        colorbar()
                        # TODO : Transform the plot into a usable image without losing the radial heatmap format
                        image_pub.publish(image_message)
                    
                    rospy.spin()

                rospy.logdebug("Main: about to set event")  # rospy.logdebug()
                event.set()
            else:
                raise ValueError(mode)


if __name__ == "__main__":
    sonar_node()
