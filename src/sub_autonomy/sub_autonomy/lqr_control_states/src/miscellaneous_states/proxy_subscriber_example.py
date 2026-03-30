#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
import rospy

# Import from ros API sensor_msgs
from sensor_msgs.msg import Imu


class ProxySubscriberExample(EventState):
    '''
    This state uses proxy subscriber to get data from IMU topic and to use it to detect
    quaternion measurements changes. To do so, IMU quaternion measurements are compared
    within a specific interval of time to a specific percentage of deviation.

    -- pct_x_deviation   float    Difference desired between 2 x quaternion measures (%)

    -- pct_y_deviation   float    Difference desired between 2 y quaternion measures (%)

    -- pct_z_deviation   float    Difference desired between 2 z quaternion measures (%)

    -- pct_w_deviation   float    Difference desired between 2 w quaternion measures (%)

    -- compare_time      float    Time interval desired between 2 measures (s)

    -- execution_time    float    Maximum time allowed to determine IMU behavior (s)

    <= completed					   A x, y, z or w quaternion deviation has been detected

    <= failed                          Indicates that the state timed out

    '''

    def __init__(self, pct_x_deviation, pct_y_deviation, pct_z_deviation, pct_w_deviation, compare_time, execution_time):
        super().__init__(outcomes=['completed', 'failed'])

        # Subscribes on vectornav/IMU topic used by asuqtr_control_node
        self._imu_topic = 'vectornav/IMU'
        self._sub = ProxySubscriberCached({self._imu_topic: Imu})

        # Parameters declared
        self.pct_x = pct_x_deviation / 100
        self.pct_y = pct_y_deviation / 100
        self.pct_z = pct_z_deviation / 100
        self.pct_w = pct_w_deviation / 100
        self.time_interval = rospy.Duration(compare_time)
        self.max_time = rospy.Duration(execution_time)

        # Fails to subscribes to a topic
        self.fail = False

    def execute(self, userdata):
        # timer condition
        if (rospy.Time.now() - self.start_time) > self.max_time:
            Logger.logwarn('ProxySubscriberExample reached max time')
            return 'failed'

        if self.fail:
            return 'failed'

        # get ros IMU msg (quaternion measurement) after a specific time interval
        if (rospy.Time.now() - self.start_time) >= self.time_interval:
            Logger.loginfo('ProxySubscriberExample has reached the time interval value')
            if self._sub.has_msg(self._imu_topic):
                imu_data = self._sub.get_last_msg(self._imu_topic)
                quats = imu_data.orientation
                x = quats.x
                y = quats.y
                z = quats.z
                w = quats.w
                # exit condition
                if (x - self.x_init)/self.x_init >= self.pct_x:
                    Logger.loginfo('ProxySubscriberExample has detected quaternion x changes')
                    return 'completed'
                if (y - self.y_init) / self.y_init >= self.pct_y:
                    Logger.loginfo('ProxySubscriberExample has detected quaternion y changes')
                    return 'completed'
                if (z - self.z_init) / self.z_init >= self.pct_z:
                    Logger.loginfo('ProxySubscriberExample has detected quaternion z changes')
                    return 'completed'
                if (w - self.w_init) / self.w_init >= self.pct_w:
                    Logger.loginfo('ProxySubscriberExample has detected quaternion w changes')
                    return 'completed'

    def on_enter(self, userdata):
        # get ros IMU msg (quaternion measurement) at start time
        if self._sub.has_msg(self._imu_topic):
            imu_data = self._sub.get_last_msg(self._imu_topic)
            quats = imu_data.orientation
            self.x_init = quats.x
            self.y_init = quats.y
            self.z_init = quats.z
            self.w_init = quats.w
            Logger.loginfo('ProxySubscriberExample has detected msg on IMU topic')
            # start timer
            self.start_time = rospy.Time.now()
        # Fail to detect msg on the given topic
        else:
            Logger.loginfo('ProxySubscriberExample failed, no msg is detected on IMU topic')
            self.fail = True

    def on_exit(self, userdata):
        self.fail = False
        Logger.loginfo('ProxySubscriberExample reached an outcome and is exiting')

    def on_stop(self):
        pass