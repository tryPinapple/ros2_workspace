#!/usr/bin/env python3                                                                                                                 


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from vision_msgs.msg import Detection2DArray, VisionInfo


# hardcoded camera resolution
# x = 0 on the left & y = 0 in the top
X_RESOLUTION = 1920.0
Y_RESOLUTION = 1080.0


'''
Created on 27.10.2020

@author: Thomas Germain
'''


class ObjectAlignment(EventState):
    '''
    look for an object position and send directional outcomes to align with it

    -- searched_object      string  name of the object between '' ('object name')

    -- minimum_score        float   minimum accuracy score you look for 0 to 100

    -- center_precision     float   acceptable pixel away from center

    -- max_align_time       float   maximum time before state fail

    -- max_iterations       int     maximum number of times this states can run before it fail (None = disabled)

    ># iterations_in        int     number of iteration receive

    #< iterations_out       int     send the number of iteration already done

    <= finished					    You are align with the object.

    <= failed                       Indicates that the state timed out

    <= object_up				    the object is in the upper camera region

    <= object_down			        the object is in the lower camera region

    <= object_left				    the object is in the left camera region

    <= object_right				    the object is in the right camera region

    <= too_many_tries               Indicates that the state failed because it reached max iterations

    '''

    def __init__(self, searched_object, minimum_score, center_precision, max_align_time, max_iterations):
        super().__init__(outcomes=['finished', 'failed', 'too_many_tries', 'object_up', 'object_down', 'object_left',
                                   'object_right'], input_keys=['iterations_in'], output_keys=['iterations_out'])
        self._sub = ProxySubscriberCached({'detectnet/detections': Detection2DArray,
                                          'detectnet/vision_info': VisionInfo})
        self.item_searched = str(searched_object)
        self.max_time = rospy.Duration(max_align_time)
        self.pixel_precision = center_precision
        self.min_score = minimum_score/100
        self.fail = False
        self.max_iterations = max_iterations

    def execute(self, userdata):
        # iterations
        if self.max_iterations and userdata.iterations_in >= self.max_iterations:
            return 'too_many_tries'

        # timer condition
        if (rospy.Time.now() - self.start_time) > self.max_time or self.fail:
            Logger.logwarn('ObjectAlignment failed')
            return 'failed'

        # ros messages
        if self._sub.has_msg('detectnet/detections'):
            cam = self._sub.get_last_msg('detectnet/detections')

            # for loop containing all detections
            for detection in cam.detections:
                detect_id = detection.results[0].id
                detect_score = detection.results[0].score
                center_x = detection.bbox.center.x
                center_y = detection.bbox.center.y

                # exit conditions
                if self.items_labels[detect_id] == self.item_searched and detect_score >= self.min_score:
                    if center_x < ((X_RESOLUTION/2) - self.pixel_precision): #object to the left
                        return 'object_left'
                    elif center_x > ((X_RESOLUTION/2) + self.pixel_precision): #object to the right
                        return 'object_right'
                    elif center_y < ((Y_RESOLUTION/2) - self.pixel_precision): #object is up
                        return 'object_up'
                    elif center_y > ((Y_RESOLUTION/2) + self.pixel_precision): #object is down
                        return 'object_down'
                    else:
                        return 'finished'

    def on_enter(self, userdata):
        # start timer
        self.start_time = rospy.Time.now()

        # get ros param database location for detection labels
        if self._sub.has_msg('detectnet/vision_info'):
            labels_location = self._sub.get_last_msg('detectnet/vision_info')

            # store objects labels from ros parameter server
            self.items_labels = rospy.get_param(labels_location.database_location)

        else:
            Logger.loginfo('ObjectAlignment failed, path to labels not found in ROS param server')
            self.fail = True

    def on_exit(self, userdata):
        # iterations
        userdata.iterations_out = userdata.iterations_in + 1
        self.fail = False
        Logger.loginfo('ObjectAlignment reached an outcome and is exiting')

    def on_stop(self):
        pass
