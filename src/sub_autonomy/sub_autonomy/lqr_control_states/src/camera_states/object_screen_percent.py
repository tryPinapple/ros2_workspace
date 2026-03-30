#!/usr/bin/env python3


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from vision_msgs.msg import Detection2DArray, VisionInfo


# hardcoded camera resolution
# x = 0 on the left & y = 0 in the top
X_RESOLUTION = 1920.0 
Y_RESOLUTION = 1080.0 
CAMERA_BOX = X_RESOLUTION * Y_RESOLUTION


'''
Created on 12.11.2020

@author: Thomas Germain
'''


class ObjectScreenPercent(EventState):
    '''
    look at an object size and send x movement outcomes to adjust the screen size it occupy

    -- searched_object      string  name of the object between '' ('object name')

    -- minimum_score        float   minimum accuracy score you look for 0 to 100

    -- object_size_percent  float   size of the object in % of the total camera

    -- size_percent_range   float   acceptable error

    -- maximum_time         float   maximum time before state fail

    -- max_iterations       int     maximum number of times this states can run before it fail (None = disabled)

    ># iterations_in        int     number of iteration receive

    #< iterations_out       int     send the number of iteration already done

    <= finished					    You are at the object

    <= too_far					    object is too far

    <= too_close					object is too close

    <= failed                       object not found state timed out

    <= too_many_tries               Indicates that the state failed because it reached max iterations

    '''

    def __init__(self, searched_object, minimum_score, object_size_percent, size_percent_range,
                 maximum_time, max_iterations):
        super().__init__(outcomes=['finished', 'failed', 'too_many_tries', 'too_far', 'too_close'],
                         input_keys=['iterations_in'], output_keys=['iterations_out'])
        self._sub = ProxySubscriberCached({'detectnet/detections': Detection2DArray,
                                          'detectnet/vision_info': VisionInfo})
        self.item_searched = str(searched_object)
        self.max_time = rospy.Duration(maximum_time)
        self.min_score = minimum_score/100
        self.max_size = CAMERA_BOX*(object_size_percent/100 + size_percent_range/100)
        self.min_size = CAMERA_BOX*(object_size_percent/100 - size_percent_range/100)
        self.fail = False
        self.max_iterations = max_iterations

    def execute(self, userdata):
        # iterations
        if self.max_iterations and userdata.iterations_in >= self.max_iterations:
            return 'too_many_tries'

        # timer condition
        if (rospy.Time.now() - self.start_time) > self.max_time or self.fail:
            Logger.logwarn('ObjectPixelPercentSize failed')
            return 'failed'

        # ros messages
        if self._sub.has_msg('detectnet/detections'):
            cam = self._sub.get_last_msg('detectnet/detections')

            # for loop containing all detections
            for detection in cam.detections:
                box_size = (detection.bbox.size_x * detection.bbox.size_y)
                detect_id = detection.results[0].id
                detect_score = detection.results[0].score

                # exit conditions
                if self.items_labels[detect_id] == self.item_searched and detect_score >= self.min_score:
                    if box_size < self.min_size:
                        return 'too_far'
                    if box_size > self.max_size:
                        return 'too_close'
                    if box_size <= self.max_size and box_size >= self.min_size:
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
            Logger.loginfo('ObjectScreenPercent failed, path to labels not found in ROS param server')
            self.fail = True

    def on_exit(self, userdata):
        # iterations
        userdata.iterations_out = userdata.iterations_in + 1
        self.fail = False
        Logger.loginfo('ObjectScreenPercent reached an outcome and is exiting')

    def on_stop(self):
        pass
        


    

