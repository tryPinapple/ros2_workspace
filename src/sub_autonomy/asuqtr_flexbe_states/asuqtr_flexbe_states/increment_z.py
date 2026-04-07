#!/usr/bin/env python3
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# Import control system action
from asuqtr_control_node.msg import ControlAction, ControlGoal
from actionlib_msgs.msg import GoalStatus


class IncrementZ(EventState):
    '''
    This state allows to move the AUV on a specified distance on z axis.

    -- add_z_distance      float32       Z distance to reach in meters

    -- max_time            float     Maximum time to travel the distance in s

    <= reached					     Z distance incrementation is reached

    <= failed                        Indicates that the state timed out

    '''

    def __init__(self, add_z_distance, max_time):
        super().__init__(outcomes=['reached', 'failed'])
        self.add_z = add_z_distance
        self.max_time = rospy.Duration(max_time)
        self._topic = 'control_action_server'
        self._client = ProxyActionClient({self._topic: ControlAction})
        self.fail = False

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self.fail:
            return 'failed'

        # Check if timed out
        if (rospy.Time.now() - self.start_time) > self.max_time:
            Logger.logwarn('IncrementZ timed out')
            return 'failed'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)

            # Based on the result, decide which outcome to trigger.
            if status == GoalStatus.SUCCEEDED:
                return 'reached'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self.fail = True
                return 'failed'

    def on_enter(self, userdata):
        # start timer
        self.start_time = rospy.Time.now()

        # Create the goal.
        goal = ControlGoal()
        goal.absolute = False
        goal.z = self.add_z

        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the Control command:\n%s' % str(e))
            self.fail = True

    def on_exit(self, userdata):
        self.fail = False
        Logger.loginfo('IncrementZ reached an outcome and is exiting')
        self.cancel_active_goals()

    def on_stop(self):
        Logger.loginfo('Behavior ended or IncrementZ state was preempted')
        self.cancel_active_goals()

    def cancel_active_goals(self):
        if self._client.is_available(self._topic):
            if self._client.is_active(self._topic):
                if not self._client.has_result(self._topic):
                    self._client.cancel(self._topic)
                    Logger.loginfo('Cancelled control_action_server active action goal.')