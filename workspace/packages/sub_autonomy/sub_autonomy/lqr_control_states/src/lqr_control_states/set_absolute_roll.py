
#!/usr/bin/env python3
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from math import pi


# Import control system action
from asuqtr_control_node.msg import ControlAction, ControlGoal
from actionlib_msgs.msg import GoalStatus


'''
Created on 017.10.2020

@author: Thomas Germain
'''


class SetAbsoluteRoll(EventState):
    '''
    Control any absolute angle

    -- set_roll_degree      int       go to roll angle -180 to 180  

    -- max_time             float     maximum time to change the angle

    <= finished				          Angle position set.

    <= failed                         Indicates that the state timed out

    '''

    def __init__(self, set_roll_degree, max_time):
        super().__init__(outcomes=['finished', 'failed'])
        self.set_roll = set_roll_degree * pi / 180
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
            Logger.logwarn('SetAbsoluteRoll timed out')
            return 'failed'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)

            # Based on the result, decide which outcome to trigger.
            if status == GoalStatus.SUCCEEDED:
                return 'finished'
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
        goal.absolute = True
        goal.roll = self.set_roll
        goal.pitch = float('nan')
        goal.yaw = float('nan')
        goal.x = float('nan')
        goal.y = float('nan')
        goal.z = float('nan')

        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            Logger.logwarn('Failed to send the Control command:\n%s' % str(e))
            self.fail = True

    def on_exit(self, userdata):
        self.fail = False
        Logger.loginfo('SetAbsoluteRoll reached an outcome and is exiting')
        self.cancel_active_goals()

    def on_stop(self):
        self.cancel_active_goals()

    def cancel_active_goals(self):
        if self._client.is_available(self._topic):
            if self._client.is_active(self._topic):
                if not self._client.has_result(self._topic):
                    self._client.cancel(self._topic)
                    Logger.loginfo('Cancelled control_action_server active action goal.')

