#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from math import pi

# Import control system action
from asuqtr_control_node.msg import ControlAction, ControlGoal
from actionlib_msgs.msg import GoalStatus


class YawRight(EventState):
    """
    This state allows to rotate the AUV positively around the referential Z axis: If
    The AUV is at 0 on roll and pitch, this is the same as "turning right" in an automotive.

    -- target_angle int 	Target angle in degrees

    <= done					Target angle was reached
    <= failed				Did not reach target.

    """

    def __init__(self, target_angle):
        super().__init__(outcomes=['done', 'failed'])

        # Transfer angles to rad
        self._rad_target_angle = abs(target_angle) * pi / 180

        # Create the action client when building the behavior.
        # This will cause the behavior to wait for the client before starting execution
        # and will trigger a timeout error if it is not available.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        self._topic = 'control_action_server'
        self._client = ProxyActionClient({self._topic: ControlAction})  # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._failed = False

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._failed:
            return 'failed'

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)

            # Based on the result, decide which outcome to trigger.
            if status == GoalStatus.SUCCEEDED:
                return 'done'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'

    # If the action has not yet finished, no outcome will be returned and the state stays active.

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.
        # Make sure to reset the error state since a previous state execution might have failed
        self._failed = False

        # Create the goal.
        goal = ControlGoal()
        goal.absolute = True
        goal.yaw = self._rad_target_angle

        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure does not necessarily cause a behavior failure, it is recommended to only print
            # warnings, not errors. Using a linebreak before appending the error log enables the operator to collapse
            # details in the GUI.
            Logger.logwarn('Failed to send the Control command:\n%s' % str(e))
            self._failed = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state. A situation where the action would still
        # be active is for example when the operator manually triggers an outcome.
        Logger.loginfo('YawRight reached an outcome and is exiting')
        self.cancel_active_goals()

    def on_stop(self):
        Logger.loginfo('YawRight state was preempted')
        self.cancel_active_goals()

    def cancel_active_goals(self):
        if self._client.is_available(self._topic):
            if self._client.is_active(self._topic):
                if not self._client.has_result(self._topic):
                    self._client.cancel(self._topic)
                    Logger.loginfo('Cancelled control_action_server active action goal.')