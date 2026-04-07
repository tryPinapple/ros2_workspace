#!/usr/bin/env python3
import rospy

# Import control system action
from asuqtr_control_node.msg import ControlAction, ControlGoal
from actionlib_msgs.msg import GoalStatus
import actionlib


class ControlActionClient:

    def __init__(self, max_goal_reach_time):
        self.max_time = rospy.Duration(max_goal_reach_time)
        self._client = actionlib.SimpleActionClient('control_action_server', ControlAction)
        self.fail = False
        self.start_time = 0

        if self._client.wait_for_server(rospy.Duration(1)):
            rospy.loginfo('Server active')
        else:
            rospy.logerr('Server not active')

    def check_goal_reached(self):
        # Check if timed out
        if (rospy.Time.now() - self.start_time) > self.max_time:
            rospy.logwarn('Control action failed, could not reach control goal')
            self._client.cancel_all_goals()
            return False
        # Get action state from action server
        status = self._client.get_state()
        # Based on the result, decide which outcome to trigger.
        if status == GoalStatus.SUCCEEDED:
            rospy.logwarn('Control goal reached! You can crack a cold one')
            return True
        elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                        GoalStatus.RECALLED, GoalStatus.ABORTED]:
            self.fail = True
            rospy.logwarn('Control action failed, you should consider ending your life')
            return True

    def send_pos_increment(self, ned_ref, distance):
        # start timer
        self.start_time = rospy.Time.now()

        # Create and fill goal
        goal = ControlGoal()
        goal.absolute = ned_ref
        goal.x = distance

        try:
            self._client.send_goal(goal)
        except Exception as error:
            rospy.logwarn(f'Failed to send the Control command:\n{error}')
            self.fail = True


def control_action_client():
    rospy.init_node('Control_client_tester')
    client = ControlActionClient(5)
    while not rospy.is_shutdown():
        val = input('Enter 1 to increase body target position by 1 on x axis\n'
                    'Enter 2 to increase NED target position by 1 on x axis:\n')
        try:
            if int(val) == 1:
                client.send_pos_increment(ned_ref=True, distance=1)
            elif int(val) == 2:
                client.send_pos_increment(ned_ref=False, distance=1)
            else:
                raise ValueError
        except Exception as error:
            print(f"Bad input:\n{str(error)}")
        while client.check_goal_reached() is None:
            pass


if __name__ == '__main__':
    control_action_client()