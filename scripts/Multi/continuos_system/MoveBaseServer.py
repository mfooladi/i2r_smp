#! /usr/bin/env python

import rospy
import roslib
import actionlib
import numpy as np
import math
from geometry_msgs.msg import (Twist, Vector3)
from i2r_smp.msg import move_baseAction, move_baseActionFeedback, move_baseActionResult

# vel = Twist()

class MoveBaseServer(object):
    # create messages that are used to publish feedback/result
    # _goal = move_baseAction.action_goal
    _feedback = move_baseActionFeedback
    _result = move_baseActionResult

    def __init__(self, name):
        rospy.loginfo('inited')
        # self._action_name = name
        self._as = actionlib.SimpleActionServer('mb', move_baseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        rospy.loginfo('called back')
        r = rospy.Rate(3)
        success = True

        # publish info to the console for the user
        # rospy.loginfo('some strings go here to show to the user if needed')

        # start executing the action
        b = 0.25

        alpha = math.atan2(goal.y_current, goal.x_current)
        theta = goal.theta_current
        vel_linear = -np.cos(alpha) * np.cos(theta) - np.sin(alpha) * np.sin(theta)
        vel_angular = 1 / b * (-np.sin(alpha) * np.cos(theta) + np.cos(alpha) * np.sin(theta))
        # self._feedback = [vel_linear, vel_angular]
        # self._feedback.v_x = vel_linear
        # self._feedback.v_z = vel_angular
        self._feedback.v_x = goal.x_current
        self._feedback.v_z = goal.y_current
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('vel linear , vel angular: %f %f: ' % (self._feedback.v_x, self._feedback.v_z))
        r.sleep()

        # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i - 1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
        self._result.goal_reached = True
        # check if reached
        # if np.sqrt(abs(goal.x_current) ** 2 + abs(goal.y_current) ** 2) < 0.2:
        #     rospy.loginfo('goal reached')
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('move_base_server')
    server = MoveBaseServer(rospy.get_name())
    rospy.spin()