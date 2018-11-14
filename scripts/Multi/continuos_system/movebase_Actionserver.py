#! /usr/bin/env python
import rospy
import roslib
import actionlib
import numpy as np
import math
import vector_fild_new as vf

from geometry_msgs.msg import (Twist, Vector3)
from i2r_smp.msg import move_baseAction, move_baseActionFeedback, move_baseActionResult
from std_msgs.msg import String

global cmd, agent
cmd = Twist()

class MoveBaseServer(object):
    # create messages that are used to publish feedback/result

    _feedback = move_baseActionFeedback
    _result = move_baseActionResult

    def __init__(self, name):
        rospy.loginfo('inited')
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, move_baseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.initialized = False

    def execute_cb(self, goal):
        global initialized, pub_ugv_cmd, agent
        # self._feedback.feedback_vel_published = False
        # self._as.publish_feedback(self._feedback)
        # self._result.result_vel_published = self._feedback.feedback_vel_published
        # helper variables
        r = rospy.Rate(20)
        success = True
        p0 = [goal.x_current, goal.y_current]
        if goal.x_current != 0 and not self.initialized:
            self.initialized = True
            agent = vf.Robot(geometry_name=vf.r1_geometry, agent_radius=100, agent_id=1, v_max=.5, goals_file=vf.r1_goals, init_pos=p0)
            vf.discrete_planner(geometry_name=agent.geometry_name, init_position=p0, goals_file=agent.goals_file)
            print 'agent made'
            rospy.loginfo(agent.current_position)

            # rospy.loginfo('task info:', 'states to go:', str(agent.patch_length))

        # start executing the action
        vel = agent.move(current_pos=[goal.x_current, goal.y_current], current_state=agent.current_state, current_patch=agent.current_patch)
        # print 'vel', vel, type(vel)

        if agent.is_finished:
            print 'finished'
            vel = [0., 0.]

        b = 0.25
        alpha = math.atan2(p0[1], p0[0])
        theta = goal.theta_current
        cmd.linear.x = vel[0] * np.cos(theta) + vel[1] * np.sin(theta)
        cmd.angular.z = 1 / b * (vel[1] * np.cos(theta) - vel[0] * np.sin(theta))
        pub_ugv_cmd.publish(cmd)
        # pub_ugv_cmd.publish(cmd)
        # rospy.loginfo('%f %f' % (cmd.linear.x, cmd.angular.z))

        self._feedback.feedback_vel_published = True
        self._as.publish_feedback(self._feedback)
        self._result.result_vel_published = self._feedback.feedback_vel_published
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('move_base_server')
    server = MoveBaseServer(rospy.get_name())
    rate = rospy.Rate(50)
    pub_ugv_cmd = rospy.Publisher('/HKB/cmd_velGA', Twist, queue_size=100)

    rospy.spin()