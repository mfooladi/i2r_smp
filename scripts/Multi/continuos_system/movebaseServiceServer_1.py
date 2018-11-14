#!/usr/bin/env python

import rospy
import numpy as np

from i2r_smp.srv import *
import vector_fild_new as vf
from geometry_msgs.msg import Twist

initialized = False

class VectorField(vf.Robot):

    def __init__(self, current_pose):
        self.cmd = Twist()
        self.current_pose = Twist()
        self.current_pose = current_pose.position
        vf.discrete_planner(geometry_name = vf.r1_geometry, init_position = [self.current_pose.linear.x, self.current_pose.linear.y], goals_file = vf.r1_goals)
        vf.discrete_planner()
        print 'agent made'

    def compute_velocity(self, pose):
        p0 = [pose.position.linear.x, pose.position.linear.y]
        rospy.loginfo('current position %s' %self.current_position)
        rospy.loginfo('current state: %s' %self.current_state)
        vel = self.move(current_pos=[pose.position.linear.x, pose.position.linear.y], current_state=self.current_state, current_patch=self.current_patch)
        if self.is_finished:
            print 'finished'
            vel = [0., 0.]

        b = 0.5
        theta = pose.position.angular.z
        rospy.loginfo('velx vely %s' % vel)
        self.cmd.linear.x = vel[0] * np.cos(theta) + vel[1] * np.sin(theta)
        self.cmd.angular.z = 1 / b * (vel[1] * np.cos(theta) - vel[0] * np.sin(theta))
        SendPoseResponse(self.cmd)

def move_base_server():
    rospy.init_node('movebase_ServiceServer')
    rospy.loginfo('%s'%SendPoseRequest)
    # s = rospy.Service('move_base_service', SendPose, VectorField())
    rospy.Service('move_base_service', SendPose, VectorField.compute_velocity)
    print "Server is ready...."
    rospy.spin()

if __name__ == "__main__":
    move_base_server()