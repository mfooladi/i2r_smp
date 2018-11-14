#!/usr/bin/env python

import rospy
import sys
import os
import numpy as np

# dir = os.path.dirname(__file__)
# sys.path.insert(0, dir+'/Multi/continuos_system')
from i2r_smp.srv import *
# print sys.path
# import home.i2rlab.catkin_ws.src.i2r_smp.scripts.Multi.continuous.vector_fild_new as vf
import vector_fild_new as vf
from geometry_msgs.msg import Twist
# import i2r_smp/scripts/Multi/contiuous/vector_fild_new as vf

initialized = False

def execute_request(reqTwist):
    global initialized, agent
    # rospy.loginfo(reqTwist.position)
    req = reqTwist.position
    cmd = Twist()
    p0 = [reqTwist.position.linear.x, reqTwist.position.linear.y]
    if not initialized:
        initialized = True
        agent = vf.Robot(geometry_name=vf.r1_geometry, agent_radius=100, agent_id=1, v_max= .5 , goals_file=vf.r1_goals, init_pos=p0)
        vf.discrete_planner(geometry_name=agent.geometry_name, init_position=p0, goals_file=agent.goals_file)
        print 'agent made'
        rospy.loginfo(agent.current_position)
    elif initialized:
        vel = agent.move(current_pos=[req.linear.x, req.linear.y], current_state=agent.current_state, current_patch=agent.current_patch)
        rospy.loginfo(agent.current_state)
        if agent.is_finished:
            print 'finished'
            vel = [0., 0.]

        b = 0.5
        theta = req.angular.z
        rospy.loginfo('velx vely %s' %vel)
        cmd.linear.x = vel[0] * np.cos(theta) + vel[1] * np.sin(theta)
        cmd.angular.z = 1 / b * (vel[1] * np.cos(theta) - vel[0] * np.sin(theta))
    # pub_ugv_cmd.publish(cmd)
    return SendPoseResponse(cmd)

def add_two_ints_server():
    rospy.init_node('movebase_ServiceServer')
    s = rospy.Service('move_base_service', SendPose, execute_request)
    print "Ready to add two ints."
    # pub_ugv_cmd = rospy.Publisher('/HKB/cmd_velGA', Twist, queue_size=1)
    rospy.on_shutdown(stop)
    rospy.spin()

def stop():
    global pub_ugv_cmd
    cmd = Twist()
    pub_ugv_cmd.publish(cmd)

if __name__ == "__main__":
    # pub_ugv_cmd = rospy.Publisher('/HKB/cmd_velGA', Twist, queue_size=1)
    add_two_ints_server()
    rospy.on_shutdown(stop)
    # rate = rospy.Rate(50)
    # pub_ugv_cmd = rospy.Publisher('/HKB/cmd_velGA', Twist, queue_size=100)
    # rospy.spin()