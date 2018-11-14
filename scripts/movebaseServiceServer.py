#!/usr/bin/env python

import rospy
import sys
import os

dir = os.path.dirname(__file__)
sys.path.insert(0, dir+'/Multi/continuos_system')
from i2r_smp.srv import *
print sys.path
# import home.i2rlab.catkin_ws.src.i2r_smp.scripts.Multi.continuous.vector_fild_new as vf
import vector_fild_new as vf

# import i2r_smp/scripts/Multi/contiuous/vector_fild_new as vf


global cmd, agent, initialized

initialized = False

def execute_request(reqTwist):
    req = reqTwist.a
    global initialized, pub_ugv_cmd, agent
    p0 = [req.linear.x, req.linear.y]
    if req.linear.x != 0 or not initialized:
        initialized = True
        agent = vf.Robot(geometry_name=vf.r1_geometry, agent_radius=100, agent_id=1, v_max=.5, goals_file=vf.r1_goals, init_pos=p0)
        vf.discrete_planner(geometry_name=agent.geometry_name, init_position=p0, goals_file=agent.goals_file)
        print 'agent made'
        rospy.loginfo(agent.current_position)

    vel = agent.move(current_pos=[req.linear.x, req.linear.y], current_state=agent.current_state, current_patch=agent.current_patch)

    if agent.is_finished:
        print 'finished'
        vel = [0., 0.]

    b = 0.25
    theta = req.angular.z
    cmd.linear.x = vel[0] * np.cos(theta) + vel[1] * np.sin(theta)
    cmd.angular.z = 1 / b * (vel[1] * np.cos(theta) - vel[0] * np.sin(theta))
    pub_ugv_cmd.publish(cmd)
    
    # rospy.loginfo(req)
    
    # rate = rospy.Rate(2)
    # rate.sleep()
    return SendPoseResponse(True)

def add_two_ints_server():
    rospy.init_node('movebase_ServiceServer')
    s = rospy.Service('move_base_service', SendPose, execute_request)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
    rate = rospy.Rate(50)
    pub_ugv_cmd = rospy.Publisher('/HKB/cmd_velGA', Twist, queue_size=100)
    # rospy.spin()