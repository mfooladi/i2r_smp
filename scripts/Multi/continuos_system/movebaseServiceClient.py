#!/usr/bin/env python

import sys
import rospy
# from beginner_tutorials.srv import *
from i2r_smp.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

current_pose = Twist()

def read_pose(pos):
    global current_pose
    current_pose = pos

def run_cb():
    rate = rospy.Rate(2)
    if response.is_executed:
        client_cb()
    else:
        rate.sleep()
        run_cb()

def client_cb():
    global current_pose, response
    rate = rospy.Rate(1)
    rospy.wait_for_service('move_base_service')
    
    try:
        clientServer = rospy.ServiceProxy('move_base_service', SendPose, persistent=True)
        response = clientServer(current_pose)
        pub_ugv_cmd.publish(response.velocity)
        rospy.loginfo(response.velocity.linear.x)
        rate.sleep()
        client_cb()
        return response.velocity
    
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e
    

# def add_two_ints_client(x, y):
#     rospy.wait_for_service('add_two_ints')
#     try:
#         add_two_ints = rospy.ServiceProxy('add_two_ints', SendPose)
#         resp1 = add_two_ints(x, y)
#         return resp1.Sum
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e
#
# def usage():
#     return "%s [x y]"%sys.argv[0]

def stop():
    global pub_ugv_cmd
    cmd = Twist()
    pub_ugv_cmd.publish(cmd)

if __name__ == "__main__":
    rospy.init_node('movebase_ServiceClient')
    pub_ugv_cmd = rospy.Publisher('/HKB/cmd_velGA', Twist, queue_size=1)
    rospy.Subscriber("khiii/khiii_pose", Twist, read_pose)
    rospy.on_shutdown(stop)

    client_cb()
    