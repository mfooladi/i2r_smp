#!/usr/bin/env python

import sys
import rospy
# from beginner_tutorials.srv import *
from i2r_smp.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist

current_pose = Twist()

def read_pose(pos):
    global current_pose
    current_pose = pos
def run_cb():
    rate = rospy.Rate(10)    
    if response.is_executed:
        client_cb()
    else:
        rate.sleep()
        run_cb()

def client_cb():
    global current_pose, response
    
    rospy.wait_for_service('move_base_service')
    
    try:
        sent_pose = rospy.ServiceProxy('move_base_service', SendPose)
        response = sent_pose(current_pose)
        rospy.loginfo(response)
        run_cb()
        return response.is_executed
    
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

if __name__ == "__main__":
    rospy.init_node('movebase_ServiceClient')
    rospy.Subscriber("khiii/khiii_pose", Twist, read_pose)
    client_cb()
    