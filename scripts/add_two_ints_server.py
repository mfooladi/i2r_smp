#!/usr/bin/env python

import rospy
from i2r_smp.srv import *

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.A, req.B, (req.A + req.B))
    return SendPoseResponse(req.A + req.B)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', SendPose, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()