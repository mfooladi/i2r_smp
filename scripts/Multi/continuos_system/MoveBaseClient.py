#! /usr/bin/env python

import roslib
# roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
import random
from geometry_msgs.msg import Twist
from i2r_smp.msg import move_baseAction, move_baseGoal

def read_pose(pos):
    global x, y, z
    x = pos.linear.x
    y= pos.linear.y
    z = pos.angular.z


def client_cb():
    global x, y, z
    while True:
        client = actionlib.SimpleActionClient('move_base_server', move_baseAction)
        client.wait_for_server()
        try:
            goal = move_baseGoal(x_current = x, y_current = y, theta_current = z)
            client.send_goal(goal)
            # client.wait_for_result(rospy.Duration.from_sec(2.0))
            rospy.sleep(1)
            rospy.loginfo(client.get_result())
        except:
            print('No position data available!')


if __name__ == '__main__':
    rospy.init_node('move_base_client')
    rospy.Subscriber("khiii/khiii_pose", Twist, read_pose)
    client_cb()
