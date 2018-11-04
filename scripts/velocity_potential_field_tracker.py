#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    Twist,
    Vector3
)
import math
import numpy as np

khiii_pose = Twist()

global pub_khiii_cmd

def read_pose(pos):
    global khiii_pose
    khiii_pose.linear.x = pos.linear.x
    khiii_pose.linear.y = pos.linear.y
    khiii_pose.angular.z = pos.angular.z
    update()

def update():
    global pub_khiii_cmd
    cmd = Twist()
    potential_r = np.sqrt(khiii_pose.linear.x ** 2 + khiii_pose.linear.y ** 2)
    potential_theta = math.atan2(khiii_pose.linear.y, khiii_pose.linear.x)
    cmd.linear.x = 2000./ 2./ np.pi/ (potential_r + 20)
    cmd.angular.z = (potential_theta - khiii_pose.angular.z)
    pub_khiii_cmd.publish(cmd)


def stop():
    global pub_khiii_cmd
    cmd = Twist()
    pub_khiii_cmd.publish(cmd)


def recorder():
    global start_time, pub_khiii_cmd
    rospy.init_node('khiii_vector_filed', anonymous=True)
    rospy.Rate(1)
    start_time = rospy.get_time()
    rospy.Subscriber("khiii/khiii_pose", Twist, read_pose)
    pub_khiii_cmd = rospy.Publisher('/HKC/cmd_velGA', Twist, queue_size=1)

    rospy.on_shutdown(stop)
    rospy.spin()

if __name__ == '__main__':
    recorder()
