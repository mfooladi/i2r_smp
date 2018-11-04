#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import (Twist, Vector3)
from std_msgs.msg import String
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# Add a new instance of agent
ugv_pose = Twist()
vf_angle = Twist()

def vector_field_test(pos):
    v_max = 1.0
    g1_angle = 0.0
    g2_angle = 0.
    g3_angle = -50.
    g1 = np.array([np.cos(np.radians(g1_angle)), np.sin(np.radians(g1_angle))])
    g2 = np.array([np.cos(np.radians(g2_angle)), np.sin(np.radians(g2_angle))])
    g3 = np.array([np.cos(np.radians(g3_angle)), np.sin(np.radians(g3_angle))])
    Glocal = np.matrix([g1, g2, g3]).transpose()
    v1 = [0., 0.]
    v2 = [1270., 0.]
    v3 = [0., 1778.]
    W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
    inv_W = np.matrix(W) ** -1
    GW = Glocal * inv_W
    velocity_at_point = np.array(GW * [[pos.linear.x], [pos.linear.y], [1]]).transpose()[0] * v_max
    theta = math.atan2(velocity_at_point[1], velocity_at_point[0])
    return velocity_at_point, theta

def plot_vf_one_triangle():
    g1_angle = 89.0
    g2_angle = 30.
    g3_angle = -50.
    g1 = np.array([np.cos(np.radians(g1_angle)), np.sin(np.radians(g1_angle))])
    g2 = np.array([np.cos(np.radians(g2_angle)), np.sin(np.radians(g2_angle))])
    g3 = np.array([np.cos(np.radians(g3_angle)), np.sin(np.radians(g3_angle))])
    Glocal = np.matrix([g1, g2, g3]).transpose()
    v1 = [0., 0.]
    v2 = [1270., 0.]
    v3 = [0., 1778.]
    W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
    inv_W = np.matrix(W) ** -1
    GW = Glocal * inv_W
    for l1 in np.linspace(0, 1, 30):
        for l2 in np.linspace(0, 1. - l1, int(30 * (1 - l1))):
            l3 = 1 - l1 - l2
            [x, y] = l1 * np.array(v1) + l2 * np.array(v2) + l3 * np.array(v3)
            v = np.array(GW * [[x], [y], [1]]).transpose()[0]
            plt.quiver(x, y, v[0], v[1], scale=.02, scale_units='xy', width=0.005)
    # fig = plt.figure()
    # axis = plt.subplot(1, 1, 1)
    # plt.axis('off')
    plt.savefig('demo.png', transparent=True, bbox_inches='tight', pad_inches=0)
    # plt.show()

# plot_vf_one_triangle()
# p

# pos = Twist()
# pos.linear.x = 0.
# pos.linear.y = 0.

# vector_field_test(pos)
# print

global pub_ugv_cmd, pub_ugv_pose

def read_pose(pos):
    global ugv_pose, pub_ugv_cmd, pub_ugv_pose, pose_info
    ugv_pose.linear.x = pos.linear.x
    ugv_pose.linear.y = pos.linear.y
    ugv_pose.angular.z = pos.angular.z
    pub_ugv_pose.publish(ugv_pose)
    vel, theta = vector_field_test(pos)
    update(vel, theta, pos.angular.z)

def update(vel, theta, current_theta):
    global pub_ugv_cmd, pose_info
    b = .25
    cmd = Twist()
    cmd.linear.x =  vel[0] * np.cos(current_theta) + vel[1] * np.sin(current_theta)
    cmd.angular.z = 1/b * (vel[1]* np.cos(current_theta) - vel[0] * np.sin(current_theta))
    pub_ugv_cmd.publish(cmd)
    pose_string = 'theta=%s'%theta+'current theta=%s'%current_theta
    pose_info.publish(pose_string)

def stop():
    global pub_ugv_cmd
    cmd = Twist()
    pub_ugv_cmd.publish(cmd)

def recorder():
    global start_time, pub_ugv_cmd, pub_ugv_pose, pose_info
    pub_ugv_pose = rospy.Publisher('ugv_pose', Twist, queue_size=1)
    pose_info = rospy.Publisher('poseInfo', String, queue_size=1)
    rospy.init_node('UGV_IOSFL', anonymous=True)
    rospy.Rate(1)
    start_time = rospy.get_time()
    rospy.Subscriber("khiii/khiii_pose", Twist, read_pose)
    pub_ugv_cmd = rospy.Publisher('/HKE/cmd_velGA', Twist, queue_size=1)
    rospy.on_shutdown(stop)
    rospy.spin()

if __name__ == '__main__':
    recorder()
