#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    Twist,
    Vector3
)
import math
import numpy as np

khiii_pos_1 = Vector3()
khiii_pos_2 = Vector3()
khiii_pose = Twist()
velocity_instant = Twist()

global pub_khiii_pose

pos_1_update = False
pos_2_update = False
recfile = open('khiii_pos.txt', 'w')
recfile.write('t,\t,pos1,\t,pos2')
recording = True
def read_pos_1(pos):
    global khiii_pos_1, pos_1_update
    if pos.x == 0 and pos.y == 0 and pos.z == 0:
        pos_1_update = False
        return
    khiii_pos_1.x = pos.x
    khiii_pos_1.y = pos.y
    khiii_pos_1.z = pos.z
    pos_1_update = True
    update()

def read_pos_2(pos):
    global khiii_pos_2, pos_2_update
    if pos.x == 0 and pos.y == 0 and pos.z == 0:
        pos_2_update = False
        return
    khiii_pos_2.x = pos.x
    khiii_pos_2.y = pos.y
    khiii_pos_2.z = pos.z
    pos_2_update = True
    update()
position_pre = 0.0
time_pre = 0.0
def update():
    global recfile, start_time, khiii_pos_1, khiii_pos_2, pub_khiii_pose, position_pre, time_pre, pub_khiii_vel_instant
    if pos_1_update and pos_2_update:

        vector_12 = Twist()
        vector_12.linear.x = khiii_pos_2.x - khiii_pos_1.x
        vector_12.linear.y = khiii_pos_2.y - khiii_pos_1.y
        khiii_pose.linear.x = (khiii_pos_2.x + khiii_pos_1.x) * .5
        khiii_pose.linear.y = (khiii_pos_2.y + khiii_pos_1.y) * .5
        khiii_pose.angular.z = math.atan2(vector_12.linear.y, vector_12.linear.x)
        position_current = np.sqrt(khiii_pose.linear.x ** 2 + khiii_pose.linear.y ** 2)
        time_current = rospy.get_time()
        velocity_instant.linear.x = abs(position_current - position_pre) / (time_current - time_pre)/1000.
        pub_khiii_vel_instant.publish(velocity_instant)
        pub_khiii_pose.publish(khiii_pose)
        rec_time = rospy.get_time()-start_time
        position_pre = position_current
        time_pre = time_current
        if recording:
            recfile.write(str(rec_time)+'\t'+str(khiii_pos_1.x)+','+str(khiii_pos_1.y)+','+str(khiii_pos_1.z)
                          +'\t'+','+str(khiii_pos_2.x)+','+str(khiii_pos_2.y)+','+str(khiii_pos_2.z)+'\n')

def stop():
    global recfile
    # recfile.close()

def recorder():
    global start_time, pub_khiii_pose, pub_khiii_vel_instant
    pub_khiii_pose = rospy.Publisher('khiii/khiii_pose', Twist, queue_size=1)
    pub_khiii_vel_instant = rospy.Publisher('khiii/khiii_vel_instant', Twist, queue_size=1)
    rospy.init_node('khiii_tracker', anonymous=True)
    rospy.Rate(1)
    start_time = rospy.get_time()
    rospy.Subscriber("LED19", Vector3, read_pos_1)
    rospy.Subscriber("LED18", Vector3, read_pos_2)
    rospy.on_shutdown(stop)
    rospy.spin()
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub_khiii_pose.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    recorder()
