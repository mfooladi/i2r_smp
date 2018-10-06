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
global pub_khiii_pose
# khiii_pre = Twist()
# khiii_vel = Twist()

pos_1_update = False
pos_2_update = False
recfile = open('khiii_pos.txt', 'w')
recfile.write('t,,,pos1,,,pos2')
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

def update():
    global recfile, start_time, khiii_pos_1, khiii_pos_2, pub_khiii_pose
    if pos_1_update and pos_2_update:
        vector_12 = Twist()
        vector_12.linear.x = khiii_pos_2.x - khiii_pos_1.x
        vector_12.linear.y = khiii_pos_2.y - khiii_pos_1.y
        khiii_pose.linear.x = (khiii_pos_2.x + khiii_pos_1.x) * .5
        khiii_pose.linear.y = (khiii_pos_2.y + khiii_pos_1.y) * .5
        khiii_pose.angular.z = math.atan2(vector_12.linear.y, vector_12.linear.x)
        pub_khiii_pose.publish(khiii_pose)
        rec_time = rospy.get_time()-start_time
        # if recording:
        #     recfile.write(str(rec_time)+','+str(khiii_pos_1.x)+','+str(khiii_pos_1.y)+','+str(khiii_pos_1.z)
        #                   +','+str(khiii_pos_2.x)+','+str(khiii_pos_2.y)+','+str(khiii_pos_2.z)+'\n')

def stop():
    global recfile
    # recfile.close()

def recorder():
    global start_time, pub_khiii_pose
    rospy.init_node('khiii_tracker', anonymous=True)
    rospy.Rate(1)
    start_time = rospy.get_time()
    rospy.Subscriber("LED10", Vector3, read_pos_1)
    rospy.Subscriber("LED11", Vector3, read_pos_2)
    pub_khiii_pose = rospy.Publisher('khiii/khiii_pose', Twist, queue_size=1)

    rospy.on_shutdown(stop)
    rospy.spin()

if __name__ == '__main__':
    recorder()
