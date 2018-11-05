#!/usr/bin/env python
import rospy
import numpy as np
import math
import vector_fild_new as vf
from geometry_msgs.msg import (Twist, Vector3)
from std_msgs.msg import String

recfile = open('ugv_pos.txt', 'w')
recfile.write('t,\t,pos')
recording = True
ugv_pose = Twist()
vf_angle = Twist()
init_pos = Twist()
monitoring_info = String()

global pub_ugv_cmd, ugv, initialized
initialized = True

def read_pose(pos):
    global ugv_pose, initialized, initial_adjustment, ugv, pub_mo
    if initialized:
        init_pos.linear.x = pos.linear.x
        init_pos.linear.y = pos.linear.y
        init_pos.angular.z = pos.angular.z
        p0 = [init_pos.linear.x, init_pos.linear.y]
        ugv = vf.Robot(geometry_name=vf.r1_geometry, agent_radius=100, agent_id=1, v_max=.3, goals_file=vf.r1_goals, init_pos=p0)
        print ugv.geometry_name
        vf.discrete_planner(geometry_name=ugv.geometry_name, init_position=p0, goals_file=ugv.goals_file)
        # ugv.plot_patch_vf(single_patch=False)
        initialized = False
        print 'task info:', 'states to go:', ugv.patch_length
    else:
        ugv_pose.linear.x = pos.linear.x
        ugv_pose.linear.y = pos.linear.y
        ugv_pose.angular.z = pos.angular.z
        pub_ugv_pose.publish(ugv_pose)
        ugv.current_position = [ugv_pose.linear.x, ugv_pose.linear.y]
        pub_monitoring_info.publish(str(pos)+','+str(ugv.current_position)+','+str(ugv.current_state))
        vel = ugv.move(current_pos=[ugv_pose.linear.x, ugv_pose.linear.y], current_state=ugv.current_state, current_patch=ugv.current_patch)
        if ugv.is_finished:
            print 'finished'
            vel = [0., 0.]

        update(vel, pos.angular.z)
        # rec_time = rospy.get_time() - start_time
        # recfile.write(str(rec_time) + '\t' + str(pos.linear.x) + ',' + str(ugv_pose.linear.y) + ',' + str(ugv_pose.angular.z) + '\n')


def update(vel, current_theta):
    global pub_ugv_cmd
    b = .25
    cmd = Twist()
    cmd.linear.x =  vel[0] * np.cos(current_theta) + vel[1] * np.sin(current_theta)
    cmd.angular.z = 1/b * (vel[1]* np.cos(current_theta) - vel[0] * np.sin(current_theta))
    pub_ugv_cmd.publish(cmd)

def stop():
    global pub_ugv_cmd
    cmd = Twist()
    pub_ugv_cmd.publish(cmd)

def recorder():
    global start_time, pub_ugv_cmd, pub_ugv_pose, pub_monitoring_info
    pub_ugv_pose = rospy.Publisher('ugv_pose', Twist, queue_size = 1)
    pub_monitoring_info = rospy.Publisher('monitoring', String, queue_size = 10)
    rospy.init_node('ugv_smp', anonymous = True)
    rate = rospy.Rate(1)
    start_time = rospy.get_time()
    rospy.Subscriber("khiii/khiii_pose", Twist, read_pose)
    pub_ugv_cmd = rospy.Publisher('/HKE/cmd_velGA', Twist, queue_size = 1)
    rospy.on_shutdown(stop)
    rospy.spin()

if __name__ == '__main__':
    try:
        recorder()
    except rospy.ROSInterruptException:
        pass

