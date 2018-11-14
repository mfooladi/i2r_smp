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
published = True

global pub_ugv_cmd, ugv, initialized
initialized = True

def read_pose(pos):
    global ugv_pose, initialized, initial_adjustment, ugv, pub_mo, published, p0, init_pos
    init_pos.linear.x = pos.linear.x
    init_pos.linear.y = pos.linear.y
    init_pos.angular.z = pos.angular.z
    p0 = [init_pos.linear.x, init_pos.linear.y]
    rate = rospy.Rate(50
                      )
    if initialized:
        ugv = vf.Robot(geometry_name=vf.r1_geometry, agent_radius=100, agent_id=1, v_max=.5, goals_file=vf.r1_goals, init_pos=p0)
        print ugv.geometry_name
        vf.discrete_planner(geometry_name=ugv.geometry_name, init_position=p0, goals_file=ugv.goals_file)
        initialized = False
    if published:
        update()
        rate.sleep()
    # else:
    # ugv_pose.linear.x = pos.linear.x
    # ugv_pose.linear.y = pos.linear.y
    # ugv_pose.angular.z = pos.angular.z
    # pub_ugv_pose.publish(ugv_pose)
    # ugv.current_position = [ugv_pose.linear.x, ugv_pose.linear.y]
    # pub_monitoring_info.publish(str(pos)+','+str(ugv.current_position)+','+str(ugv.current_state))
    # vel = ugv.move(current_pos=[ugv_pose.linear.x, ugv_pose.linear.y], current_state=ugv.current_state, current_patch=ugv.current_patch)
    # if ugv.is_finished:
    #     print 'finished'
    #     vel = [0., 0.]

    # update(vel, pos.angular.z)
    # rec_time = rospy.get_time() - start_time
    # recfile.write(str(rec_time) + '\t' + str(pos.linear.x) + ',' + str(ugv_pose.linear.y) + ',' + str(ugv_pose.angular.z) + '\n')


def update():
    global pub_ugv_cmd, published, ugv, p0, init_pos
    published = False
    cmd = Twist()
    b = .25

    vel = ugv.move(current_pos=[p0[0], p0[1]], current_state=ugv.current_state, current_patch=ugv.current_patch)
    rospy.loginfo(init_pos.angular.z)
    rate = rospy.Rate(50)
    if ugv.is_finished:
        print 'finished'
        vel = [0., 0.]

    cmd.linear.x =  vel[0] * np.cos(init_pos.angular.z) + vel[1] * np.sin(init_pos.angular.z)
    cmd.angular.z = 1/b * (vel[1]* np.cos(init_pos.angular.z) - vel[0] * np.sin(init_pos.angular.z))
    pub_ugv_cmd.publish(cmd)
    published = True

def stop():
    global pub_ugv_cmd
    cmd = Twist()
    pub_ugv_cmd.publish(cmd)

def recorder():
    global start_time, pub_ugv_cmd, pub_ugv_pose, pub_monitoring_info, published
    pub_ugv_pose = rospy.Publisher('ugv_pose', Twist, queue_size = 1)
    pub_monitoring_info = rospy.Publisher('monitoring', String, queue_size = 1)
    rospy.init_node('ugv_smp', anonymous = True)
    rate = rospy.Rate(50)
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

