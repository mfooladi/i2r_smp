#!/usr/bin/env python
import rospy
import geometry_msgs.msg

obj_pos_1 = geometry_msgs.msg.Vector3()
obj_pos_2 = geometry_msgs.msg.Vector3()
human_pos = geometry_msgs.msg.Vector3()
human2_pos = geometry_msgs.msg.Vector3()
pos_1_update = False
pos_2_update = False
human_update = False
human2_update = False
recfile = open('chest_???.txt', 'w')
recfile.write('t,pos1,,,pos2,,,human_pos,,human2_pos,,')
recording = True
def read_pos_1(pos):
    global obj_pos_1, pos_1_update
    if pos.x == 0 and pos.y == 0 and pos.z == 0:
        pos_1_update = False
        return
    obj_pos_1.x = pos.x
    obj_pos_1.y = pos.y
    obj_pos_1.z = pos.z
    pos_1_update = True
    update()

def read_pos_2(pos):
    global obj_pos_2, pos_2_update
    if pos.x == 0 and pos.y == 0 and pos.z == 0:
        pos_2_update = False
        return
    obj_pos_2.x = pos.x
    obj_pos_2.y = pos.y
    obj_pos_2.z = pos.z
    pos_2_update = True
    update()

def read_human_pos(pos):
    global human_pos, human_update
    if pos.x == 0 and pos.y == 0 and pos.z == 0:

        return
    human_pos.x = pos.x
    human_pos.y = pos.y
    human_pos.z = pos.z
    human_update = True
    update()

def read_human2_pos(pos):
    global human2_pos, human2_update
    if pos.x == 0 and pos.y == 0 and pos.z == 0:
        return
    human2_pos.x = pos.x
    human2_pos.y = pos.y
    human2_pos.z = pos.z
    human2_update = True
    update()

def update():
    global recfile, start_time, obj_pos_1, obj_pos_2, human_pos, human2_pos
    if pos_1_update and pos_2_update and human_update and human2_update:
    # if True:
        rec_time = rospy.get_time()-start_time
        # print([rec_time,', ',obj_pos_1.x,', ',obj_pos_1.y,', ',obj_pos_1.z,', ',obj_pos_2.x,', ',obj_pos_2.y,', ',obj_pos_2.z,', ',human_pos.x,', ',human_pos.y,', ',human_pos.z])
        print(
            str(rec_time) + ',' + str(obj_pos_1.x) + ',' + str(obj_pos_1.y) + ',' + str(obj_pos_1.z) + ',' + str(
                obj_pos_2.x) + ',' + str(obj_pos_2.y) + ',' + str(obj_pos_2.z) + ',' + str(human_pos.x) + ',' + str(
                human_pos.y) + ',' + str(human_pos.z) +  ',' + str(human2_pos.x) + ',' + str(
                human2_pos.y) + ',' + str(human2_pos.z) + '\n')
        if recording:
            recfile.write(str(rec_time)+','+str(obj_pos_1.x)+','+str(obj_pos_1.y)+','+str(obj_pos_1.z)
                          +','+str(obj_pos_2.x)+','+str(obj_pos_2.y)+','+str(obj_pos_2.z)
                          +','+str(human_pos.x)+','+str(human_pos.y)+','+str(human_pos.z)
                          +','+str(human2_pos.x)+','+str(human2_pos.y)+','+str(human2_pos.z)+'\n')

def stop():
    global recfile
    # recfile.close()

def recorder():
    global start_time
    rospy.init_node('tracker', anonymous=True)
    rospy.Rate(1)
    start_time = rospy.get_time()
    rospy.Subscriber("LED24", geometry_msgs.msg.Vector3, read_pos_1)
    rospy.Subscriber("LED25", geometry_msgs.msg.Vector3, read_pos_2)
    rospy.Subscriber("LED6", geometry_msgs.msg.Vector3, read_human_pos)
    rospy.Subscriber("LED8", geometry_msgs.msg.Vector3, read_human2_pos)
    rospy.on_shutdown(stop)
    rospy.spin()

if __name__ == '__main__':
    recorder()
