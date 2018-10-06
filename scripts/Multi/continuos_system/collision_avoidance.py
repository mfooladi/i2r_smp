# coding=utf-8
# import required libs
import numpy as np
import math
from interval import interval
import vector_field
from numpy.linalg import inv
import time
from Tkinter import *

# Drawing parameters
radius = 10  # the radius of the agent
epsilon = 0.1
pixelsize = 1080
framedelay = 1
drawVels = True
QUIT = False
paused = False
step = False
circle = []
circle2 = []
velLine = []
gvLine = []
vectorLine = []
Grid = []
g1 = []
g2 = []
g_max = 1.
g_matrix = []
w_matrix = []
d = 1000000.
theta1 = 0.
theta_ref_pre1 = 0.
theta2 = 1.
theta_ref_pre2 = 1.

# Initalize parameters to run a simulation
# the simulation time step
dt = .1
ittr = 0
maxIttr = 20
globalTime = 0

triangle_id_1 = []
triangle_id_2 = []
# ======================================================================================================================
# normalizing the normal vectors
# ======================================================================================================================
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v/norm

# ======================================================================================================================
# initialize world - requesting for points on canvas
# ======================================================================================================================
def init_world(canvas):
    global circle, circle2, velLine, gvLine, vectorLine
    print ("")
    print ("Simulation of a robot in Triangular environment.")
    print ("White Arrow is Current Velocity")
    print ("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
    print ("")

    triangle_id_1.append(canvas.create_polygon(vertices[0][0], vertices[0][1], vertices[1][0], vertices[1][1],
                                             vertices[0][0], vertices[0][1], vertices[2][0], vertices[2][1],
                                             vertices[1][0], vertices[1][1], vertices[2][0], vertices[2][1],
                                             outline='black', fill='gray'))
    # print triangle_id
    triangle_id_2.append(canvas.create_polygon(vertices[3][0], vertices[3][1], vertices[1][0], vertices[1][1],
                                             vertices[3][0], vertices[3][1], vertices[2][0], vertices[2][1],
                                             vertices[1][0], vertices[1][1], vertices[2][0], vertices[2][1],
                                             outline='black', fill='gray'))

    colors = ["white", "blue", "yellow", "#FAA"]
    if click_num == 6:
        circle = canvas.create_oval(robot_position[0][0] - radius, robot_position[0][1] - radius,
                                    robot_position[0][0] + radius, robot_position[0][1] + radius,
                                    fill=colors[1])  # color the disc of an agent based on its group id
        circle2 = canvas.create_oval(robot_position[1][0] - radius, robot_position[1][1] - radius,
                                    robot_position[1][0] + radius, robot_position[1][1] + radius,
                                    fill=colors[2])  # color the disc of an agent based on its group id
    velLine = canvas.create_line(0, 0, 10, 10, fill="white")
    gvLine = canvas.create_line(0, 0, 10, 10, fill="yellow")

trinagle_1_angle = []
triangle_1_angle_range = []
trinagle_2_angle = []
triangle_2_angle_range = []

def distance(pos1, pos2):
    return d


def no_collision_vector_field():
    global gw1, gw2
    g1_matrix = np.array([[g1[0][0],g1[1][0],g1[2][0]],[g1[0][1],g1[1][1],g1[2][1]]])
    g2_matrix = np.array([[g2[0][0], g2[1][0], g2[2][0]], [g2[0][1], g2[1][1], g2[2][1]]])
    print 'G1=', g1_matrix
    w1_matrix = np.array([[vertices[0][0],vertices[1][0],vertices[2][0]],[vertices[0][1],vertices[1][1],vertices[2][1]],[1.,1.,1.]])
    w2_matrix = np.array(
        [[vertices[3][0], vertices[1][0], vertices[2][0]], [vertices[3][1], vertices[1][1], vertices[2][1]],
         [1., 1., 1.]])
    print 'W1=', w1_matrix
    gw1 = np.matmul(g1_matrix, inv(w1_matrix))
    gw2 = np.matmul(g2_matrix, inv(w2_matrix))
    print 'gw1', gw1
    # velocity = np.array(gw1*[[x], [y], [1]]).transpose()
    # print 'Velocity', velocity





def triangle_norms():
    triangle_1_norms = vector_field.find_norms(vertices[0], vertices[1], vertices[2])
    triangle_2_norms = vector_field.find_norms(vertices[3], vertices[1], vertices[2])
    triangle_1_norms = [normalize(triangle_1_norms[0]), normalize(triangle_1_norms[1]), normalize(triangle_1_norms[2])]
    triangle_2_norms = [normalize(triangle_2_norms[0]), normalize(triangle_2_norms[1]), normalize(triangle_2_norms[2])]
    print 'triangle_1_norms = ', triangle_1_norms
    print 'triangle_2_norms', triangle_2_norms
    triangle_1_angle = [math.atan2(triangle_1_norms[0][1],triangle_1_norms[0][0]) / np.pi*180,
                        math.atan2(triangle_1_norms[1][1],triangle_1_norms[1][0]) / np.pi*180,
                        math.atan2(triangle_1_norms[2][1],triangle_1_norms[2][0]) / np.pi*180]
    triangle_2_angle = [math.atan2(triangle_2_norms[0][1], triangle_2_norms[0][0]) / np.pi * 180,
                        math.atan2(triangle_2_norms[1][1], triangle_2_norms[1][0]) / np.pi * 180,
                        math.atan2(triangle_2_norms[2][1], triangle_2_norms[2][0]) / np.pi * 180]
    print 'triangle_1_angle', triangle_1_angle
    print 'triangle_2_angle', triangle_2_angle

    triangle_1_angle_range = [interval([triangle_1_angle[1]+90, triangle_1_angle[2]-90]),
                              interval([triangle_1_angle[0]+90, triangle_1_angle[2]+90]),
                              interval([triangle_1_angle[0]-90, triangle_1_angle[1]-90])]
    triangle_2_angle_range = [interval([triangle_2_angle[1]+90, triangle_2_angle[2]-90]),
                              interval([triangle_2_angle[0]+90, triangle_2_angle[2]+90]),
                              interval([triangle_2_angle[0]-90, triangle_2_angle[1]-90])]
    print 'triangle_1_angle_rane', triangle_1_angle_range
    print 'triangle_2_angle_range', triangle_2_angle_range
    print 'triangle_1_angle_range[0]', triangle_1_angle_range[0].midpoint[0][0]
    g1.append(np.array([math.cos(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max,
                        math.sin(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180)]))
    g1.append(np.array([math.cos(triangle_1_angle_range[1].midpoint[0][0] * np.pi / 180) * g_max,
                        math.sin(triangle_1_angle_range[1].midpoint[0][0] * np.pi / 180)]))
    g1.append(np.array([math.cos(triangle_1_angle_range[2].midpoint[0][0] * np.pi / 180) * g_max,
                        math.sin(triangle_1_angle_range[2].midpoint[0][0] * np.pi / 180)]))
    g2.append(np.array([math.cos(triangle_2_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max,
                        math.sin(triangle_2_angle_range[0].midpoint[0][0] * np.pi / 180)]))
    g2.append(np.array([math.cos(triangle_2_angle_range[1].midpoint[0][0] * np.pi / 180) * g_max,
                        math.sin(triangle_2_angle_range[1].midpoint[0][0] * np.pi / 180)]))
    g2.append(np.array([math.cos(triangle_2_angle_range[2].midpoint[0][0] * np.pi / 180) * g_max,
                        math.sin(triangle_2_angle_range[2].midpoint[0][0] * np.pi / 180)]))
    print 'g1', g1
    print 'g2', g2
    print 'g1[0][0]=', g1[0][0]
    no_collision_vector_field()

# ======================================================================================================================
# calculation of vector field
# ======================================================================================================================
# def no_collision_vector_field():
#     global g1
    # g1.append(math.cos(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max)
                        # , math.sin(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max)])
    # g1.append(np.array([math.cos(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max, math.sin(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max]))
    # g1.append(np.array([math.cos(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max, math.sin(triangle_1_angle_range[0].midpoint[0][0] * np.pi / 180) * g_max]))
    # print 'g1', g1
    # g2 =







# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():
    # canvas.coords(triangle_id, world_scale*(v1[0] - world_xmin), world_scale*(v1[1] - world_ymin),
    #               world_scale * (v2[0] - world_xmin), world_scale * (v2[1] - world_ymin),
    #               world_scale * (v2[0] - world_xmin), world_scale * (v2[1] - world_ymin),
    #               world_scale * (v3[0] - world_xmin), world_scale * (v3[1] - world_ymin),
    #               world_scale * (v3[0] - world_xmin), world_scale * (v3[1] - world_ymin),
    #               world_scale * (v1[0] - world_xmin), world_scale * (v1[1] - world_ymin))
    canvas.coords(circle,  (robot_position[0][0] - radius - world_xmin),
                   (robot_position[0][1] - radius - world_ymin),
                   (robot_position[0][0] + radius - world_xmin),
                   (robot_position[0][1] + radius - world_ymin))
    canvas.coords(circle2,  (robot_position[1][0] - radius - world_xmin),
                   (robot_position[1][1] - radius - world_ymin),
                   (robot_position[1][0] + radius - world_xmin),
                   (robot_position[1][1] + radius - world_ymin))
    # canvas.coords(velLine, world_scale * (pos[0] - world_xmin), world_scale * (pos[1] - world_ymin),
    #               world_scale * (pos[0] + radius * vel[0] * 2.5 - world_xmin),
    #               world_scale * (pos[1] + radius * vel[1] * 2.5 - world_ymin))
    # canvas.coords(gvLine, world_scale * (pos[0] - world_xmin), world_scale * (pos[1] - world_ymin),
    #               world_scale * (pos[0] + radius * np.cos(theta) * 1.5 - world_xmin),
                  # world_scale * (pos[1] + radius * np.sin(theta) * 1.5 - world_ymin))
    if drawVels:
        canvas.itemconfigure(velLine, state="normal")
        canvas.itemconfigure(gvLine, state="normal")
    else:
        canvas.itemconfigure(velLine, state="hidden")
        canvas.itemconfigure(gvLine, state="hidden")


# ======================================================================================================================
# read a scenario
# ======================================================================================================================
def readScenario(scalex=1., scaley=1.):
    x_min = 0 * scalex
    y_min = 0 * scaley
    x_max = 10 * scalex
    y_max = 10 * scaley
    print 'xmin', x_min, 'xmax', x_max, 'ymin', y_min, 'ymax', y_max
    return x_min, x_max, y_min, y_max


def velocity_finder(gw ,x,y):
    vel = np.array(np.matmul(gw,[[x], [y], [1]])).transpose()
    return vel
# ======================================================================================================================
# update the simulation
# ======================================================================================================================
first_run = True
def updateSim(dt):
    global pos1, vel1, theta1, vel_pre1, theta_ref_pre1, u_pre1, pos2, vel2, theta2, vel_pre2, theta_ref_pre2, u_pre2
    R1 = np.array([[np.cos(theta1), -np.sin(theta1)], [np.sin(theta1), np.cos(theta1)]])
    R2 = np.array([[np.cos(theta2), -np.sin(theta2)], [np.sin(theta2), np.cos(theta2)]])
    x1 = np.matmul(R1, np.array([[epsilon], [0]])).transpose()[0] + robot_position[0]
    x2 = np.matmul(R2, np.array([[epsilon], [0]])).transpose()[0] + robot_position[1]
    print 'x=', x1
    # find velocity
    vel1 = velocity_finder(gw1, x1[0], x1[1])
    vel2 = velocity_finder(gw2, x2[0], x2[1])

    print 'vel', vel1
    if np.any(vel1):
        theta_ref1 = math.atan2(vel1[0][1], vel1[0][0])
    else:
        vel1 = .5 * vel_pre1
        theta_ref1 = theta_ref_pre1

    if np.any(vel2):
        theta_ref2 = math.atan2(vel2[0][1], vel2[0][0])
    else:
        vel2 = .5 * vel_pre2
        theta_ref2 = theta_ref_pre2

    if abs(theta_ref_pre1 - theta_ref1) > np.pi:
        theta_ref1 += 2. * np.pi
    if abs(theta_ref_pre2 - theta_ref2) > np.pi:
        theta_ref2 += 2. * np.pi
    # u = np.matmul(E2inv * R.transpose(), vel)
    theta_err1 = theta_ref1 - theta1
    theta_err2 = theta_ref2 - theta2
    theta_err1 = np.fmod(theta_err1, 2. * np.pi)
    theta_err2 = np.fmod(theta_err2, 2. * np.pi)
    # print theta_ref, theta, theta_r
    u1 = np.zeros(2)
    u2 = np.zeros(2)
    u1[1] = 1.5 * theta_err1
    u1[1] = min([u1[1], 1.])
    u1[1] = max([u1[1], -1.])
    u2[1] = 1.5 * theta_err2
    u2[1] = min([u2[1], 1.])
    u2[1] = max([u2[1], -1.])
    if abs(theta_err1) > np.pi / 6.:
        u1[0] = 0.
    else:
        u1[0] = .45 * np.linalg.norm(vel1) / np.cos(theta_err1)
        u1[0] = min([u1[0], 1.])
        u1[0] = max([u1[0], 0.])
    if abs(theta_err2) > np.pi / 6.:
        u2[0] = 0.
    else:
        u2[0] = .45 * np.linalg.norm(vel2) / np.cos(theta_err2)
        u2[0] = min([u2[0], 1.])
        u2[0] = max([u2[0], 0.])
    # print 'u', u
    theta1 += u1[1] * dt  # update the orintation
    theta2 += u2[1] * dt  # update the orintation
    R1 = np.array([[np.cos(theta1), -np.sin(theta1)], [np.sin(theta1), np.cos(theta1)]])
    R2 = np.array([[np.cos(theta2), -np.sin(theta2)], [np.sin(theta2), np.cos(theta2)]])
    posdot1 = np.matmul(R1, np.array([u1[0], 0.]))
    posdot2 = np.matmul(R2, np.array([u2[0], 0.]))
    print 'posdot', posdot1
    robot_position[0] += posdot1 * dt  # update the position
    robot_position[1] += posdot2 * dt  # update the position
    theta1 = np.fmod(theta1, 2 * np.pi)
    theta2 = np.fmod(theta2, 2 * np.pi)
    vel_pre1 = vel1
    vel_pre2 = vel2
    theta_ref_pre1 = theta_ref1
    theta_ref_pre2 = theta_ref2
# ======================================================================================================================
# simulate and draw frames
# ======================================================================================================================
def drawFrame(dt):
    global start_time, step, paused, ittr, globalTime
    if ittr > maxIttr or QUIT:  # Simulation Loop
        print("%s itterations ran ... quitting" % ittr)
        # win.destroy()
    else:
        # elapsed_time = time.time() - start_time
        start_time = time.time()
        if not paused:
            updateSim(dt)
            ittr += 1
            print(ittr)
            globalTime += dt

        drawWorld()
        # step = True
        if step == True:
            step = False
            paused = True

        win.title('UniCycle Simulation')
        win.after(framedelay, lambda: drawFrame(dt))


# ======================================================================================================================
# Main execution of the code
# ======================================================================================================================
world_xmin, world_xmax, world_ymin, world_ymax = readScenario()
world_width = world_xmax - world_xmin
world_height = world_ymax - world_ymin
world_scale = pixelsize / world_width
click_num = 0
vertices = dict()
robot_position = dict()

def callback(event):
    global click_num
    print "clicked at", event.x, event.y
    if click_num < 6:
        vertices[click_num] = np.array([float(event.x), float(event.y)])
        if click_num == 3:
                init_world(canvas)
        print vertices
        click_num += 1
    elif click_num == 6:

        robot_position[0] = vertices[4]

        print 'robot pos[0]', robot_position[0]
        robot_position[1] = vertices[5]
        init_world(canvas)
        click_num = 7
    if click_num == 7:
        triangle_norms()
        click_num = 8
        drawFrame(dt)


# ======================================================================================================================
# set the visualizer
# ======================================================================================================================
win = Tk()
canvas = Canvas(win, width=pixelsize, height=pixelsize * world_height / world_width, background="#F66733")
win.bind("<Button-1>", callback)
canvas.pack()
# init_world(canvas)
start_time = time.time()
# the main loop of the program

win.after(framedelay, lambda: drawFrame(dt))
mainloop()




