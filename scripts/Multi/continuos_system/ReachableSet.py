import numpy as np
import math
from interval import interval
import vector_field
from numpy.linalg import inv
import time
from Tkinter import *
import triangle_plot

# triangle vertices position
v1 = np.array([0.,0.])
v2 = np.array([500.,400.])
v3 = np.array([100.,700.])
v4 = np.array([1000, 700])
# calculating outer normal vectors to each facet
norms = vector_field.find_norms(v1,v2,v3)
print norms[0], norms[1], norms[2]

# normalizing the normal vectors
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v/norm
norms1 = normalize(norms[0])
norms2 = normalize(norms[1])
norms3 = normalize(norms[2])
print norms1, norms2, norms3

# calculating the angle of each normal vector
angle1 = math.atan2(norms[0][1],norms[0][0])/np.pi*180
angle2 = math.atan2(norms[1][1],norms[1][0])/np.pi*180
angle3 = math.atan2(norms[2][1],norms[2][0])/np.pi*180
print angle1, angle2, angle3
range1 = interval([angle2-90, angle3+90])
print 'range1', range1
range2 = interval([angle1+90, angle3+90])
print 'range2', range2
range3 = interval([angle1-90, angle2-90])
print 'range3', range3
print "interval 1 opening", range1[0][0]
# the max g value for each vertice
gMax1 = .01
gMax2 = .01
gMax3 = 1000
g1 = np.array([math.cos(range1[0][0]*np.pi/180)*gMax1, math.sin(range1[0][0]*np.pi/180)*gMax1])
print 'g1', g1
g2 = np.array([math.cos(range2[0][0]*np.pi/180)*gMax2, math.sin(range2[0][0]*np.pi/180)*gMax2])
print 'g2', g2
g3 = np.array([math.cos(range3[0][0]*np.pi/180)*gMax3, math.sin(range3[0][0]*np.pi/180)*gMax3])
print 'g3', g3
# G-matrix
G = np.array([[g1[0],g2[0],g3[0]],[g1[1],g2[1],g3[1]]])
print G
# W-matrix
W = np.array([[v1[0],v2[0],v3[0]],[v1[1],v2[1],v3[1]],[1,1,1]])
# G*inv(W) = GW
GW = np.matmul(G,inv(W))
print 'W', W, 'G*W', GW
def vectorField(x,y):
    f = np.matmul(GW, [x,y,1])
    return f
print 'resultvector', vectorField(3,3)

# Triangles Geometry
gemotry_name = 'D'
# grid_triangles = triangle_plot.read_triangles(gemotry_name)
grid_triangles = [v1, v2, v3]
# print grid_triangles

# Create the Vector Field
# Stages = vector_field.init_field(gemotry_name)
# current_stage = 0

# robot parameters
# if gemotry_name == 'D':
#     pos = np.array([0.6, 9.3])  # the position of the agent
# else:
#     pos = np.array([2.0, 0.5])  # the position of the agent
# initial postion of robot 1
pos = np.array([2.,4.])
# vel = np.zeros(2)
vel = vectorField(pos[0], pos[1])  # the rotation of the agent
print 'vel initial = ',  vel
u_pre = vel
vel_pre = vel
theta = math.atan2(vel[1], vel[0])
theta_ref_pre = theta
print 'theta = ', theta
# need_transition = False
radius = 0.1  # the radius of the agent
epsilon = 0.1
E1 = np.array([[0., -1], [1, 0]])
E2 = np.array([[1, 0], [0, epsilon]])
E2inv = np.linalg.inv(E2)

# Drawing parameters
pixelsize = 1080
framedelay = 6
drawVels = True
QUIT = False
paused = False
step = False
circle = []
velLine = []
gvLine = []
vectorLine = []
Grid = []








# Initalize parameters to run a simulation
# the simulation time step
dt = .1
ittr = 0
maxIttr = 200000
globalTime = 0

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


# ======================================================================================================================
# initialize the agents
# ======================================================================================================================
triangle_id = []
def initWorld(canvas):
    global circle, velLine, gvLine, vectorLine
    print ("")
    print ("Simulation of a robot in Triangular environment.")
    print ("White Arrow is Current Velocity")
    print ("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
    print ("")

    triangle_id.append(canvas.create_polygon(v1[0], v1[1], v2[0], v2[1],
                                             v1[0], v1[1], v3[0], v3[1],
                                             v2[0], v2[1], v3[0], v3[1], outline='black', fill='gray'))
    print triangle_id
    colors = ["white", "blue", "yellow", "#FAA"]
    circle = canvas.create_oval(0, 0, radius, radius, fill=colors[1])  # color the disc of an agent based on its group id
    velLine = canvas.create_line(0, 0, 10, 10, fill="white")
    gvLine = canvas.create_line(0, 0, 10, 10, fill="yellow")
    # vectorLine = canvas.create_line(20, 30, 50, 50, fill ="blue")
    for i in xrange(0, 100):
        for j in xrange(0, 100):

            p = np.array([i/10, j/10])
            vector0 = v3 - v1
            vector1 = v2 - v1
            vector2 = p - v1
            dot00 = np.dot(vector0, vector0)
            dot01 = np.dot(vector0, vector1)
            dot02 = np.dot(vector0, vector2)
            dot11 = np.dot(vector1, vector1)
            dot12 = np.dot(vector1, vector2)
            invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
            u = (dot11 * dot02 - dot01 * dot12) * invDenom
            v = (dot00 * dot12 - dot01 * dot02) * invDenom
            if (u >= 0) and (v >= 0) and (u + v <= 1):
                # vectorLine = canvas.create_line(i, j, i + vectorField(i,j)[0], j + vectorField(i, j)[1], fill='blue')
                True


# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():
    # for i, triangle in enumerate(grid_triangles):
    #     canvas.coords(Grid[i], world_scale * (triangle[0][0] - world_xmin),
    #                   world_scale * (triangle[0][1] - world_ymin),
    #                   world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
    #                   world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
    #                   world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
    #                   world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
    #                   world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
    canvas.coords(triangle_id, world_scale*(v1[0] - world_xmin), world_scale*(v1[1] - world_ymin),
                  world_scale * (v2[0] - world_xmin), world_scale * (v2[1] - world_ymin),
                  world_scale * (v2[0] - world_xmin), world_scale * (v2[1] - world_ymin),
                  world_scale * (v3[0] - world_xmin), world_scale * (v3[1] - world_ymin),
                  world_scale * (v3[0] - world_xmin), world_scale * (v3[1] - world_ymin),
                  world_scale * (v1[0] - world_xmin), world_scale * (v1[1] - world_ymin))
    canvas.coords(circle, world_scale * (pos[0] - radius - world_xmin),
                  world_scale * (pos[1] - radius - world_ymin),
                  world_scale * (pos[0] + radius - world_xmin), world_scale * (pos[1] + radius - world_ymin))
    canvas.coords(velLine, world_scale * (pos[0] - world_xmin), world_scale * (pos[1] - world_ymin),
                  world_scale * (pos[0] + radius * vel[0] * 2.5 - world_xmin),
                  world_scale * (pos[1] + radius * vel[1] * 2.5 - world_ymin))
    canvas.coords(gvLine, world_scale * (pos[0] - world_xmin), world_scale * (pos[1] - world_ymin),
                  world_scale * (pos[0] + radius * np.cos(theta) * 1.5 - world_xmin),
                  world_scale * (pos[1] + radius * np.sin(theta) * 1.5 - world_ymin))
    if drawVels:
        canvas.itemconfigure(velLine, state="normal")
        canvas.itemconfigure(gvLine, state="normal")
    else:
        canvas.itemconfigure(velLine, state="hidden")
        canvas.itemconfigure(gvLine, state="hidden")

# ======================================================================================================================
# keyboard events
# ======================================================================================================================
def on_key_press(event):
    global paused, step, QUIT, drawVels
    if event.keysym == "space":
        paused = not paused
    if event.keysym == "s":
        step = True
        paused = False
    if event.keysym == "v":
        drawVels = not drawVels
    if event.keysym == "Escape":
        QUIT = True

# ======================================================================================================================
# update the simulation
# ======================================================================================================================
def updateSim(dt):
    global pos, vel, theta, vel_pre, theta_ref_pre, u_pre
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    x = np.matmul(R, np.array([[epsilon], [0]])).transpose()[0] + pos
    # print 'x=', x
    # find velocity
    vel = vectorField(x[0], x[1])
    # print 'vel', vel
    if np.any(vel):
        theta_ref = math.atan2(vel[1], vel[0])
    else:
        vel = .5 * vel_pre
        theta_ref = theta_ref_pre

    if abs(theta_ref_pre - theta_ref) > np.pi:
        theta_ref += 2. * np.pi
    # u = np.matmul(E2inv * R.transpose(), vel)
    theta_err = theta_ref - theta
    theta_err = np.fmod(theta_err, 2. * np.pi)
    # print theta_ref, theta, theta_r
    u = np.zeros(2)
    u[1] = 1.5 * theta_err
    u[1] = min([u[1], 1.])
    u[1] = max([u[1], -1.])
    if abs(theta_err) > np.pi / 6.:
        u[0] = 0.
    else:
        u[0] = .45 * np.linalg.norm(vel) / np.cos(theta_err)
        u[0] = min([u[0], 1.])
        u[0] = max([u[0], 0.])
    # print 'u', u
    theta += u[1] * dt  # update the orintation
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    posdot = np.matmul(R, np.array([u[0], 0.]))
    # print 'posdot', posdot
    pos += posdot * dt  # update the position
    theta = np.fmod(theta, 2 * np.pi)
    vel_pre = vel
    theta_ref_pre = theta_ref

    # R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    # x = np.matmul(R, np.array([[epsilon], [0]])).transpose()[0] + pos
    # # find velocity
    # vel, islast = vector_field.vector_field(x[0], x[1], current_stage)
    # theta_ref = math.atan2(vel[1], vel[0])
    # if islast and current_stage < Stages - 1:
    #     if abs(theta - theta_ref)<np.pi/8:
    #         current_stage += 1
    #         print 'stage: ', current_stage
    # v = np.matmul(R * E2, u_pre)
    # u = np.matmul(E2inv * R.transpose(), 0.05*(vel-v))
    # posdot = np.matmul(R, np.array([u[0], 0]))
    # pos += posdot * dt  # update the position
    # theta += u[1] * dt  # update the orintation
    # u_pre = u

# ======================================================================================================================
# simulate and draw frames
# ======================================================================================================================
def drawFrame(dt):
    global start_time, step, paused, ittr, globalTime
    if ittr > maxIttr or QUIT:  # Simulation Loop
        print("%s itterations ran ... quitting" % ittr)
        win.destroy()
    else:
        elapsed_time = time.time() - start_time
        start_time = time.time()
        if not paused:
            updateSim(dt)
            ittr += 1
            # print(ittr)
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
print "world_scale= ", world_scale

def callback(event):
    print "clicked at", event.x, event.y


# set the visualizer
win = Tk()
# keyboard interaction
win.bind("<space>", on_key_press)
win.bind("<space>", on_key_press)
win.bind("s", on_key_press)
win.bind("<Escape>", on_key_press)
win.bind("v", on_key_press)
win.bind("<Button-1>", callback)
# the drawing canvas
# canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#666")
# canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#000")
canvas = Canvas(win, width=pixelsize, height=pixelsize * world_height / world_width, background="#F66733")
# win.geometry("+{}+{}".format(2500, 100))
canvas.pack()
initWorld(canvas)
start_time = time.time()
# the main loop of the program
win.after(framedelay, lambda: drawFrame(dt))
mainloop()