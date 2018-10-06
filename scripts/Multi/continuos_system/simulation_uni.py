import numpy as np
import math
import time
from Tkinter import *
import triangle_plot
import vector_field

# Triangles Geometry
gemotry_name='D'
grid_triangles = triangle_plot.read_triangles(gemotry_name)
print grid_triangles
# print grid_triangles

# Create the Vector Field
Stages = vector_field.init_field(gemotry_name)
current_stage = 0

# robot parameters
if gemotry_name=='D':
    pos = np.array([0.6, 9.3])  # the position of the agent
else:
    pos = np.array([2.0, 0.5])  # the position of the agent

# vel = np.zeros(2)
vel, islast = vector_field.vector_field(pos[0], pos[1], current_stage) # the rotation of the agent
u_pre = vel
vel_pre = vel
theta = math.atan2(vel[1], vel[0])
theta_ref_pre = theta
print '/theta = ', theta
need_transition = False
radius = 0.1  # the radius of the agent
epsilon = 0.1
E1 = np.array([[0., -1], [1, 0]])
E2 = np.array([[1, 0], [0, epsilon]])
E2inv = np.linalg.inv(E2)

# Drawing parameters
pixelsize = 780
framedelay = 60
drawVels = True
QUIT = False
paused = False
step = False
circle = []
velLine = []
gvLine = []
Grid = []

# Initalize parameters to run a simulation
# the simulation time step
dt = .1
ittr = 0
maxIttr = 2000
globalTime = 0

# ======================================================================================================================
# read a scenario
# ======================================================================================================================
def readScenario(scalex=1., scaley=1.):
    x_min =	0*scalex - 2.
    y_min =	0*scaley -1.
    x_max =	10*scalex + 2.
    y_max =	10*scaley + 2.
    return x_min, x_max, y_min, y_max
# ======================================================================================================================
# initialize the agents 
# ======================================================================================================================
def initWorld(canvas):
    global circle, velLine, gvLine
    print ("")
    print ("Simulation of a robot in Triangular environment.")
    print ("White Arrow is Current Velocity")
    print ("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
    print ("")
    for i, triangle in enumerate(grid_triangles):
        Grid.append(canvas.create_polygon(triangle[0][0], triangle[0][1], triangle[1][0], triangle[1][1],
                                          triangle[1][0], triangle[1][1], triangle[2][0], triangle[2][1],
                                          triangle[2][0], triangle[2][1], triangle[0][0], triangle[0][1], outline="black", fill ="gray"))
    colors = ["white", "blue", "yellow", "#FAA"]
    circle = canvas.create_oval(0, 0, radius, radius,
                                fill=colors[1])  # color the disc of an agenr based on its group id
    velLine = canvas.create_line(0, 0, 10, 10, fill="white")
    gvLine = canvas.create_line(0, 0, 10, 10, fill="yellow")
# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():    
    for i, triangle in enumerate(grid_triangles):
        canvas.coords(Grid[i], world_scale*(triangle[0][0]-world_xmin), world_scale*(triangle[0][1]-world_ymin),
                      world_scale*(triangle[1][0]-world_xmin), world_scale*(triangle[1][1]-world_ymin),
                      world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
                      world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
                      world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
                      world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
    canvas.coords(circle,world_scale*(pos[0]- radius - world_xmin), world_scale*(pos[1] - radius - world_ymin),
                  world_scale*(pos[0] + radius - world_xmin), world_scale*(pos[1] + radius - world_ymin))
    canvas.coords(velLine,world_scale*(pos[0] - world_xmin), world_scale*(pos[1] - world_ymin),
                  world_scale*(pos[0]+ radius*vel[0]*2.5 - world_xmin), world_scale*(pos[1] + radius*vel[1]*2.5 - world_ymin))
    canvas.coords(gvLine,world_scale*(pos[0] - world_xmin), world_scale*(pos[1] - world_ymin),
                  world_scale*(pos[0]+ radius*np.cos(theta)*1.5 - world_xmin), world_scale*(pos[1] + radius*np.sin(theta)*1.5 - world_ymin))
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
    global pos, current_stage, vel, theta, vel_pre, theta_ref_pre, u_pre, need_transition
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    x = np.matmul(R, np.array([[epsilon], [0]])).transpose()[0] + pos
    # find velocity
    vel, islast = vector_field.vector_field(x[0], x[1], current_stage)
    if np.any(vel):
        theta_ref = math.atan2(vel[1], vel[0])
    else:
        vel = .5*vel_pre
        theta_ref = theta_ref_pre
    if islast and current_stage < Stages - 1:
        need_transition = True
    if need_transition and np.linalg.norm(vel) < 0.2 :
        need_transition = False
        current_stage += 1
    if abs(theta_ref_pre - theta_ref) > np.pi:
            theta_ref += 2*np.pi
    # u = np.matmul(E2inv * R.transpose(), vel)
    theta_err = theta_ref - theta
    theta_err = np.fmod(theta_err, 2 * np.pi)
    # print theta_ref, theta, theta_r
    u = np.zeros(2)
    u[1] = 1.5 * theta_err
    u[1] = min([u[1], 1])
    u[1] = max([u[1], -1])
    if abs(theta_err) > np.pi/6:
        u[0] = 0
    else:
        u[0] = .45 * np.linalg.norm(vel)/np.cos(theta_err)
        u[0] = min([u[0], 1])
        u[0] = max([u[0], 0])
    # print 'u', u
    theta +=  u[1] * dt  # update the orintation
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    posdot = np.matmul(R, np.array([u[0], 0]))
    pos += posdot * dt  # update the position
    theta = np.fmod(theta, 2*np.pi)
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
    global start_time,step,paused,ittr,globalTime
    if ittr > maxIttr or QUIT: #Simulation Loop
        print("%s itterations ran ... quitting"%ittr)
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
        win.after(framedelay,lambda: drawFrame(dt))

# ======================================================================================================================
# Main execution of the code
# ======================================================================================================================
world_xmin, world_xmax, world_ymin, world_ymax = readScenario()
world_width = world_xmax - world_xmin
world_height = world_ymax - world_ymin
world_scale = pixelsize/world_width

# set the visualizer
win = Tk()
# keyboard interaction
win.bind("<space>",on_key_press)
win.bind("<space>",on_key_press)
win.bind("s",on_key_press)
win.bind("<Escape>",on_key_press)
win.bind("v",on_key_press)
# the drawing canvas
# canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#666")
# canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#000")
canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#F66733")
win.geometry("+{}+{}".format(2500,  100))
canvas.pack()
initWorld(canvas)
start_time = time.time()
# the main loop of the program
win.after(framedelay, lambda: drawFrame(dt))
mainloop()