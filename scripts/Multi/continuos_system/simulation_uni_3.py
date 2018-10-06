import numpy as np
import math
import time
# from tkinter import *
from Tkinter import *
import triangle_plot
import vector_field
import Tkinter as tk

# Triangles Geometry
# gemotry_name='C'
# gemotry_name='Proposal'
gemotry_name='robot2'
# gemotry_name='dscc_simulation'
grid_triangles = triangle_plot.read_triangles(gemotry_name)
# Create the Vector Field
[Stages, States_v, G_v] = vector_field.init_field(gemotry_name)
current_stage = 0

# robot parameters
if gemotry_name=='D':
    pos = np.array([0.6, 9.3])  # the position of the agent
    # pos = np.array([2, 7.9])  # the position of the agent
    # current_stage = 1
else:
    pos = np.array([3., 1.])  # the inital position of the agent
    # pos = np.array([9.1, 4.])  # the inital position of the agent
    pos = np.array([1.07, .63])  # the inital position of the agent

# vel = np.zeros(2)
# [v_ref, islast, triangle] = vector_field.vector_field(pos[0], pos[1], current_stage, States_v, G_v) # the rotation of
a = 0
islast = True
print 'aaa', vector_field.vector_field(pos[0], pos[1], current_stage)
[v_ref, islast, a] = vector_field.vector_field(pos[0], pos[1], current_stage)
# vv = vector_field.vector_field(pos[0], pos[1], current_stage) # the rotation of
# v= vv[0]
#  the
# agent
u_pre = v_ref
v_ref_pre = v_ref
print 'vref = ', v_ref
theta = math.atan2(v_ref[1], v_ref[0])
theta_ref_pre = theta
print '/theta = ', theta
need_transition = False
radius = 0.05  # the radius of the agent
epsilon = 0.1
E1 = np.array([[0., -1.0], [1.0, 0.0]])
E2 = np.array([[1.0, 0.0], [0, epsilon]])
E2inv = np.linalg.inv(E2)

# Drawing parameters
pixelsize = 780
framedelay = 10
drawVels = True
QUIT = False
paused = False
step = False
circle = []
trajectory = []
quad =[]
velLine = []
gvLine = []
Grid = []
# photo = PhotoImage(file='image.gif')
# background_image = canvas. create_image(300,300,image=photo)
# Initalize parameters to run a simulation
# the simulation time step
dt = .05
ittr = 0
maxIttr = 20000
globalTime = 0
[x_ctr,y_ctr] = triangle_plot.plot_triangles(show_numbers=True)
x_feasible, y_feasible, v_feasible = vector_field.plot_vector_field(gemotry_name)
# print 'xfeasible, y_feasible',v_feasible
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
    global circle, velLine, gvLine, quad_photo, x_ctr,y_ctr, x_feasible, y_feasible, v_feasible
    print ("")
    print ("Simulation of a robot in Triangular environment.")
    print ("White Arrow is Current Velocity")
    print ("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
    print ("")
    # for i, triangle in enumerate(grid_triangles):
    #     Grid.append(canvas.create_polygon(triangle[0][0], triangle[0][1], triangle[1][0], triangle[1][1],
    #                                       triangle[1][0], triangle[1][1], triangle[2][0], triangle[2][1],
    #                                       triangle[2][0], triangle[2][1], triangle[0][0], triangle[0][1], outline="black", fill =""))
    colors = ["white", "blue", "yellow", "#FAA"]
    circle = canvas.create_oval(0, 0, radius, radius,
                                fill=colors[1])  # color the disc of an agenr based on its group id
    trajectory = canvas.create_oval(0, 0, radius/3, radius/3, fill='black')
    velLine = canvas.create_line(0, 0, 10, 10, fill="white")
    gvLine = canvas.create_line(0, 0, 10, 10, fill="yellow")
    for i, xctr in enumerate(x_ctr):
        if i < len(x_ctr)-2:
            canvas.create_line(world_scale*(xctr-world_xmin), world_scale*(y_ctr[i]-world_ymin)
                               , world_scale*(x_ctr[i+1]-world_xmin), world_scale*(y_ctr[i+1]-world_ymin),fill='white',arrow=tk.LAST)
    for i, x in enumerate(x_feasible):
        if i< len(x_feasible)-2:
            canvas.create_line(world_scale*(x-world_xmin),world_scale*(y_feasible[i]-world_ymin),world_scale*
                               (x+v_feasible[i][0]/5-world_xmin),world_scale*
                               (y_feasible[i]+v_feasible[i][1]/5-world_ymin),fill='blue',
                               arrow=tk.LAST, arrowshape=(3,5,.5))


    # quad = PhotoImage(file='quad.gif')
    # quad = quad.subsample(20, 20)
    # quad_photo = canvas.create_image(100, 100, image = quad)
    # photo = PhotoImage(file='image.gif')
    #
    # background_image = canvas.create_image(400,400,image=photo)
    # # canvas.itemconfigure(background_image, state="normal")


# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():
    # photo = PhotoImage(file='image.gif')
    # canvas.create_image(400, 400, image = photo, state = "normal")
    # canvas.itemconfigure(background_image, state="normal")
    # canvas.coords(quad_photo, world_scale * (pos[0] - world_xmin), world_scale * (pos[1] - world_ymin))
    # for i, triangle in enumerate(grid_triangles):
    #     canvas.coords(Grid[i], world_scale*(triangle[0][0]-world_xmin), world_scale*(triangle[0][1]-world_ymin),
    #                   world_scale*(triangle[1][0]-world_xmin), world_scale*(triangle[1][1]-world_ymin),
    #                   world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
    #                   world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
    #                   world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
    #                   world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
        canvas.create_oval(world_scale*(pos[0]-world_xmin), world_scale*(pos[1]-world_ymin),
                           world_scale*(pos[0]+radius/3-world_xmin), world_scale*(pos[1]+radius/3-world_ymin), fill='black')
        canvas.coords(circle,world_scale*(pos[0]- radius - world_xmin), world_scale*(pos[1] - radius - world_ymin),
                  world_scale*(pos[0] + radius - world_xmin), world_scale*(pos[1] + radius - world_ymin))
        canvas.coords(velLine,world_scale*(pos[0] - world_xmin), world_scale*(pos[1] - world_ymin),
                  world_scale*(pos[0]+ radius*v_ref[0]*2.5 - world_xmin), world_scale*(pos[1] + radius*v_ref[1]*2.5 - world_ymin))
        canvas.coords(gvLine,world_scale*(pos[0] - world_xmin), world_scale*(pos[1] - world_ymin),
                  world_scale*(pos[0]+ radius*np.cos(theta)*1.5 - world_xmin), world_scale*(pos[1] + radius*np.sin(theta)*1.5 - world_ymin))
        # quad = PhotoImage(file='quad.gif')
        # quad = quad.subsample(20, 20)
        # print quad_photo
        # quad_photo = canvas.create_image(100, 100, image=quad)
        canvas.coords(quad_photo, world_scale*(pos[0]-world_xmin), world_scale*(pos[1]-world_ymin))

    # if drawVels:
    #     canvas.itemconfigure(velLine, state="normal")
    #     canvas.itemconfigure(gvLine, state="normal")
    # else:
    #     canvas.itemconfigure(velLine, state="hidden")
    #     canvas.itemconfigure(gvLine, state="hidden")
    # canvas.coords(background_image,300,300)
    # photo = PhotoImage(file="image.gif")
    # photo.zoom(1,1)
    # canvas.create_image(350, 350, image=photo)
    # canvas.itemconfig(background_image, state="normal")
    # canvas.coords(background_image, 500, 500)

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
    global pos, current_stage, v_ref, theta,  v_ref_pre, theta_ref_pre, u_pre, need_transition
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    x = np.matmul(R, np.array([[epsilon], [0]])).transpose()[0] + pos
    print 'aa', vector_field.vector_field(x[0], x[1], current_stage)
    [v_ref, islast, a] = vector_field.vector_field(x[0], x[1], current_stage)
    u = np.zeros(2)
    if not np.any(v_ref):
        v_ref = -v_ref_pre
        theta_ref = -theta_ref_pre
    else:
        theta_ref = math.atan2(v_ref[1], v_ref[0])
    if islast and current_stage < Stages - 1:
        need_transition = True
        print 'need transition need transition need transition need transition need transition'
        u[0]=-u_pre[0]
    if need_transition:
        theta_err = theta_ref - theta
        if theta_err < np.pi/8 :
        # if np.linalg.norm(v_ref) < 0.2 :
            need_transition = False
            current_stage += 1
            [v_ref, _] = vector_field.vector_field(x[0], x[1], current_stage)
        else:
            theta_err = theta_ref - theta
            theta_err = np.fmod(theta_err, 2 * np.pi)
            u[1] = 0.5 * theta_err
            # u[1] = min([u[1], 1])
            # u[1] = max([u[1], -1])
    if not need_transition:
        u_pre = [0., 0.]
        v_out = np.matmul(R * E2, u_pre)
        v_err = 2.*(v_ref - v_out)
        E2invR = np.matmul(E2inv, R.transpose())
        u = np.matmul(E2invR, v_err)
        # u = np.matmul(E2inv * R.transpose(), v_err)
        # u[1] = (-np.sin(theta)*v_err[0]+np.cos(theta)*v_err[1])/epsilon
        # u[0] = min([u[0], 1])
        # u[0] = max([u[0], -1])
        # u[1] = min([u[1], 1])
        # u[1] = max([u[1], -1])
        # u[1] = (-np.sin(theta)*v_err[0]+np.cos(theta)*v_err[1])/epsilon
    # print u, need_transition
    u += u_pre
    # print 'u_pre', u_pre
    theta += u[1] * dt  # update the orintation
    # theta += test * dt  # update the orintation
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    posdot = np.matmul(R, np.array([u[0], 0]))
    pos += posdot * dt  # update the position
    theta = np.fmod(theta, 2*np.pi)
    v_ref_pre = v_ref
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
canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width)
win.geometry("+{}+{}".format(2500,  100))
canvas.pack()
# photo= PhotoImage(file='battlefield-F.gif')
# photo = photo.zoom(31,24)
# photo = photo.subsample(33,28)
# background_image = canvas.create_image(390,330,image=photo)
#
# discrete_image= PhotoImage(file='battlefield-F.gif')
# discrete_image = photo.zoom(31,24)
# discrete_image = photo.subsample(33,28)
# canvas.create_image(390,330,image=discrete_image)

quad = PhotoImage(file = 'quad.gif')
quad = quad.subsample(35,35)
quad_photo = canvas.create_image(200,100, image = quad)
# vf_image = PhotoImage(file='')

initWorld(canvas)
start_time = time.time()
# the main loop of the program
win.after(framedelay, lambda: drawFrame(dt))
mainloop()