import numpy as np
import math
import time
# from tkinter import *
from Tkinter import *
import triangle_plot
import vector_field
import matplotlib
matplotlib.use("TkAgg")

gemotry_name='dscc_simulation' # Triangles Geometry
grid_triangles = triangle_plot.read_triangles(gemotry_name)
Stages = vector_field.init_field(gemotry_name)# Create the Vector Field
current_stage = 0
# robot parameters
if gemotry_name=='D':
    pos = np.array([0.6, 9.2])  # the position of the agent
else:
    pos = np.array([3.0, 0.85])  # the position of the agent
need_transition = False
v_ref, islast = vector_field.vector_field(pos[0], pos[1], current_stage) # the rotation of the agent
v_act = v_ref *.9
psi, theta, phi = 0, 0, 0
p, q, r = 0., 0., 0.
radius = 0.10  # the radius of the agent
epsilon = 0.1 # observable point distance to the center of the agent 
g = 9.81 # gravitational acc
m = 1. # mass
Ixx = 0.0820
Iyy = 0.0820
Izz = 0.1377
J = [[Ixx, 0 ,0],[0, Iyy, 0],[0, 0, Izz]] # agent moment of intertia
# Drawing parameters
pixelsize = 780
framedelay = 10
drawVels = True
QUIT = False
paused = False
step = False
circle = []
velLine = []
gvLine = []
Grid = []
# starPoints = [10, 14, 11, 11, 14, 10, 11, 9, 10, 6, 9, 9, 6, 10, 9, 11]
dt = .01
ittr = 0
maxIttr = 100000
globalTime = 0
# ======================================================================================================================
# flat output calculation
# ======================================================================================================================
def flatoutput(pos, current_stage):
    print 'current stage=', current_stage
    dx = 0.0001
    dy = 0.0001
    pos2 = pos + [dx, 0]
    pos3 = pos2 + [dx, 0]
    pos4 = pos3 + [dx, 0]
    pos5 = pos + [0, dy]
    pos6 = pos5 + [dx, 0]
    pos7 = pos6 + [dx, 0]
    pos8 = pos5 + [0, dy]
    pos9 = pos8 + [dx, 0]
    pos10 = pos8 + [0, dy]
    v1, islast = vector_field.vector_field(pos[0], pos[1], current_stage)
    v2, islast = vector_field.vector_field(pos2[0], pos2[1],  current_stage)
    v3, islast = vector_field.vector_field(pos3[0], pos3[1], current_stage)
    v4, islast = vector_field.vector_field(pos4[0], pos4[1], current_stage)
    v5, islast = vector_field.vector_field(pos5[0], pos5[1], current_stage)
    v6, islast = vector_field.vector_field(pos6[0], pos6[1], current_stage)
    v7, islast = vector_field.vector_field(pos7[0], pos7[1], current_stage)
    v8, islast = vector_field.vector_field(pos8[0], pos8[1], current_stage)
    v9, islast = vector_field.vector_field(pos9[0], pos9[1], current_stage)
    v10, islast = vector_field.vector_field(pos10[0], pos10[1], current_stage)
    v1dot = [(v2[0] - v1[0]) /dx * v1[0]+ (v5[0] - v1[0]) / dy * v1[1],
             (v2[1] - v1[1]) /dx * v1[0] + (v5[1] - v1[1]) / dy * v1[1]]
    v2dot = [(v3[0] - v2[0]) / dx * v2[0] + (v6[0] - v2[0]) / dy * v2[1],
             (v3[1] - v2[1]) / dx * v2[0] + (v6[1] - v2[1]) / dy * v2[1]]
    v3dot = [(v4[0] - v3[0]) / dx * v3[0] + (v7[0] - v3[0]) / dy * v3[1],
             (v4[1] - v3[1]) / dx * v3[0] + (v7[1] - v3[1]) / dy * v3[1]]
    v5dot = [(v6[0] - v5[0]) / dx * v5[0] + (v8[0] - v5[0]) / dy * v5[1],
             (v6[1] - v5[1]) / dx * v5[0] + (v8[1] - v5[1]) / dy * v5[1]]
    v6dot = [(v7[0] - v6[0]) / dx * v6[0] + (v9[0] - v6[0]) / dy * v6[1],
             (v7[1] - v6[1]) / dx * v6[0] + (v9[1] - v6[1]) / dy * v6[1]]
    v8dot = [(v9[0] - v8[0]) / dx * v8[0] + (v10[0] - v8[0]) / dy * v8[1],
             (v9[1] - v8[1]) / dx * v8[0] + (v10[1] - v8[1]) / dy * v8[1]]

    v1dotdot = [(v2dot[0] - v1dot[0]) / dx * v1[0] + (v5dot[0] - v1dot[0]) / dy * v1[1],
                (v2dot[1] - v1dot[1]) / dx * v1[0] + (v5dot[1] - v1dot[1]) / dy * v1[1]]
    v2dotdot = [(v3dot[0] - v2dot[0]) / dx * v2[0] + (v6dot[0] - v2dot[0]) / dy * v2[1],
                (v3dot[1] - v2dot[1]) / dx * v2[0] + (v6dot[1] - v2dot[1]) / dy * v2[1]]
    v5dotdot = [(v6dot[0] - v5dot[0]) / dx * v5[0] + (v8dot[0] - v5dot[0]) / dy * v5[1],
                (v6dot[1] - v5dot[1]) / dx * v5[0] + (v8dot[1] - v5dot[1]) / dy * v5[1]]

    v1dotdotdot = [(v2dotdot[0] - v1dotdot[0]) / dx * v1[0] + (v5dotdot[0] - v1dotdot[0]) / dy * v1[1],
                   (v2dotdot[1] - v1dotdot[1]) / dx * v1[0] + (v5dotdot[1] - v1dotdot[1]) / dy * v1[1]]

    sigma2dot = v1dot
    sigma3dot = v1dotdot
    sigma4dot = v1dotdotdot
    return islast, v1, sigma2dot, sigma3dot, sigma4dot
# ======================================================================================================================
# Robot Dynamics
# ======================================================================================================================
def robot_dynamics(pos, phi, theta, v_ref, fz, tau_x, tau_y, tau_z, p, q, r, R_euler, v_act):
    global g, dt, m
    v_dot = [0, 0, g] + 1 / m * np.matmul(R_euler, [0, 0, fz])
    tau = [tau_x, tau_y, tau_z]
    wb = [p, q, r]
    Omega = [[0, -r , q], [r, 0 , -p], [-q, p, 0]]
    wb_dot = np.matmul(np.matrix(J)**-1, tau) - np.matmul(np.matrix(J)**-1, np.matmul(Omega, np.matmul(J, wb)))
    wb += wb_dot * dt
    phi += np.array(wb)[0][0] * dt
    theta += np.array(wb)[0][1] * dt
    v_act += v_dot[0:2] * dt
    pos += v_act * dt
    return [pos, v_ref, v_act, phi, theta, wb]
# ======================================================================================================================
# initialize the agents
# ======================================================================================================================
def initWorld(canvas):
    global circle, velLine, gvLine, star
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
                                fill=colors[3])  # color the disc of an agenr based on its group id
    velLine = canvas.create_line(0, 0, 10, 10, fill="white")
    gvLine = canvas.create_line(0, 0, 10, 10, fill="red")
    # star = canvas.create_polygon(starPoints, fill="black")
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
                  world_scale*(pos[0]+ radius*v_ref[0]*2.5 - world_xmin), world_scale*(pos[1] + radius*v_ref[1]*2.5 - world_ymin))
    canvas.coords(gvLine,world_scale*(pos[0] - world_xmin), world_scale*(pos[1] - world_ymin),
                  world_scale*(pos[0]+ radius*np.cos(psi)*1.5 - world_xmin), world_scale*(pos[1] + radius*np.sin(psi)*1.5 - world_ymin))
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
    global g, pos, current_stage, v_ref, v_act, phi, theta, psi, need_transition, p, q, r, u, Ixx, Iyy, Izz
    islast, v_ref, sigma2dot, sigma3dot, sigma4dot = flatoutput(pos, current_stage)
    beta_a = -sigma2dot[0]
    beta_b = g
    beta_ab = math.sqrt(beta_a**2 + beta_b**2)
    beta_c = sigma2dot[1]
    theta = math.atan2(beta_a, beta_b)
    phi = math.atan2(beta_c, beta_ab)
    psi = 0
    R_euler = [[np.cos(theta) * np.cos(psi), np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi),
                np.cos(phi) * np.sin(theta) * np.cos(psi) - np.sin(phi) * np.sin(psi)],
               [np.cos(theta) * np.sin(psi), np.sin(phi) * np.sin(theta) * np.sin(psi) - np.cos(phi) * np.cos(psi),
                np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi)],
               [-np.sin(theta), np.sin(phi) * np.cos(theta), np.cos(phi) * np.cos(theta)]]
    p = sigma3dot[1] * math.sqrt(sigma2dot[0]**2 + g**2) - sigma2dot[1] * sigma3dot[0] * sigma2dot[0] / math.sqrt(sigma2dot[0]**2 + g**2)
    q = - g * sigma3dot[0] / (sigma2dot[0]**2 + g**2)
    r = 0
    vzegond = ((sigma3dot[0]**2 + sigma2dot[0] * sigma4dot[0]) * (sigma2dot[0]**2 + g**2) - (sigma2dot[0] *
                                                                                             sigma3dot[0])) /\
              (sigma2dot[0]**2 + g**2)**1.5
    numerator1 =(sigma4dot[1] * math.sqrt(sigma2dot[0]**2 + g**2) - sigma2dot[1] * vzegond)
    numerator2 = 2. * (sigma3dot[1] * math.sqrt(sigma2dot[0]**2 + g**2) - (sigma2dot[1] * sigma3dot[0] * sigma2dot[0] /\
                       (math.sqrt(sigma2dot[0]**2 + g**2))))\
                 * (sigma3dot[1] * sigma2dot[1]  + sigma3dot[0] * sigma2dot[0])
    p_dot = (numerator1 + numerator2) / (sigma2dot[0]**2 + sigma2dot[1]**2 + g**2)**2
    q_dot = (-sigma4dot[0] * g * (sigma2dot[0]**2 + g**2) + 2. * g * sigma2dot[0] * sigma3dot[0]**2) /\
            (sigma2dot[0]**2 + g**2)**2
    r_dot = 0
    # ======================================================================================================================
    # Control Inputs Calculation based on Flat Outputs
    # ======================================================================================================================
    kv = -.9
    # print 'v act', v_act
    ev = math.sqrt((v_act[0]-v_ref[0])**2+(v_act[1]- v_ref[1])**2)
    # print 'velocity error=', ev
    # print 'sigma2dot[0]', - m * math.sqrt(sigma2dot[0]**2 + sigma2dot[1]**2  + g**2)
    fz = -kv * ev - m * math.sqrt(sigma2dot[0]**2 + sigma2dot[1]**2  + g**2)
    # print 'fz vector', fz
    tau_x = Ixx * p_dot + (Izz - Iyy) * r * q
    tau_y =  Iyy * q_dot + (Ixx - Izz) * p * r
    tau_z = Izz * r_dot + (Iyy- Ixx) * p * q
    # print 'tau_x,tau_y,tau_z', tau_x, tau_y, tau_z
    if islast and current_stage < Stages - 1:
        need_transition = True
        print 'need transition'
    if need_transition:
        need_transition = False
        current_stage += 1
        v_ref, islast = vector_field.vector_field(pos[0], pos[1], current_stage)
    if not need_transition:
        pos, v_ref, v_act, phi, theta, wb = robot_dynamics(pos, phi, theta, v_ref, fz, tau_x, tau_y, tau_z, p, q, r, R_euler, v_act)
# ======================================================================================================================
# read a scenario
# ======================================================================================================================
def readScenario(scalex = 1., scaley = 1.):
    x_min = 0 * scalex - 2.
    y_min = 0 * scaley - 1.
    x_max = 10 * scalex + 2.
    y_max = 10 * scaley + 2.
    return x_min, x_max, y_min, y_max
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
            globalTime += dt
        drawWorld()
        if step == True:
            step = False
            paused = True
        win.after(framedelay,lambda: drawFrame(dt))
# ======================================================================================================================
# Main execution of the code
# ======================================================================================================================
world_xmin, world_xmax, world_ymin, world_ymax = readScenario()
world_width = world_xmax - world_xmin
world_height = world_ymax - world_ymin
world_scale = pixelsize/world_width
win = Tk()
win.bind("<space>",on_key_press)
win.bind("<space>",on_key_press)
win.bind("s",on_key_press)
win.bind("<Escape>",on_key_press)
win.bind("v",on_key_press)
canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#F66733")
win.geometry("+{}+{}".format(2500,  100))
canvas.pack()
initWorld(canvas)
start_time = time.time()
win.title('UniCycle Simulation')
win.after(framedelay, lambda: drawFrame(dt))
mainloop()
