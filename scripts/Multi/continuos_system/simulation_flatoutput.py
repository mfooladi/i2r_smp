import numpy as np
import math
import time
# from tkinter import *
from Tkinter import *
import triangle_plot
import vector_field

# Triangles Geometry
# gemotry_name = 'D'
gemotry_name = 'dscc_simulation'
grid_triangles = triangle_plot.read_triangles()
# Create the Vector Field
Stages = vector_field.init_field(gemotry_name)
current_stage = 0
# robot parameters
if gemotry_name == 'D':
    pos = np.array([0.6, 9.3])  # the position of the agent
else:
    pos = np.array([3., 1.])  # the position of the agent
need_transition = False
b = 0.2
# pos = [3., 1.]
v_act = [pos[1] - b * pos[0], -pos[0] - b * pos[1]]
def velocity_vector(x):
    b = 0.2
    v = [x[1] - b * x[0], -x[0] - b * x[1]]
    # print 'position=', x, 'distance', math.sqrt(x[0]**2 + x[1]**2), 'ref vel=', v
    if math.sqrt(v[0]**2 + v[1]**2) > 0.4:
        sigma2dot = [v[1] - b * v[0], -v[0] - b * v[1]]
        sigma3dot = [sigma2dot[1] - b * sigma2dot[0], -sigma2dot[0] - b * sigma2dot[1]]
        sigma4dot = [sigma3dot[1] - b * sigma3dot[0], -sigma3dot[0] - b * sigma3dot[1]]
    else:
        v = 0.
        sigma2dot = 0.
        sigma3dot = 0.
        sigma4dot = 0.
    return  v, sigma2dot, sigma3dot, sigma4dot
psi = 0
theta = 0
phi = 0
# psi = math.atan2(v_ref[1], v_ref[0])
# psi_ref_pre = psi
# theta_ref_pre = theta
# phi_ref_pre = phi
# print '/psi = ', psi
p, q, r = 0., 0., 0.
radius = 0.10  # the radius of the agent
# epsilon = 0.1  # observable point distance to the center of the agent
# E1 = np.array([[0., -1.0], [1.0, 0.0]])
# E2 = np.array([[1.0, 0.0], [0, epsilon]])
# E2inv = np.linalg.inv(E2)
g = 9.81  # gravitational acc
m = 1.  # mass

Ixx = 0.0820
Iyy = 0.0820
Izz = 0.1377
J = [[Ixx, 0., 0.], [0., Iyy, 0.], [0., 0., Izz]]  # agent moment of intertia

# Drawing parameters
pixelsize = 780
framedelay = 2
drawVels = True
QUIT = False
paused = False
step = False
circle = []
velLine = []
gvLine = []
Grid = []
starPoints = [10, 14, 11, 11, 14, 10, 11, 9, 10, 6, 9, 9, 6, 10, 9, 11]

# Initalize parameters to run a simulation
# the simulation time step
dt = .01
ittr = 0
maxIttr = 100000
globalTime = 0


# ======================================================================================================================
# flat output calculation
# ======================================================================================================================
def flatoutput(pos, current_stage):
    global dt
    v1, islast = vector_field.vector_field(pos[0], pos[1], current_stage)
    ######################################################################################################
    # sigma2:4 calculation ### vector field derivations - first method
    ######################################################################################################
    # print 'reference velocity', v1
    # velocity_angle_p1 = math.atan2(v1[1], v1[0])
    # pos1 = (pos + [v1 * math.cos(velocity_angle_p1) * dt, v1 * math.sin(velocity_angle_p1) * dt])[0]
    # print 'position0', pos, 'position1', pos1
    # v2, islast = vector_field.vector_field(pos1[0], pos1[1], current_stage)
    # print 'velocity1', v1, 'velocity2', v2
    # velocity_angle_p2 = math.atan2(v2[1], v2[0])
    # sigma2dot = (v2 - v1) / dt
    # print 'refernce acceleration', sigma2dot
    # pos2 = (pos1 + [v2 * math.cos(velocity_angle_p2) * dt, v2 * math.sin(velocity_angle_p2) * dt])[0]
    # v3, islast = vector_field.vector_field(pos2[0], pos2[1], current_stage)
    # # print 'v3', v3
    # velocity_angle_p3 = math.atan2(v3[1], v3[0])
    # v2dot = (v3 - v2) / dt
    # sigma3dot = (v2dot - sigma2dot) /dt
    # pos3 = (pos2 + [v3 * math.cos(velocity_angle_p3) * dt, v3 * math.sin(velocity_angle_p3) * dt])[0]
    # v4, islast = vector_field.vector_field(pos3[0], pos3[1], current_stage)
    # v3dot = (v4 - v3) / dt
    # v2dotdot = (v3dot - v2dot) / dt
    # sigma4dot = (sigma3dot - v2dotdot) /dt

    ######################################################################################################
    # sigma2:4 calculation ### vector field derivations - second method
    ######################################################################################################
    # dx = 0.001
    # dy = 0.001
    # pos2 = pos + [dx, 0]
    # pos3 = pos2 + [dx, 0]
    # pos4 = pos3 + [dx, 0]
    # pos5 = pos + [0, dy]
    # pos6 = pos5 + [dx, 0]
    # pos7 = pos6 + [dx, 0]
    # pos8 = pos5 + [0, dy]
    # pos9 = pos8 + [dx, 0]
    # pos10 = pos8 + [0, dy]
    # v2, islast = vector_field.vector_field(pos2[0], pos2[1], current_stage)
    # v3, islast = vector_field.vector_field(pos3[0], pos3[1], current_stage)
    # v4, islast = vector_field.vector_field(pos4[0], pos4[1], current_stage)
    # v5, islast = vector_field.vector_field(pos5[0], pos5[1], current_stage)
    # v6, islast = vector_field.vector_field(pos6[0], pos6[1], current_stage)
    # v7, islast = vector_field.vector_field(pos7[0], pos7[1], current_stage)
    # v8, islast = vector_field.vector_field(pos8[0], pos8[1], current_stage)
    # v9, islast = vector_field.vector_field(pos9[0], pos9[1], current_stage)
    # v10, islast = vector_field.vector_field(pos10[0], pos10[1], current_stage)
    # v1dot = [(v2[0] - v1[0]) / dx * v1[0] + (v2[1] - v1[1]) / dy * v1[1],
    #          (v5[0] - v1[0]) / dx * v1[0] + (v5[1] - v1[1]) / dy * v1[1]]
    # print 'v1dot or sigma dobule dot = ', v1dot
    # v2dot = [(v3[0] - v2[0]) / dx * v2[0] + (v3[1] - v2[1]) / dy * v2[1],
    #          (v6[0] - v2[0]) / dx * v2[0] + (v6[1] - v2[1]) / dy * v2[1]]
    # v3dot = [(v4[0] - v3[0]) / dx * v3[0] + (v4[1] - v3[1]) / dy * v3[1],
    #          (v7[0] - v3[0]) / dx * v3[0] + (v7[1] - v3[1]) / dy * v3[1]]
    # v5dot = [(v6[0] - v5[0]) / dx * v5[0] + (v6[1] - v5[1]) / dy * v5[1],
    #          (v8[0] - v5[0]) / dx * v5[0] + (v8[1] - v5[1]) / dy * v5[1]]
    # v6dot = [(v7[0] - v6[0]) / dx * v6[0] + (v7[1] - v6[1]) / dy * v6[1],
    #          (v9[0] - v6[0]) / dx * v6[0] + (v9[1] - v6[1]) / dy * v6[1]]
    # v8dot = [(v9[0] - v8[0]) / dx * v8[0] + (v9[1] - v8[1]) / dy * v8[1],
    #          (v10[0] - v8[0]) / dx * v8[0] + (v10[1] - v8[1]) / dy * v8[1]]
    #
    # v1dotdot = [(v2dot[0] - v1dot[0]) / dx * v1dot[0] + (v2dot[1] - v1dot[1]) / dy * v1dot[1],
    #             (v5dot[0] - v1dot[0]) / dx * v1dot[0] + (v5dot[1] - v1dot[1]) / dy * v1dot[1]]
    # v2dotdot = [(v3dot[0] - v2dot[0]) / dx * v2dot[0] + (v3dot[1] - v2dot[1]) / dy * v2dot[1],
    #             (v6dot[0] - v2dot[0]) / dx * v2dot[0] + (v6dot[1] - v2dot[1]) / dy * v2dot[1]]
    # v5dotdot = [(v6dot[0] - v5dot[0]) / dx * v5dot[0] + (v6dot[1] - v5dot[1]) / dy * v5dot[1],
    #             (v8dot[0] - v5dot[0]) / dx * v5dot[0] + (v8dot[1] - v5dot[1]) / dy * v5dot[1]]
    #
    # v1dotdotdot = [(v2dotdot[0] - v1dotdot[0]) / dx * v1dotdot[0] + (v2dotdot[1] - v1dotdot[1]) / dy * v1dotdot[1],
    #                (v5dotdot[0] - v1dotdot[0]) / dx * v1dotdot[0] + (v5dotdot[1] - v1dotdot[1]) / dy * v1dotdot[1]]
    # sigma2dot = v1dot
    # sigma3dot = v1dotdot
    # sigma4dot = v1dotdotdot

    return sigma2dot, sigma3dot, sigma4dot


# ======================================================================================================================
# Robot Dynamics
# ======================================================================================================================
def robot_dynamics(pos, phi, theta, v_ref, fz, tau_x, tau_y, tau_z, p, q, r, R_euler, v_act):
    global g, dt, m
    v_dot = [0, 0, g] + 1 / m * np.matmul(R_euler, [0, 0, fz])
    # print 'v dot', v_dot
    # R_dot = np.matmul(R, omega)
    tau = [tau_x, tau_y, tau_z]
    wb = [p, q, r]
    Omega = [[0, -r, q], [r, 0, -p], [-q, p, 0]]
    wb_dot = np.matmul(np.matrix(J) ** -1, tau) - np.matmul(np.matrix(J) ** -1, np.matmul(Omega, np.matmul(J, wb)))
    # print 'wb dot = ', wb_dot
    wb += wb_dot * dt
    # print 'wb', np.array(wb)[0]

    phi += np.array(wb)[0][0] * dt
    theta += np.array(wb)[0][1] * dt
    # v_ref_pre = v_ref
    v_act += v_dot[0:2] * dt
    # print 'v actuall', v_act
    # print 'v_act', v_act[0:2]
    # print 'pos', pos
    pos += v_act * dt
    # print 'position', pos
    return [pos, v_ref, v_act, phi, theta, wb]


# ======================================================================================================================
# read a scenario
# ======================================================================================================================
def readScenario(scalex=1., scaley=1.):
    x_min = 0 * scalex - 2.
    y_min = 0 * scaley - 1.
    x_max = 10 * scalex + 2.
    y_max = 10 * scaley + 2.
    return x_min, x_max, y_min, y_max


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
                                          triangle[2][0], triangle[2][1], triangle[0][0], triangle[0][1],
                                          outline="black", fill="gray"))
    colors = ["white", "blue", "yellow", "#FAA"]
    circle = canvas.create_oval(0, 0, radius, radius,
                                fill=colors[3])  # color the disc of an agenr based on its group id
    velLine = canvas.create_line(0, 0, 10, 10, fill="white")
    gvLine = canvas.create_line(0, 0, 10, 10, fill="red")
    star = canvas.create_polygon(starPoints, fill="black")


# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():
    global starPoints
    # for i, triangle in enumerate(grid_triangles):
        # canvas.coords(Grid[i], world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin),
        #               world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
        #               world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
        #               world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
        #               world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
        #               world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
    canvas.coords(circle, world_scale * (pos[0] - radius - world_xmin), world_scale * (pos[1] - radius - world_ymin),
                  world_scale * (pos[0] + radius - world_xmin), world_scale * (pos[1] + radius - world_ymin))

    # newPoints = [pos[0], pos[1], pos[0], pos[1], pos[0], pos[1], pos[0], pos[1], pos[0], pos[1], pos[0], pos[1],
    #              pos[0], pos[1], pos[0], pos[1]]

    # newStar = [x+y for x,y in zip(starPoints, newPoints)]

    # canvas.create_polygon(newStar, fill ='black')

    canvas.coords(velLine, world_scale * (pos[0] - world_xmin), world_scale * (pos[1] - world_ymin),
                  world_scale * (pos[0] + radius * v_ref[0] * 2.5 - world_xmin),
                  world_scale * (pos[1] + radius * v_ref[1] * 2.5 - world_ymin))
    canvas.coords(gvLine, world_scale * (pos[0] - world_xmin), world_scale * (pos[1] - world_ymin),
                  world_scale * (pos[0] + radius * np.cos(psi) * 3.5 - world_xmin),
                  world_scale * (pos[1] + radius * np.sin(psi) * 3.5 - world_ymin))
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
    global g, pos, current_stage, v_ref, v_act, phi, theta, psi, v_ref_pre, phi_ref_pre, theta_ref_pre, psi_ref_pre, u_pre, \
        need_transition, x_2dot_ref_pre, y_2dot_ref_pre, x_3dot_ref_pre, y_3dot_ref_pre, p, q, r, u, Ixx, Iyy, Izz

    # R = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])

    # print 'R_euler', R_Euler, 'R_euler_transpose', np.transpose(R_Euler)
    # x = pos
    # x = np.matmul(R, np.array([[epsilon], [0]])).transpose()[0] + pos
    # v_ref, islast = vector_field.vector_field(pos[0], pos[1], current_stage)
    # u = control input u = [fz, tau_x, tau_y, tau_z]
    #################################################################################################################
    # Endogenous Transformation Calculation
    #################################################################################################################
    # x = pos[0]
    # y = pos[1]
    # z = 0
    # sigma2dot, sigma3dot, sigma4dot = flatoutput(pos, current_stage)

    v_ref, sigma2dot, sigma3dot, sigma4dot = velocity_vector(pos)

    # x_dot_ref = v_ref[0]
    # y_dot_ref = v_ref[1]
    # z_dot_ref = 0
    # x_2dot_ref = (v_ref[0] - v_ref_pre[0]) / dt
    # y_2dot_ref = (v_ref[1] - v_ref_pre[1]) / dt
    # z_2dot_ref = 0
    # x_3dot_ref = (x_2dot_ref - x_2dot_ref_pre) / dt
    # y_3dot_ref = (y_2dot_ref - y_2dot_ref_pre) / dt
    # z_3dot_ref = 0
    # x_4dot_ref = (x_3dot_ref - x_3dot_ref_pre) / dt
    # y_4dot_ref = (y_3dot_ref - y_3dot_ref_pre) / dt
    # z_4dot_ref = 0
    beta_a = -sigma2dot[0]
    beta_b = g
    beta_ab = math.sqrt(beta_a ** 2 + beta_b ** 2)
    beta_c = sigma2dot[1]
    theta = math.atan2(beta_a, beta_b)
    # print 'theta', theta
    phi = math.atan2(beta_c, beta_ab)
    psi = 0
    R_euler = [[np.cos(theta) * np.cos(psi), np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi),
                np.cos(phi) * np.sin(theta) * np.cos(psi) - np.sin(phi) * np.sin(psi)],
               [np.cos(theta) * np.sin(psi), np.sin(phi) * np.sin(theta) * np.sin(psi) - np.cos(phi) * np.cos(psi),
                np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi)],
               [-np.sin(theta), np.sin(phi) * np.cos(theta), np.cos(phi) * np.cos(theta)]]
    p = sigma3dot[1] * math.sqrt(sigma2dot[0] ** 2 + g ** 2) - sigma2dot[1] * sigma3dot[0] * sigma2dot[0] / math.sqrt(
        sigma2dot[0] ** 2 + g ** 2)
    q = - g * sigma3dot[0] / (sigma2dot[0] ** 2 + g ** 2)
    r = 0
    vzegond = ((sigma3dot[0] ** 2 + sigma2dot[0] * sigma4dot[0]) * (sigma2dot[0] ** 2 + g ** 2) - (sigma2dot[0] *
                                                                                                   sigma3dot[0])) / \
              (sigma2dot[0] ** 2 + g ** 2) ** 1.5
    numerator1 = (sigma4dot[1] * math.sqrt(sigma2dot[0] ** 2 + g ** 2) - sigma2dot[1] * vzegond)
    numerator2 = 2. * (
    sigma3dot[1] * math.sqrt(sigma2dot[0] ** 2 + g ** 2) - (sigma2dot[1] * sigma3dot[0] * sigma2dot[0] / \
                                                            (math.sqrt(sigma2dot[0] ** 2 + g ** 2)))) \
                 * (sigma3dot[1] * sigma2dot[1] + sigma3dot[0] * sigma2dot[0])
    p_dot = (numerator1 + numerator2) / (sigma2dot[0] ** 2 + sigma2dot[1] ** 2 + g ** 2) ** 2
    q_dot = (-sigma4dot[0] * g * (sigma2dot[0] ** 2 + g ** 2) + 2. * g * sigma2dot[0] * sigma3dot[0] ** 2) / \
            (sigma2dot[0] ** 2 + g ** 2) ** 2
    r_dot = 0
    #################################################################################################################
    # Control Inputs Calculation based on Flat Outputs
    #################################################################################################################
    kv = 0.5
    # print 'v act', v_act
    ev = math.sqrt(v_act[0] ** 2 + v_act[1] ** 2) - math.sqrt(v_ref[0] ** 2 + v_ref[1] ** 2)
    # print 'velocity error=', ev
    # print 'sigma2dot[0]', - m * math.sqrt(sigma2dot[0]**2 + sigma2dot[1]**2  + g**2)
    fz = +kv * ev - m * math.sqrt(sigma2dot[0] ** 2 + sigma2dot[1] ** 2 + g ** 2)
    # print 'fz vector', fz
    # fz = math.sqrt(fz_vector[0]**2 + fz_vector[1]**2)
    tau_x = Ixx * p_dot + (Izz - Iyy) * r * q
    tau_y = Iyy * q_dot + (Ixx - Izz) * p * r
    tau_z = Izz * r_dot + (Iyy - Ixx) * p * q
    # print 'tau_x,tau_y,tau_z', tau_x, tau_y, tau_z
    #################################################################################################################
    # omega_tensor = [[0, -r, q],[r, 0, -p],[-q, p, 0]]
    # print 'omega_tensor'
    # R_Euler_dot = np.matmul(R_Euler, omega_tensor)
    # omega_b_ref = np.matmul(np.transpose(R_Euler), R_Euler_dot)
    # print 'omega_b_ref tensor inverse', omega_b_ref
    # omega_b_ref = [p, q, r]
    # omega_b_dot_ref = np.matmul(np.linalg.inv(J), u[1:4]) -\
    #                   np.matmul(np.matmul( np.matmul(np.linalg.inv(J), omega_tensor), J), omega_b_ref)
    #################################################################################################################
    # u = np.zeros(4)
    # u = [fz, tau_x, tau_y, tau_z]

    if not np.any(v_ref):

        v_ref = -v_ref_pre
        # theta_ref = -theta_ref_pre
        # psi_ref = -psi_ref_pre
        # psi_ref = -psi_ref_pre
    else:
        psi_ref = math.atan2(v_ref[1], v_ref[0])

    # if islast and current_stage < Stages - 1:
    #     need_transition = True
    #     print 'need transition'
    #     u[0] = -u_pre[0]
    #
    # if need_transition:
    #     psi_err = psi_ref - psi
    #     if psi_err < np.pi/8 :
        # need_transition = False
        # current_stage += 1
        # else:
        #     psi_err = psi_ref - psi
        # psi_err = np.fmod(psi_err, 2 * np.pi)
        # u[1] = 0.9 * psi_err

    # if not need_transition:
    pos, v_ref, v_act, phi, theta, wb = robot_dynamics(pos, phi, theta, v_ref, fz, tau_x, tau_y, tau_z, p, q, r,
                                                           R_euler, v_act)
        # u[0] = -m * math.sqrt(x_2dot_ref**2 + y_2dot_ref**2 + g**2) # fz
        # u[1:4] = np.matmul(J, omega_b_dot_ref) + np.matmul(omega_tensor,np.matmul(J, omega_b_ref)) # tau_x, tau_y, tau_z
        # v_out = np.matmul(R * E2, u_pre)
        # v_err = .5*(v_ref - v_out)
        # E2invR = np.matmul(E2inv, R.transpose())
        # u = np.matmul(E2invR, v_err)
        # print 'fz', fz, 'phi', phi, 'theta', theta
        # print u, need_transition
        # u += u_pre
        # psi = 0
        # phi = p * dt
        # theta = q * dt

        # psi += u[1] * dt  # update the orintation
        # R = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
        # posdot = np.matmul(R, np.array([u[0], 0]))
        # posdot = [[0], [0], [g]] + 1/m * np.matmul(R_Euler, [[0],[0],[u[0]]]) #velocity
        # pos += posdot * dt  # update the position
        # psi = np.fmod(psi, 2*np.pi)
        # psi = 0
        # v_ref_pre = v_ref
        # x_2dot_ref_pre = x_2dot_ref
        # y_2dot_ref_pre  = y_2dot_ref
        # x_3dot_ref_pre = x_3dot_ref
        # y_3dot_ref_pre = y_3dot_ref
        # psi_ref_pre = psi_ref
        # phi_ref_pre = phi_ref
        # theta_ref_pre = phi_ref


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

# set the visualizer
win = Tk()
# keyboard interaction
win.bind("<space>", on_key_press)
win.bind("<space>", on_key_press)
win.bind("s", on_key_press)
win.bind("<Escape>", on_key_press)
win.bind("v", on_key_press)
# the drawing canvas
# canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#666")
canvas = Canvas(win, width=pixelsize, height=pixelsize*world_height/world_width, background="#000")
# canvas = Canvas(win, width=pixelsize, height=pixelsize * world_height / world_width, background="#999")
win.geometry("+{}+{}".format(2500, 100))
canvas.pack()
initWorld(canvas)
start_time = time.time()
# the main loop of the program
win.after(framedelay, lambda: drawFrame(dt))
mainloop()