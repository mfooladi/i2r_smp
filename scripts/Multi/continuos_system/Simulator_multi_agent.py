import numpy as np
import math
import time
from Tkinter import *
import triangle_plot
import vector_field
import matplotlib
from vector_fild_new import State
matplotlib.use("TkAgg")
import uConstraints
import vertice_gs

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
dt = .01
ittr = 0
maxIttr = 100000
globalTime = 0


def find_velocity(pos, tri, agent_id):
	velocity = vertice_gs.simple_point_vel(initial_points[agent_id + 1], tri)
	return velocity


class Robot(State):
	def __init__(self, id, geometry_name):
		super(Robot, self).__init__(geometry_name)
		self.agent_id = id
		self.position = []
		self.velocity = []
		self.trajectory = []
		self.reached = False
		self.triangle = []
		self.new_pos = []
		self.inital_position = []
		self.stage = []
		self.triangle_list = []
		self.need_transition = False
		self.robot_state = []
		self.pos = []
		self.current_stage = 0
		self.fz = 0.
		self.tau_x = 0.
		self.tau_y = 0.
		self.tau_z = 0.
		self.R_euler = [[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
		self.finished = False
		self.geometry_name = ['dscc_simulation', 'robot2']
		self.dt = 0.05
		self.xi = 0.
		self.yi = 0.
		self.psi, self.theta, self.phi = 0, 0, 0
		self.p, self.q, self.r = 0., 0., 0.
		self.radius = 0.10  # the radius of the agent
		self.epsilon = 0.1  # observable point distance to the center of the agent
		self.g = 9.81  # gravitational acc
		self.m = 1.  # mass
		self.Ixx = 0.0820
		self.Iyy = 0.0820
		self.Izz = 0.1377
		self.J = [[self.Ixx, 0, 0], [0, self.Iyy, 0], [0, 0, self.Izz]]  # agent moment of intertia
		self.dx = 0.0001
		self.dy = 0.0001
		self.islast = False
		self.v1 = [0., 0.]
		self.sigma2dot = [0., 0.]
		self.sigma3dot = [0., 0.]
		self.sigma4dot = [0., 0.]
		self.triangle_list = triangle_plot.read_triangles(self.geometry_name[self.agent_id - 1])
		for [i, j] in self.triangle_list[self.agent_id - 1]:
			self.xi += i
			self.yi += j
		self.xi = self.xi / 3.
		self.yi = self.yi / 3.
		self.inital_position.append([self.xi, self.yi])
		self.states = self.state_list
		self.current_state = 1
		self.stage.append(len(self.patch_list))
		self.stage.append(len(self.patch_list))
		# self.G = vector_field.init_field(self.geometry_name[self.agent_id - 1])[2]
		self.G = self.patch_list[self.current_stage]
		self.v_ref, self.islast, self.vertice1 = self.vector_field_point(self.inital_position,self.current_stage,self.current_state)
		self.v_act = self.v_ref * 1.
		self.pos = self.inital_position[0]
		self.v_max = 1.0
	
	def robot_dynamics(self, pos, phi, theta, v_ref, fz, tau_x, tau_y, tau_z, p, q, r, R_euler, v_act):
		
		self.v_dot = [0., 0., self.g] + 1. / self.m * np.matmul(R_euler, [0., 0., fz])
		self.tau = [tau_x, tau_y, tau_z]
		self.wb = [p, q, r]
		self.Omega = [[0., -r, q], [r, 0., -p], [-q, p, 0.]]
		self.wb_dot = np.matmul(np.matrix(self.J) ** -1, self.tau) - np.matmul(np.matrix(self.J) ** -1,
		                                                                       np.matmul(self.Omega,
		                                                                                 np.matmul(self.J, self.wb)))
		self.wb += self.wb_dot * dt
		self.phi += np.array(self.wb)[0][0] * dt
		self.theta += np.array(self.wb)[0][1] * dt
		self.v_act += self.v_dot[0:2] * dt
		self.pos += self.v_act * dt
		return [self.pos, self.v_ref, self.v_act, self.phi, self.theta, self.wb]
	
	def flatoutput(self, pos, current_stage, current_state):
		self.pos2 = pos + [self.dx, 0]
		self.pos3 = self.pos2 + [self.dx, 0]
		self.pos4 = self.pos3 + [self.dx, 0]
		self.pos5 = pos + [0, self.dy]
		self.pos6 = self.pos5 + [self.dx, 0]
		self.pos7 = self.pos6 + [self.dx, 0]
		self.pos8 = self.pos5 + [0, self.dy]
		self.pos9 = self.pos8 + [self.dx, 0]
		self.pos10 = self.pos8 + [0, self.dy]
		self.v1, self.islast, self.vertice1 = vector_field.vector_field(self.pos[0], self.pos[1], self.current_stage, self.states, self.G)
		self.v1, self.islast, self.vertice1 = self.vector_field_point(self.pos, self.current_stage, self.current_state)
		self.v2, self.islast, self.vertice1 = self.vector_field_point(self.pos2, self.current_stage, self.current_state)
		self.v3, self.islast, self.vertice1 = self.vector_field_point(self.pos3, self.current_stage, self.current_state)
		self.v4, self.islast, self.vertice1 = self.vector_field_point(self.pos4, self.current_stage, self.current_state)
		self.v5, self.islast, self.vertice1 = self.vector_field_point(self.pos5, self.current_stage, self.current_state)
		self.v6, self.islast, self.vertice1 = self.vector_field_point(self.pos6, self.current_stage, self.current_state)
		self.v7, self.islast, self.vertice1 = self.vector_field_point(self.pos7, self.current_stage, self.current_state)
		self.v8, self.islast, self.vertice1 = self.vector_field_point(self.pos8, self.current_stage, self.current_state)
		self.v9, self.islast, self.vertice1 = self.vector_field_point(self.pos9, self.current_stage, self.current_state)
		self.v10, self.islast, self.vertice1 = self.vector_field_point(self.pos10, self.current_stage, self.current_state)
		
		if self.v1 == None or self.v2 == None or self.v3 == None or self.v4 == None or self.v5 == None:
			self.finished = True
		else:
			self.v1dot = [
				(self.v2[0] - self.v1[0]) / self.dx * self.v1[0] + (self.v5[0] - self.v1[0]) / self.dy * self.v1[1],
				(self.v2[1] - self.v1[1]) / self.dx * self.v1[0] + (self.v5[1] - self.v1[1]) / self.dy * self.v1[1]]
			self.v2dot = [
				(self.v3[0] - self.v2[0]) / self.dx * self.v2[0] + (self.v6[0] - self.v2[0]) / self.dy * self.v2[1],
				(self.v3[1] - self.v2[1]) / self.dx * self.v2[0] + (self.v6[1] - self.v2[1]) / self.dy * self.v2[1]]
			self.v3dot = [
				(self.v4[0] - self.v3[0]) / self.dx * self.v3[0] + (self.v7[0] - self.v3[0]) / self.dy * self.v3[1],
				(self.v4[1] - self.v3[1]) / self.dx * self.v3[0] + (self.v7[1] - self.v3[1]) / self.dy * self.v3[1]]
			self.v5dot = [
				(self.v6[0] - self.v5[0]) / self.dx * self.v5[0] + (self.v8[0] - self.v5[0]) / self.dy * self.v5[1],
				(self.v6[1] - self.v5[1]) / self.dx * self.v5[0] + (self.v8[1] - self.v5[1]) / self.dy * self.v5[1]]
			self.v6dot = [
				(self.v7[0] - self.v6[0]) / self.dx * self.v6[0] + (self.v9[0] - self.v6[0]) / self.dy * self.v6[1],
				(self.v7[1] - self.v6[1]) / self.dx * self.v6[0] + (self.v9[1] - self.v6[1]) / self.dy * self.v6[1]]
			self.v8dot = [
				(self.v9[0] - self.v8[0]) / self.dx * self.v8[0] + (self.v10[0] - self.v8[0]) / self.dy * self.v8[1],
				(self.v9[1] - self.v8[1]) / self.dx * self.v8[0] + (self.v10[1] - self.v8[1]) / self.dy * self.v8[1]]
			
			self.v1dotdot = [
				(self.v2dot[0] - self.v1dot[0]) / self.dx * self.v1[0] + (self.v5dot[0] - self.v1dot[0]) / self.dy *
				self.v1[1],
				(self.v2dot[1] - self.v1dot[1]) / self.dx * self.v1[0] + (self.v5dot[1] - self.v1dot[1]) / self.dy *
				self.v1[1]]
			self.v2dotdot = [
				(self.v3dot[0] - self.v2dot[0]) / self.dx * self.v2[0] + (self.v6dot[0] - self.v2dot[0]) / self.dy *
				self.v2[1],
				(self.v3dot[1] - self.v2dot[1]) / self.dx * self.v2[0] + (self.v6dot[1] - self.v2dot[1]) / self.dy *
				self.v2[1]]
			self.v5dotdot = [
				(self.v6dot[0] - self.v5dot[0]) / self.dx * self.v5[0] + (self.v8dot[0] - self.v5dot[0]) / self.dy *
				self.v5[1],
				(self.v6dot[1] - self.v5dot[1]) / self.dx * self.v5[0] + (self.v8dot[1] - self.v5dot[1]) / self.dy *
				self.v5[1]]
			
			self.v1dotdotdot = [(self.v2dotdot[0] - self.v1dotdot[0]) / self.dx * self.v1[0] + (
				self.v5dotdot[0] - self.v1dotdot[0]) / self.dy * self.v1[1],
			                    (self.v2dotdot[1] - self.v1dotdot[1]) / self.dx * self.v1[0] + (
				                    self.v5dotdot[1] - self.v1dotdot[1]) / self.dy * self.v1[1]]
			
			self.sigma2dot = self.v1dot
			self.sigma3dot = self.v1dotdot
			self.sigma4dot = self.v1dotdotdot
		return self.islast, self.v1, self.sigma2dot, self.sigma3dot, self.sigma4dot
	
	def control_inputs(self, islast, v_ref, sigma2dot, sigma3dot, sigma4dot):
		self.beta_a = -sigma2dot[0]
		self.beta_b = self.g
		self.beta_ab = math.sqrt(self.beta_a ** 2 + self.beta_b ** 2)
		self.beta_c = sigma2dot[1]
		self.theta = math.atan2(self.beta_a, self.beta_b)
		self.phi = math.atan2(self.beta_c, self.beta_ab)
		self.psi = 0
		self.R_euler = [[np.cos(self.theta) * np.cos(self.psi), np.sin(self.phi) * np.sin(self.theta) * np.cos(self.psi) - np.cos(self.phi) *
		                 np.sin(self.psi),
		                 np.cos(self.phi) * np.sin(self.theta) * np.cos(self.psi) - np.sin(self.phi) * np.sin(self.psi)],
		                [np.cos(self.theta) * np.sin(self.psi),
		                 np.sin(self.phi) * np.sin(self.theta) * np.sin(self.psi) - np.cos(self.phi) * np.cos(self.psi),
		                 np.cos(self.phi) * np.sin(self.theta) * np.sin(self.psi) - np.sin(self.phi) * np.cos(self.psi)],
		                [-np.sin(self.theta), np.sin(self.phi) * np.cos(self.theta), np.cos(self.phi) * np.cos(self.theta)]]
		self.p = sigma3dot[1] * math.sqrt(sigma2dot[0] ** 2 + self.g ** 2) - sigma2dot[1] * sigma3dot[0] * sigma2dot[
			0] / math.sqrt(sigma2dot[0] ** 2 + self.g ** 2)
		self.q = - self.g * sigma3dot[0] / (sigma2dot[0] ** 2 + self.g ** 2)
		self.r = 0
		self.vzegond = ((sigma3dot[0] ** 2 + sigma2dot[0] * sigma4dot[0]) * (sigma2dot[0] ** 2 + self.g ** 2) - (sigma2dot[0] *
		                                                                                                         sigma3dot[0])) / \
		               (sigma2dot[0] ** 2 + self.g ** 2) ** 1.5
		self.numerator1 = (sigma4dot[1] * math.sqrt(sigma2dot[0] ** 2 + self.g ** 2) - sigma2dot[1] * self.vzegond)
		self.numerator2 = 2. * (
			sigma3dot[1] * math.sqrt(sigma2dot[0] ** 2 + self.g ** 2) - (sigma2dot[1] * sigma3dot[0] * sigma2dot[0] / \
			                                                             (math.sqrt(sigma2dot[0] ** 2 + self.g ** 2)))) \
		                  * (sigma3dot[1] * sigma2dot[1] + sigma3dot[0] * sigma2dot[0])
		self.p_dot = (self.numerator1 + self.numerator2) / (sigma2dot[0] ** 2 + sigma2dot[1] ** 2 + self.g ** 2) ** 2
		self.q_dot = (-sigma4dot[0] * self.g * (sigma2dot[0] ** 2 + self.g ** 2) + 2. * self.g * sigma2dot[0] *
		              sigma3dot[0] ** 2) / \
		             (sigma2dot[0] ** 2 + self.g ** 2) ** 2
		self.r_dot = 0
		self.kv = -.9
		self.ev = math.sqrt((self.v_act[0] - self.v_ref[0]) ** 2 + (self.v_act[1] - self.v_ref[1]) ** 2)
		self.fz = -self.kv * self.ev - self.m * math.sqrt(sigma2dot[0] ** 2 + sigma2dot[1] ** 2 + self.g ** 2)
		self.tau_x = self.Ixx * self.p_dot + (self.Izz - self.Iyy) * self.r * self.q
		self.tau_y = self.Iyy * self.q_dot + (self.Ixx - self.Izz) * self.p * self.r
		self.tau_z = self.Izz * self.r_dot + (self.Iyy - self.Ixx) * self.p * self.q
	
	def collision_check(self, p1, p2, v1, v2, agent_id):  # Velocity Obstacle method
		feasible_gs = []
		radius_total = 1.  # radii of the two UAVs r1+r2 = radious_total
		distance_u2u = np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
		theta_half = math.asin(radius_total / distance_u2u)
		theta_f = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
		theta_right = theta_f - theta_half
		theta_left = theta_f + theta_half
		p_ab = v2
		p_vo_right = (p_ab + np.array([math.cos(theta_right), math.sin(theta_right)]) * 2. * self.v_max + v1) / 2.
		p_vo_left = (p_ab + np.array([math.cos(theta_left), math.sin(theta_left)]) * 2. * self.v_max + v1) / 2.
		points_to_check = []
		g_set_1 = uConstraints.point_g_calculator(point_xy = p1, vertice1 = self.triangle_list[self.vertice1 - 1][0],
		                                          common_facet_points = [self.triangle_list[self.vertice1][1], self.triangle_list[self.vertice1][2]],
		                                          vel_max = self.v_max)
		sides = len(g_set_1)
		for i in range(0, sides):
			[x, y] = g_set_1[i]
			if agent_id == 1:
				right_sign = np.sign(y - p_ab[1] - math.tan(theta_right) * (x - p_ab[0])) * np.sign(
					np.tan(theta_right)) * np.sign(-1 ** (self.agent_id + 1))
				left_sign = np.sign(np.tan(theta_left)) * np.sign(
					y - p_ab[1] - math.tan(theta_left) * (x - p_ab[0])) * np.sign(-1 ** (agent_id + 1))
			else:
				right_sign = np.sign(y - p_ab[1] - math.tan(theta_right) * (x - p_ab[0])) * np.sign(
					np.tan(theta_right)) * np.sign(-1 ** (self.agent_id + 1))
				left_sign = np.sign(np.tan(theta_left)) * np.sign(
					y - p_ab[1] - math.tan(theta_left) * (x - p_ab[0])) * np.sign(-1 ** (agent_id + 1))
			feasible_gs.append([i, right_sign, left_sign])
		best_left = False
		best_right = False
		best_right_index = None
		best_left_index = None
		for ind, right_sign, left_sign in feasible_gs:
			velocity_magnitude = np.sqrt(g_set_1[i][0] ** 2. + g_set_1[i][1] ** 2.)
			if self.agent_id == 1:
				if right_sign - feasible_gs[ind - 1][1] > 0:
					best_right_index = ind - 1
					best_right = True
				if left_sign - feasible_gs[ind - 1][2] > 0:
					best_left_index = ind
					best_left = True
			if self.agent_id == 2:
				if right_sign - feasible_gs[ind - 1][1] < 0:
					best_right_index = ind - 1
					best_right = True
				if left_sign - feasible_gs[ind - 1][2] < 0:
					best_left_index = ind
					best_left = True
		point_intersect_right = []
		point_intersect_left = []
		if best_right == True:
			points_to_check.append(g_set_1[best_right_index])
			right_out = best_right_index
			right_in = best_right_index + 1
			a1_right = math.tan(theta_right)
			b1_right = p_ab[1] - ((p_ab[1] - p_vo_right[1]) / (p_ab[0] - p_vo_right[0])) * p_ab[0]
			if g_set_1[right_out][0] == g_set_1[right_in][0]:
				point_intersect_right.append([g_set_1[right_in][0], a1_right * g_set_1[right_in][0] + b1_right])
				points_to_check.append(np.array(point_intersect_right[0]))
			else:
				a2_right = (g_set_1[right_out][1] - g_set_1[right_in][1]) / (
					g_set_1[right_out][0] - g_set_1[right_in][0])
				b2_right = g_set_1[right_out][1] - a2_right * g_set_1[right_out][0]
				point_intersect_right.append([(b2_right - b1_right) / (a1_right - a2_right),
				                              a1_right * (b2_right - b1_right) / (a1_right - a2_right) + b1_right])
				points_to_check.append(np.array(point_intersect_right[0]))
		if best_left == True:
			points_to_check.append(g_set_1[best_left_index])
			left_out = best_left_index
			left_in = best_left_index - 1
			# left intersection point
			a1_left = math.tan(theta_left)
			b1_left = p_ab[1] - ((p_ab[1] - p_vo_left[1]) / (p_ab[0] - p_vo_left[0])) * p_ab[0]
			if g_set_1[left_out][0] == g_set_1[left_in][0]:
				point_intersect_left.append([g_set_1[left_in][0], a1_left * g_set_1[left_in][0] + b1_left])
				points_to_check.append(np.array(point_intersect_left[0]))
			else:
				a2_left = (g_set_1[left_out][1] - g_set_1[left_in][1]) / (g_set_1[left_out][0] - g_set_1[left_in][0])
				b2_left = g_set_1[left_out][1] - a2_left * g_set_1[left_out][0]
				point_intersect_left.append([(b2_left - b1_left) / (a1_left - a2_left),
				                             a1_left * (b2_left - b1_left) / (a1_left - a2_left) + b1_left])
				points_to_check.append(np.array(point_intersect_left[0]))
		if agent_id == 1:
			v_ref = find_velocity(p1, self.triangle_list[self.vertice1 - 1], agent_id)
		else:
			v_ref = find_velocity(p1, tri_2, agent_id)
		
		if len(points_to_check) == 0:
			optimal_point = v_ref[0]
		else:
			distance_to_v_ref = v_ref - points_to_check
			optimal_point = points_to_check[np.argmin(np.sqrt(x ** 2 + y ** 2) for (x, y) in distance_to_v_ref[0])]
		return optimal_point

robot2 = Robot(2, 'robot2')
robot1 = Robot(1, 'robot2')
robot1.collision_check(robot1.pos, robot2.pos, robot1.v_ref, robot2.v_ref, robot1.agent_id)
print ''

def collision_simulator():
	dt = 0.1
	r1_reached = False
	r2_reached = False
	trajectory_r_1 = []
	trajectory_r_2 = []
	while not r1_reached or not r2_reached:
		# velocities = [find_velocity(positions[0], tri_1, 1), find_velocity(positions[1], tri_2, 2)]
		# plt.plot(positions[0][0], positions[0][1], '.r')
		# plt.plot(positions[1][0], positions[1][1], '.b')
		
		if r1_reached == False:
			if r2_reached == True:
				vo_ab = collision_check(positions[0], trajectory_r_2[-2], velocities[0], velocities[1], 1)
			else:
				vo_ab = collision_check(positions[0], positions[1], velocities[0], velocities[1], 1)
			new_pos_1 = positions[0] + vo_ab * dt
			# print 'new pos 1', new_pos_1
			positions[0] = new_pos_1
			velocities[0] = find_velocity(new_pos_1, tri_1, 1)
			if new_pos_1[1] < 0:
				r1_reached = True
				print 'r1_reached = True'
			# velocities[0] = 0
			trajectory_r_1.append(new_pos_1)
		# plt.plot(new_pos_1,'*k')
		# plt.show()
		if r2_reached == False:
			if r1_reached == True:
				vo_ba = collision_check(positions[1], trajectory_r_1[-2], velocities[1], velocities[0], 2)
			else:
				vo_ba = collision_check(positions[1], positions[0], velocities[1], velocities[0], 2)
			new_pos_2 = positions[1] + vo_ba * dt
			# print 'new pos 2', new_pos_2
			positions[1] = new_pos_2
			velocities[1] = find_velocity(new_pos_2, tri_2, 2)


# ======================================================================================================================
# initialize the agents
# ======================================================================================================================
def initWorld(canvas):
	global circle1, circle2, velLine, gvLine, star
	print ("")
	print ("Simulation of a robot in Triangular environment.")
	print ("White Arrow is Current Velocity")
	print ("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
	print ("")
	for i, triangle in enumerate(robot1.triangle_list):
		Grid.append(canvas.create_polygon(triangle[0][0], triangle[0][1], triangle[1][0], triangle[1][1],
		                                  triangle[1][0], triangle[1][1], triangle[2][0], triangle[2][1],
		                                  triangle[2][0], triangle[2][1], triangle[0][0], triangle[0][1], outline = "black", fill = "gray"))
	colors = ["white", "blue", "yellow", "#FAA"]
	# circle1 = canvas.create_oval(0, 0, robot1.radius, robot1.radius,
	# 							fill=colors[3])  # color the disc of an agenr based on its group id
	# circle2 = canvas.create_oval(0, 0, robot2.radius, robot2.radius,
	#                             fill = colors[2])  # color the disc of an agenr based on its group id
	velLine = canvas.create_line(0, 0, 10, 10, fill = "white")
	gvLine = canvas.create_line(0, 0, 10, 10, fill = "red")


# star = canvas.create_polygon(starPoints, fill="black")
# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():
	for i, triangle in enumerate(robot2.triangle_list):
		canvas.coords(Grid[i], world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin),
		              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
		              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
		              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
		              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
		              world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
	# print circle1, world_scale * (robot1.pos[0] - world_xmin), world_scale * (robot1.pos[1] - world_ymin)
	canvas.coords(uav_1_image, world_scale * (robot1.pos[0] - world_xmin), world_scale * (robot1.pos[1] - world_ymin))
	canvas.coords(uav_2_image, world_scale * (robot2.pos[0] - world_xmin), world_scale * (robot2.pos[1] - world_ymin))


# 	canvas.coords(circle2, world_scale * (robot2.pos[0] - robot2.radius - world_xmin), world_scale * (robot2.pos[1] -
# 	                                                                                                  robot2.radius -
# 	                                                                                                  world_ymin),
# 	              world_scale * (robot2.pos[0] + robot2.radius - world_xmin), world_scale * (robot2.pos[1] +
# 	                                                                                         robot2.radius -
# 	                                                                                         world_ymin))
# 	canvas.coords(circle1, world_scale * (robot1.pos[0] - robot1.radius - world_xmin), world_scale * (robot1.pos[1] -
# 	                                                                                                  robot1.radius -
# 	                                                                                                  world_ymin),
# 	              world_scale * (robot1.pos[0] + robot1.radius - world_xmin), world_scale * (robot1.pos[1] +
# 	                                                                                         robot1.radius -
# 	                                                                                         world_ymin))
# #
# canvas.coords(velLine,world_scale*(robot1.pos[0] - world_xmin), world_scale*(robot1.pos[1] - world_ymin),
# 			  world_scale*(robot1.pos[0]+ robot1.radius*robot1.v_ref[0]*2.5 - world_xmin), world_scale*(
# 	              robot1.pos[1] +
# 			                                                                               robot1.radius*robot1.v_ref[
# 				                                                                              1]*2.5 - world_ymin))
# canvas.coords(gvLine, world_scale * (robot1.pos[0] - world_xmin), world_scale * (robot1.pos[1] - world_ymin),
#               world_scale * (robot1.pos[0] + robot1.radius * np.cos(robot1.psi) * 1.5 - world_xmin),
#               world_scale * (robot1.pos[1] + robot1.radius * np.sin(robot1.psi) * 1.5 - world_ymin))
# if drawVels:
# 	canvas.itemconfigure(velLine, state="normal")
# 	canvas.itemconfigure(gvLine, state="normal")
# else:
# 	canvas.itemconfigure(velLine, state="hidden")
# 	canvas.itemconfigure(gvLine, state="hidden")
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
	robot2.islast, robot2.v_ref, robot2.sigma2dot, robot2.sigma3dot, robot2.sigma4dot = robot2.flatoutput(robot2.pos, robot2.current_stage)
	robot1.islast, robot1.v_ref, robot1.sigma2dot, robot1.sigma3dot, robot1.sigma4dot = robot1.flatoutput(robot1.pos, robot1.current_stage)
	
	if robot1.finished == False:
		robot1.control_inputs(robot1.islast, robot1.v_ref, robot1.sigma2dot, robot1.sigma3dot, robot1.sigma4dot)
		if robot1.islast and robot1.current_stage < robot1.stage[0] - 1:
			robot1.need_transition = True
			robot1.current_stage += 1
			print 'need transition'
		
		if robot1.need_transition:
			robot1.need_transition = False
			robot1.v_ref, robot1.islast, robot1.vertice1 = vector_field.vector_field(robot1.pos[0], robot1.pos[1], robot1.current_stage)
		
		if not robot1.need_transition:
			robot1.pos, robot1.v_ref, robot1.v_act, robot1.phi, robot1.theta, robot1.wb = robot1.robot_dynamics(robot1.pos, robot1.phi, robot1.theta, robot1.v_ref, robot1.fz,
			                                                                                                    robot1.tau_x, robot1.tau_y, robot1.tau_z, robot1.p, robot1.q, robot1.r,
			                                                                                                    robot1.R_euler, robot1.v_act)
	
	if robot2.finished == False:
		robot2.control_inputs(robot2.islast, robot2.v_ref, robot2.sigma2dot, robot2.sigma3dot, robot2.sigma4dot)
		
		if robot2.islast and robot2.current_stage < robot2.stage[0] - 1:
			robot2.need_transition = True
			robot2.current_stage += 1
			print 'need transition'
		
		if robot2.need_transition:
			robot2.need_transition = False
			robot2.v_ref, robot2.islast, robot2.vertice1 = vector_field.vector_field(robot2.pos[0], robot2.pos[1],
			                                                                         robot2.current_stage)
		
		if not robot2.need_transition:
			robot2.pos, robot2.v_ref, robot2.v_act, robot2.phi, robot2.theta, robot2.wb = \
				robot2.robot_dynamics(robot2.pos, robot2.phi, robot2.theta, robot2.v_ref, robot2.fz, robot2.tau_x,
				                      robot2.tau_y, robot2.tau_z, robot2.p, robot2.q, robot2.r, robot2.R_euler,
				                      robot2.v_act)

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
	global start_time, step, paused, ittr, globalTime
	if ittr > maxIttr or QUIT:  # Simulation Loop
		print("%s itterations ran ... quitting" % ittr)
		win.destroy()
	else:
		start_time = time.time()
		if not paused:
			updateSim(dt)
			ittr += 1
			globalTime += dt
		drawWorld()
		if step == True:
			step = False
			paused = True
		win.after(framedelay, lambda: drawFrame(dt))

# ======================================================================================================================
# Main execution of the code
# ======================================================================================================================
world_xmin, world_xmax, world_ymin, world_ymax = readScenario()
world_width = world_xmax - world_xmin
world_height = world_ymax - world_ymin
world_scale = pixelsize / world_width
win = Tk()
win.bind("<space>", on_key_press)
win.bind("<space>", on_key_press)
win.bind("s", on_key_press)
win.bind("<Escape>", on_key_press)
win.bind("v", on_key_press)
canvas = Canvas(win, width = pixelsize, height = pixelsize * world_height / world_width, background = "#F66733")
win.geometry("+{}+{}".format(2500, 100))
canvas.pack()
initWorld(canvas)
start_time = time.time()
win.title('UniCycle Simulation')
win.after(framedelay, lambda: drawFrame(dt))
quad = PhotoImage(file = 'quad.gif')
quad = quad.subsample(35, 35)
uav_1_image = canvas.create_image(200, 100, image = quad)
uav_2_image = canvas.create_image(200, 100, image = quad)
mainloop()
