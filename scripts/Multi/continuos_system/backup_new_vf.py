import numpy as np
import math
from interval import interval
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import copy
import time
import os.path
from Tkinter import *
import uConstraints
from operator import add
import JarvisMarch

parent_dir = os.path.dirname(__file__) + '/../'


class Vertice(object):
	def __init__(self, geometry_name):
		with open(parent_dir + 'geometry/{}.1.node'.format(geometry_name)) as vpos:
			vertices_pos = vpos.readlines()
		vpos.close()
		del (vertices_pos[-1])
		self.vertices_coords_temp = []
		for i in xrange(1, len(vertices_pos)):
			vertice = vertices_pos[i].strip().split(' ')
			vertice = filter(None, vertice)
			vertice = map(float, vertice[1:3])
			self.vertices_coords_temp.append(vertice)
	
	def position(self, tri):
		return self.vertices_coords_temp[tri - 1]


class Triangle(Vertice):
	def __init__(self, geometry_name):
		super(Triangle, self).__init__(geometry_name)
		self.geometry_name = geometry_name
		self.number_of_triangles = 0
		self.triangle_nodes = []  # triangles = [[#ver.1, #ver.2, #ver.3],...,[#ver.1, #ver.2, #ver.3]]
		with open(parent_dir + 'geometry/{}.1.ele'.format(geometry_name)) as elements:
			triangle_vertice_list = elements.readlines()
		elements.close()
		del (triangle_vertice_list[-1])
		# Triangles numbers and Vertices numbers
		for i in xrange(1, len(triangle_vertice_list)):
			triangle_nodes = triangle_vertice_list[i].strip().split(' ')
			triangle_nodes = filter(None, triangle_nodes)
			triangle_nodes = map(int, triangle_nodes[1:])
			self.triangle_nodes.append(triangle_nodes)
		self.number_of_triangles = len(self.triangle_nodes)
		
		with open(parent_dir + 'geometry/{}.1.neigh'.format(geometry_name)) as neighbors_text:
			neighbors = neighbors_text.readlines()
		neighbors_text.close()
		del (neighbors[-1])
		self.neighbors_list_temp = []
		for i in xrange(1, len(neighbors)):
			neighbor_num = neighbors[i].strip().split(' ')
			neighbor_num = filter(None, neighbor_num)
			self.neighbors_list_temp.append(neighbor_num)
	
	def vertices(self, tri):
		return self.triangle_nodes[tri - 1]
	
	def vertices_poses(self, tri):
		return [self.position(value) for value in self.vertices(tri)]
	
	def neighbors_list(self, tri):
		return [int(value) for value in self.neighbors_list_temp[tri - 1] if value != '-1'][1:]
	
	def common_vertices(self, tri1, tri2):
		return [value for value in self.triangle_nodes[tri1 - 1] if value in self.triangle_nodes[tri2 - 1]]


class State(Triangle):
	def __init__(self, geometry_name):
		super(State, self).__init__(geometry_name)
		self.state_list = ''
		with open(parent_dir + 'discrete_transition/state_order_{}.txt'.format(geometry_name)) as state:
			self.state_list = state.readlines()
		state.close()
		self.state_total_number = len(self.state_list)
		self.last_state = self.state_list[-1]
		self.gs = []
		self.global_vertex_index = []
		self.vector_field = dict()
		self.last_valid_patch = []
	
	def triangle_number(self, state_number):
		return self.state_list[state_number - 1]
	
	def vertices(self, state_num):
		return super(State, self).vertices(int(self.triangle_number(state_num)))
	
	#
	# def vertices_poses(self, state_num):
	# 	return super(State, self).vertices_poses(state_num)
	
	def common_vertices(self, state1, state2):
		return super(State, self).common_vertices(int(self.triangle_number(state1)), int(self.triangle_number(state2)))
	
	def v1v2v3(self, state_number):
		# v1 = [], v2 = [], v3 = []
		if state_number == self.state_total_number:
			v1 = [value for value in self.vertices(state_number) if value not in
			      self.common_vertices(state_number, state_number - 1)][0]
			v2 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) + 1, 3)]
			v3 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) - 1, 3)]
		else:
			v1 = [value for value in self.vertices(state_number) if value not in
			      self.common_vertices(state_number, state_number + 1)][0]
			v2 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) + 1, 3)]
			v3 = self.vertices(state_number)[np.mod(self.vertices(state_number).index(v1) - 1, 3)]
		return [v1, v2, v3]
	
	def initial_gs(self):
		v_pos = [[0., 0.], [0., 0.], [0., 0.]]
		for state_iter in range(1, self.state_total_number + 1):
			for i in range(0, 3):
				v_pos[i] = self.position(self.v1v2v3(state_iter)[i])
			v12_vector = np.array(v_pos[1]) - np.array(v_pos[0])
			v12_angle = math.degrees(math.atan2(v12_vector[1], v12_vector[0]))
			v13_vector = np.array(v_pos[2]) - np.array(v_pos[0])
			v13_angle = math.degrees(math.atan2(v13_vector[1], v13_vector[0]))
			v23_vector = np.array(v_pos[2]) - np.array(v_pos[1])
			v23_angle = math.degrees(math.atan2(v23_vector[1], v23_vector[0]))
			v32_vector = np.array(v_pos[1]) - np.array(v_pos[2])
			v32_angle = math.degrees(math.atan2(v32_vector[1], v32_vector[0]))
			if v12_angle > v13_angle:
				int1 = interval([v12_angle, 180.], [-180., v13_angle])
			else:
				int1 = interval([v12_angle, v13_angle])
			
			if v12_angle > v23_angle:
				int2 = interval([v12_angle, 180.], [-180., v23_angle])
			else:
				int2 = interval([v12_angle, v23_angle])
			
			if v32_angle > v13_angle:
				int3 = interval([v32_angle, 180.], [-180., v13_angle])
			else:
				int3 = interval([v32_angle, v13_angle])
			self.gs.append([int1, int2, int3])
		# print self.gs, 'gs'
	
	def last_triangle_gs(self, last_tri_vs):
		self.last_tri_gs = []
		verts = [[0., 0.], [0., 0.], [0., 0.]]
		for i in range(0, 3):
			verts[i] = self.position(last_tri_vs[i])
		
		for i in range(0, 3):
			vector_right = np.array(verts[np.mod(i + 1, 3)] - np.array(verts[np.mod(i, 3)]))
			vector_left = np.array(verts[np.mod(i - 1, 3)] - np.array(verts[np.mod(i, 3)]))
			angle_right = math.degrees(math.atan2(vector_right[1], vector_right[0]))
			angle_left = math.degrees(math.atan2(vector_left[1], vector_left[0]))
			if angle_right > angle_left:
				self.last_tri_gs.append(interval([angle_right, 180.], [-180., angle_left]))
			else:
				self.last_tri_gs.append(interval([angle_right, angle_left]))
		return self.last_tri_gs
	
	def vertex_ijk(self):
		self.global_vertex_index.append(self.v1v2v3(1)[0])
		self.global_vertex_index.append(self.v1v2v3(1)[1])
		self.global_vertex_index.append(self.v1v2v3(1)[2])
		for j in range(1, self.state_total_number):
			self.global_vertex_index.append(list(set(self.v1v2v3(j + 1)) - set(self.common_vertices(j, j + 1)))[0])
		print self.global_vertex_index, 'global vertex index'
	
	def patches(self):
		self.patch_list = []
		self.patch_state_list = []
		last_valid_patch = dict()
		state_iter = 1
		while state_iter <= self.state_total_number:
			self.patch_state_list.append(state_iter)
			for i in range(0, 3):
				if self.vector_field.has_key(self.v1v2v3(state_iter)[i]):
					self.vector_field[self.v1v2v3(state_iter)[i]] = self.vector_field[self.v1v2v3(state_iter)[i]] & self.gs[state_iter - 1][i]
				else:
					self.vector_field[self.v1v2v3(state_iter)[i]] = self.gs[state_iter - 1][i]
				if ((len([value for value in self.vector_field.iterkeys() if self.vector_field[value] == interval()]) > 0) or (len([value for value in self.vector_field.iterkeys() if
				                                                                                                                    (self.vector_field[value]).midpoint == (
						                                                                                                                    self.vector_field[value]).extrema]) > 0)):
					last_tri_g = self.last_triangle_gs(self.v1v2v3(state_iter))
					for k in range(0, 3):
						if last_valid_patch.has_key(self.v1v2v3(state_iter)[k]):
							last_valid_patch[self.v1v2v3(state_iter)[k]] = last_valid_patch[self.v1v2v3(state_iter)[k]] & last_tri_g[k]
						else:
							last_valid_patch[self.v1v2v3(state_iter)[k]] = last_tri_g[k]
					self.patch_list.append(last_valid_patch)
					print 'patch number {} is ready!'.format(len(self.patch_list)), self.patch_list[-1]
					self.vector_field = dict()
					state_iter = state_iter - 1
					break
				last_valid_patch = copy.deepcopy(self.vector_field)
			if state_iter == self.state_total_number:
				self.patch_list.append(self.vector_field)
			state_iter = state_iter + 1
		
		print 'final patch list', self.patch_list
		print 'number of patches is {}'.format(len(self.patch_list))
	
	def vector_field_point(self, point, patch_num, state_num, v_max):
		last = False
		last_patches_total_length = 0
		for i in range(0, patch_num):
			last_patches_total_length += len(self.patch_list[i]) - 2
		local_state = state_num + patch_num - 1 - last_patches_total_length
		g1_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[0]].midpoint
		g2_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[1]].midpoint
		g3_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[2]].midpoint
		g1 = np.array([np.cos(np.radians(g1_angle)[0][0]), np.sin(np.radians(g1_angle)[0][0])])
		g2 = np.array([np.cos(np.radians(g2_angle)[0][0]), np.sin(np.radians(g2_angle)[0][0])])
		g3 = np.array([np.cos(np.radians(g3_angle)[0][0]), np.sin(np.radians(g3_angle)[0][0])])
		v1 = self.position(self.v1v2v3(state_num)[0])
		v2 = self.position(self.v1v2v3(state_num)[1])
		v3 = self.position(self.v1v2v3(state_num)[2])
		Glocal = np.matrix([g1, g2, g3]).transpose()
		W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
		inv_W = np.matrix(W) ** -1
		GW = Glocal * inv_W
		[x, y] = [value for value in point]
		velocity_at_point = np.array(GW * [[x], [y], [1]]).transpose()[0] * v_max
		vertice1_pos = self.position(self.v1v2v3(state_num)[0])
		if len(self.patch_list[patch_num - 1]) == local_state:
			last = True
		return [velocity_at_point, last, vertice1_pos]
	
	def gConstraints(self, point_xy, patch_num, state_num, v_max):
		g_6points = []
		for i in range(0, 3):
			default_gs = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[i]]
			print 'default gs', default_gs
			
			if len(default_gs) == 2:
				g_6points.append([[np.cos(math.radians(default_gs[0][1])), np.sin(math.radians(default_gs[0][1]))], [np.cos(math.radians(default_gs[1][0])), np.sin(math.radians(
					default_gs[1][0]))]])
			else:
				g_6points.append([[np.cos(math.radians(default_gs[0][0])), np.sin(math.radians(default_gs[0][0]))], [np.cos(math.radians(default_gs[0][1])), np.sin(math.radians(
					default_gs[0][1]))]])
		
		print 'g6points', g_6points
		v = [self.position(self.v1v2v3(self.current_state)[0]), self.position(self.v1v2v3(self.current_state)[1]), self.position(self.v1v2v3(self.current_state)[2])]
		W = [[v[0][0], v[1][0], v[2][0]], [v[0][1], v[1][1], v[2][1]], [1.0, 1.0, 1.0]]
		inv_W = np.matrix(W) ** -1
		x, y = point_xy[0], point_xy[1]
		# lambdas = np.dot(vel_max,np.array(inv_W * [[x],[y], [1]]))  # .transpose()
		lambdas = np.array(inv_W * [[x], [y], [1]])
		print 'lambda', lambdas
		g_total_333 = [[[0., 0.], g_6points[0][0], g_6points[0][1]], [[0., 0.], g_6points[1][0], g_6points[1][1]], [[0., 0.], g_6points[2][0], g_6points[2][1]]]
		print 'g total 333', g_total_333
		t1 = lambdas[0] * g_total_333[0]
		t2 = lambdas[1] * g_total_333[1]
		t3 = lambdas[2] * g_total_333[2]
		points = list()
		it = 0
		for i in t1:
			for j in t2:
				for k in t3:
					it += 1
					points.append(map(add, map(add, i, j), k))
		try:
			JarvisMarch.main(np.array(points))
		except ValueError:
			L = JarvisMarch.main(np.array(points))
		L = JarvisMarch.main(np.array(points))
		return L
	
	def plot_triangles(self):
		for i in range(0, self.number_of_triangles):
			for k in range(0, 3):
				plt.plot([self.position(self.triangle_nodes[i][k])[0], self.position(self.triangle_nodes[i][np.mod(k + 1, 3)])[0]],
				         [self.position(self.triangle_nodes[i][k])[1], self.position(self.triangle_nodes[i][np.mod(k + 1, 3)])[1]], 'k')
			# plt.show()
	
	def plot_vf_one_triangle(self, state_num = 1, patch_num = 1):
		g1_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[0]].midpoint
		g2_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[1]].midpoint
		g3_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[2]].midpoint
		g1 = np.array([np.cos(np.radians(g1_angle)[0][0]), np.sin(np.radians(g1_angle)[0][0])])
		g2 = np.array([np.cos(np.radians(g2_angle)[0][0]), np.sin(np.radians(g2_angle)[0][0])])
		g3 = np.array([np.cos(np.radians(g3_angle)[0][0]), np.sin(np.radians(g3_angle)[0][0])])
		v1 = self.position(self.v1v2v3(state_num)[0])
		v2 = self.position(self.v1v2v3(state_num)[1])
		v3 = self.position(self.v1v2v3(state_num)[2])
		Glocal = np.matrix([g1, g2, g3]).transpose()
		W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
		inv_W = np.matrix(W) ** -1
		GW = Glocal * inv_W
		for l1 in np.linspace(0, 1, 10):
			for l2 in np.linspace(0, 1. - l1, int(10 * (1 - l1))):
				l3 = 1 - l1 - l2
				# print 'l1', l1, 'l2', l2, 'l3', l3
				[x, y] = l1 * np.array(v1) + l2 * np.array(v2) + l3 * np.array(v3)
				# plt.plot(x,y, '*')
				v = np.array(GW * [[x], [y], [1]]).transpose()[0]
				# print 'vx vy x y', v[0], v[1], x, y
				plt.quiver(x, y, v[0], v[1], scale = 5, scale_units = 'xy', width = 0.0015)
			# plt.show()
	
	def plot_patch_vf(self, single_patch = True, patch_num = 1):
		self.initial_gs()
		self.vertex_ijk()
		self.patches()
		if single_patch:
			if patch_num == 1:
				patch_state_length = len(self.patch_list[patch_num - 1]) - 2
				for i in range(1, patch_state_length + 1):
					self.plot_vf_one_triangle(i, patch_num)
			else:
				patch_length = len(self.patch_list[patch_num - 1]) - 2
				patch_length_pre = 0
				for i in range(1, patch_num):
					patch_length_pre += len(self.patch_list[i - 1]) - 2
				for i in range(patch_length_pre, min([patch_length_pre + patch_length, self.state_total_number + 1])):
					self.plot_vf_one_triangle(i, patch_num)
		else:
			for patch_num in range(1, len(self.patch_list) + 1):
				f = plt.figure(patch_num, figsize = (7.5, 9), dpi = 100)
				self.plot_triangles()
				if patch_num == 1:
					patch_state_length = len(self.patch_list[patch_num - 1]) - 2
					for i in range(1, patch_state_length + 1):
						self.plot_vf_one_triangle(i, patch_num)
				else:
					patch_length = len(self.patch_list[patch_num - 1]) - 2
					patch_length_pre = 0
					for i in range(1, patch_num):
						patch_length_pre += len(self.patch_list[i - 1]) - 2
					for i in range(patch_length_pre, min([patch_length_pre + patch_length, self.state_total_number + 1])):
						self.plot_vf_one_triangle(i, patch_num)
				plt.show()


class Robot(State):
	def __init__(self, geometry_name, agent_radius, agent_id, v_max):
		super(Robot, self).__init__(geometry_name)
		super(Robot, self).initial_gs()
		super(Robot, self).vertex_ijk()
		super(Robot, self).patches()
		self.v_max = v_max
		self.agent_id = agent_id
		self.agent_radius = agent_radius
		self.current_state = 1
		self.current_triangle = int(self.triangle_number(self.current_state))
		self.current_patch = 1
		self.is_finished = False
		v1 = self.position(self.v1v2v3(self.current_state)[0])
		v2 = self.position(self.v1v2v3(self.current_state)[1])
		v3 = self.position(self.v1v2v3(self.current_state)[2])
		x_init = v1[0] + v2[0] + v3[0]
		y_init = v1[1] + v2[1] + v3[1]
		x_init /= 3.
		y_init /= 3.
		self.initial_postion = np.array([x_init, y_init])
		# print 'initial position', self.initial_postion
		self.current_position = self.initial_postion
		self.curent_velocity = np.array(self.vector_field_point(self.current_position, self.current_patch, self.current_state, self.v_max)[0])
		# print 'current velocity', self.curent_velocity
		self.visted_goals = []
		self.not_created = True
	
	def ispoint_in_tri(self, point, state_num):
		v1v2v3_poses = []
		self.is_new_state = False
		
		for i in range(0, 3):
			v1v2v3_poses.append(np.array(self.position(self.v1v2v3(state_number = state_num)[i])))
		[v1, v2, v3] = v1v2v3_poses
		p = np.array(point)
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
			self.is_new_state = False
		else:
			self.is_new_state = True
	
	# def point_in_tri_check(self, point, tri):
	# 	[v1, v2, v3] = tri
	# 	p = np.array(point)
	# 	vector0 = v3 - v1
	# 	vector1 = v2 - v1
	# 	vector2 = p - v1
	# 	dot00 = np.dot(vector0, vector0)
	# 	dot01 = np.dot(vector0, vector1)
	# 	dot02 = np.dot(vector0, vector2)
	# 	dot11 = np.dot(vector1, vector1)
	# 	dot12 = np.dot(vector1, vector2)
	# 	invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
	# 	u = (dot11 * dot02 - dot01 * dot12) * invDenom
	# 	v = (dot00 * dot12 - dot01 * dot02) * invDenom
	# 	return (u >= 0) and (v >= 0) and (u + v <= 1)
	
	def move(self, current_pos, current_vel, current_state = 1, current_patch = 1, dt = 0.02):
		print '**'
		print 'state number', self.current_state, 'agent_id', self.agent_id
		print current_pos, current_vel, dt
		self.next_posotion = current_pos + current_vel * dt  # find the next point
		self.state_has_changed = False
		self.state_has_changed = self.ispoint_in_tri(self.next_posotion, current_state)  # check if the next point is out of the current state, equivalently, the robot has moved to next
		# state
		print 'state change?', self.state_has_changed
		patch_len = len(self.patch_list[current_patch - 1])
		last_patches_total_length = 0
		if current_patch > 1:
			for i in range(1, current_patch):
				last_patches_total_length += len(self.patch_list[i - 1]) - 2
			# print 'last patch total length', last_patches_total_length
		
		local_state = current_state + current_patch - 1 - last_patches_total_length
		# print 'local state', local_state, 'len(self.patch_list[current_patch - 1])', len(self.patch_list[current_patch - 1]) - 2
		is_last_state_in_patch = False
		
		if len(self.patch_list[current_patch - 1]) - 2 == local_state:
			is_last_state_in_patch = True
		
		if is_last_state_in_patch == True:
			if np.sqrt(current_vel[0] ** 2 + current_vel[1] ** 2) < 0.1:
				self.curent_velocity = np.array([0., 0.])
				if self.current_patch == len(self.patch_list):
					self.is_finished = True
				self.current_patch = current_patch + 1
			else:
				self.curent_velocity = np.array(self.vector_field_point(self.next_posotion, current_patch, self.current_state, self.v_max)[0])
		else:
			self.curent_velocity = np.array(self.vector_field_point(self.next_posotion, current_patch, self.current_state, self.v_max)[0])
		
		if self.state_has_changed == True:
			print 'transition'
			self.current_state = current_state + 1
		
		self.current_position = self.next_posotion
		return self.current_position, self.curent_velocity, self.current_state, self.current_patch, self.is_finished
	
	def collision_check(self, p1, p2, v1, v2, agent_id, teammate_id):  # Velocity Obstacle method
		g_set_1 = []
		radius_total = self.agent_radius * 3.
		distance_u2u = np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
		# print 'distance between the UAVs:::::', distance_u2u
		theta_half = math.asin(radius_total / distance_u2u)
		theta_forward = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
		theta_right = theta_forward - theta_half
		theta_left = theta_forward + theta_half
		p_ab = v2
		v_ref = self.curent_velocity
		p_vo_right = (p_ab + np.array([math.cos(theta_right), math.sin(theta_right)]) * 2. * self.v_max)
		p_vo_left = (p_ab + np.array([math.cos(theta_left), math.sin(theta_left)]) * 2. * self.v_max)
		# y = ax + b right line , a_r, b_r , left line a_l, b_l
		print 'p_ab', p_ab, p_vo_right, p_vo_left, 'pvo right left '
		
		def is_point_in_tri(point, triangle):
			p = np.array(point)
			[v1, v2, v3] = triangle
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
			return (u >= 0) and (v >= 0) and (u + v <= 1)
		
		if p_vo_right[0] - p_ab[0] != 0.:
			a_r = (p_vo_right[1] - p_ab[1]) / (p_vo_right[0] - p_ab[0])
			b_r = p_vo_right[1] - a_r * p_vo_right[0]
		else:
			a_r = None
		
		if p_vo_left[0] - p_ab[0] != 0.:
			a_l = (p_vo_left[1] - p_ab[1]) / (p_vo_left[0] - p_ab[0])
			b_l = p_vo_left[1] - a_r * p_vo_left[0]
		else:
			a_l = None
		
		g_set_1 = uConstraints.point_g_calculator(point_xy = self.current_position, vertice1 = self.position(self.v1v2v3(self.current_state)[0]),
		                                          common_facet_points = [self.position(self.v1v2v3(self.current_state)[1]), self.position(self.v1v2v3(self.current_state)[2])],
		                                          vel_max = self.v_max)
		print 'gset1', g_set_1
		dist = []
		
		for i in range(0, len(g_set_1)):
			dist.append(abs(g_set_1[i][0]) + abs(g_set_1[i][1]))
		
		zero_point_index = dist.index(min(dist))
		g_set_ordered = []
		
		for i in range(0, len(g_set_1)):
			g_set_ordered.append(g_set_1[np.mod(zero_point_index + i, len(g_set_1))])
		print 'gset ordered', g_set_ordered
		length = 0.
		
		if is_point_in_tri(v1, [p_ab, p_vo_right, p_vo_left]):
			print 'collision detected!'
			for i in range(0, len(g_set_ordered)):
				if not is_point_in_tri(g_set_ordered[i], [p_ab, p_vo_right, p_vo_left]):
					if length < np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2):
						length = np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2)
						best_vertex_index = i
			v_optimal = g_set_ordered[best_vertex_index]
		else:
			v_optimal = v1
		
		#
		# # print 'g set ordered', g_set_ordered
		# intersection_list_rl = [[False for value in g_set_ordered], [False for value in g_set_ordered]]
		# # print intersection_list_rl
		#
		# for j in range(0, len(g_set_1)):
		# 	[x0,y0] = [g_set_ordered[np.mod(j, len(g_set_ordered))][0], g_set_ordered[np.mod(j, len(g_set_ordered))][1]]
		# 	[x1,y1] = [g_set_ordered[np.mod(j + 1, len(g_set_ordered))][0], g_set_ordered[np.mod(j + 1, len(g_set_ordered))][1]]
		# 	if a_r != None:
		# 		sign_0_r = np.sign(y0 - a_r * x0 - b_r)
		# 		sign_1_r = np.sign(y1 - a_r * x1 - b_r)
		# 	else:
		# 		sign_0_r = np.sign(x0 - p_vo_right[0])
		# 		sign_1_r = np.sign(x1 - p_vo_right[0])
		#
		# 	if a_l != None:
		# 		sign_0_l = np.sign(y0 - a_l * x0 - b_l)
		# 		sign_1_l = np.sign(y1 - a_l * x1 - b_l)
		# 	else:
		# 		sign_0_l = np.sign(x0 - p_vo_left[0])
		# 		sign_1_=l = np.sign(x1 - p_vo_left[0])
		#
		# 	if sign_0_r != sign_1_r:
		# 		intersection_list_rl[0][j] = True
		# 	if sign_0_l != sign_1_l:
		# 		intersection_list_rl[1][j] = True
		# # print 'intersection_list_rl', intersection_list_rl
		#
		# no_right = False
		# no_left = False
		#
		# try:
		# 	intersection_list_rl[0].index(True)
		# except:
		# 	no_right = True
		#
		# if not no_right:
		# 	intersection_list_rl[0][intersection_list_rl[0].index(True)] = False
		# 	right_intersect_index = intersection_list_rl[0].index(True)
		# 	p_r_out = g_set_ordered[right_intersect_index]
		# 	p_r_in = g_set_ordered[np.mod(right_intersect_index + 1, len(g_set_ordered))]
		#
		# 	if p_r_out[0] - p_r_in[0] != 0.:
		# 		a_g_r = (p_r_out[1] - p_r_in[1]) / (p_r_out[0] - p_r_in[0])
		# 		b_g_r = p_r_out[1] - a_g_r * p_r_out[0]
		# 	else:
		# 		a_g_r = None
		# 		g_vo_int_right = [p_r_in[0], a_r * p_r_in[0] + b_r]
		# 	if a_g_r != None:
		# 		g_vo_int_right = [(b_g_r - b_r) / (a_r - a_g_r), a_r * (b_g_r - b_r) / (a_r - a_g_r) + b_r]
		#
		# try:
		# 	intersection_list_rl[1].index(True)
		# except:
		# 	no_left = True
		#
		# if not no_left:
		# 	intersection_list_rl[1][intersection_list_rl[1].index(True)] = False
		# 	left_intersect_index = intersection_list_rl[1].index(True)
		# 	p_l_in = g_set_ordered[left_intersect_index]
		# 	p_l_out = g_set_ordered[np.mod(left_intersect_index + 1, len(g_set_ordered))]
		#
		# 	if p_l_out[0] - p_l_in[0] != 0.:
		# 		a_g_l = (p_l_out[1] - p_l_in[1]) / (p_l_out[0] - p_l_in[0])
		# 		b_g_l = p_l_out[1] - a_g_l * p_l_out[0]
		# 	else:
		# 		a_g_l = None
		# 		g_vo_int_left = [p_l_in[0], a_l * p_l_in[0] + b_l]
		# 	if a_g_l != None:
		# 		g_vo_int_left = [(b_g_l - b_l) / (a_l - a_g_l), a_l * (b_g_l - b_l) / (a_l - a_g_l) + b_l]
		# # print 'gvo= right left, gvo =', g_vo_int_right, g_vo_int_right
		# # print 'v1=', v1, 'p vo right', p_vo_right, 'p vo left ', p_vo_left
		# is_between = is_point_in_tri(v1, [p_ab, p_vo_right, p_vo_left])
		# v_optimal = v1
		# if is_between:
		# 	print 'collision effort'
		# 	if not no_left:
		# 		v_optimal = np.array(g_vo_int_left)
		# 		if not no_right:
		# 			if np.sqrt(g_vo_int_left[0]**2+g_vo_int_left[1]**2) - np.sqrt(g_vo_int_right[0]**2+g_vo_int_right[1]**2) < 0:
		# 				v_optimal = np.array(g_vo_int_right)
		# 	elif not no_right:
		# 		v_optimal = np.array(g_vo_int_right)
		# 	else:
		# 		# print self.current_position, self.current_state, self.curent_velocity, g_set_ordered, intersection_list_rl
		# 		print "No collision-free path available!"
		#
		# else:
		# 	v_optimal = v1
		
		polygon_gs = []
		
		for i, g_vertex in enumerate(g_set_ordered):
			x_pos = round((g_vertex[0] - world_xmin + p1[0]) * world_scale)
			y_pos = round((g_vertex[1] - world_ymin + p1[1]) * world_scale)
			polygon_gs.append(x_pos)
			polygon_gs.append(y_pos)
		
		polygon_vo = tuple([round((p1[0] + p_ab[0] - world_xmin) * world_scale), round((p1[1] + p_ab[1] - world_ymin) * world_scale), round((p1[0] + p_vo_right[0] - world_xmin) *
		                                                                                                                                    world_scale), \
		                    round((p1[1] + p_vo_right[1] - world_ymin) * world_scale), round((p1[0] + p_vo_left[0] - world_xmin) * world_scale),
		                    round((p1[1] + p_vo_left[1] - world_ymin) *
		                          world_scale)])
		
		x1_v = (self.current_position[0] - world_xmin) * world_scale
		y1_v = (self.current_position[1] - world_ymin) * world_scale
		x2_v = (self.current_position[0] - world_xmin + v_optimal[0]) * world_scale
		y2_v = (self.current_position[1] - world_ymin + v_optimal[1]) * world_scale
		x1_v_ref = (self.current_position[0] - world_xmin) * world_scale
		y1_v_ref = (self.current_position[1] - world_ymin) * world_scale
		x2_v_ref = (self.current_position[0] - world_xmin + v1[0]) * world_scale
		y2_v_ref = (self.current_position[1] - world_ymin + v1[1]) * world_scale
		velocity_arrow_coords = tuple([x1_v, y1_v, x2_v, y2_v])
		velocity_ref_coords = tuple([x1_v_ref, y1_v_ref, x2_v_ref, y2_v_ref])
		
		if self.not_created:
			self.poly = canvas.create_polygon(tuple(polygon_gs), fill = '', outline = 'black', )
			self.tri_vo = canvas.create_polygon(polygon_vo, fill = '', outline = 'blue')
			self.velocity_arrow_image = canvas.create_line(velocity_arrow_coords, fill = 'yellow')
			self.velocity_ref_image = canvas.create_line(velocity_ref_coords, fill = 'white')
			self.not_created = False
		
		canvas.coords(self.velocity_arrow_image, velocity_arrow_coords)
		canvas.coords(self.velocity_ref_image, velocity_ref_coords)
		canvas.coords(self.poly, tuple(polygon_gs))
		canvas.coords(self.tri_vo, polygon_vo)
		
		return v_optimal
# =====================================================================================================================================================================================
# Simulation parameteres
# =====================================================================================================================================================================================
robot1 = Robot(geometry_name = 'robot1', agent_radius = .2, agent_id = 1, v_max = 1.)
robot2 = Robot(geometry_name = 'robot2', agent_radius = .2, agent_id = 2, v_max = 1.)
trajectory = []
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
# =====================================================================================================================================================================================
# initialize the agents
# =====================================================================================================================================================================================
def initWorld(canvas):
	global traj_point
	print ("")
	print ("Simulation of multi-robot in triangular environment.")
	print ("")
	for i, node in enumerate(robot1.triangle_nodes):
		# print 'node', node
		triangle = [robot1.position(node[0]), robot1.position(node[1]), robot1.position(node[2])]
		# print 'triangle', triangle
		Grid.append(canvas.create_polygon(triangle[0][0], triangle[0][1], triangle[1][0], triangle[1][1],
		                                  triangle[1][0], triangle[1][1], triangle[2][0], triangle[2][1],
		                                  triangle[2][0], triangle[2][1], triangle[0][0], triangle[0][1], outline = "black", fill = "gray"))
	# traj_point = canvas.create_rectan(0.,0.,2.,2.)
	colors = ["white", "blue", "yellow", "#FAA"]
# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():
	for i, node in enumerate(robot1.triangle_nodes):
		triangle = [robot1.position(node[0]), robot1.position(node[1]), robot1.position(node[2])]
		# print 'triangle', triangle
		canvas.coords(Grid[i], world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin),
		              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
		              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
		              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
		              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
		              world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
	# print circle1, world_scale * (robot1.pos[0] - world_xmin), world_scale * (robot1.pos[1] - world_ymin)
	canvas.coords(uav_1_image, world_scale * (robot1.current_position[0] - world_xmin), world_scale * (robot1.current_position[1] - world_ymin))
	canvas.coords(uav_2_image, world_scale * (robot2.current_position[0] - world_xmin), world_scale * (robot2.current_position[1] - world_ymin))
	canvas.create_image(world_scale * (robot1.current_position[0] - world_xmin), world_scale * (robot1.current_position[1] - world_ymin), image = trajectory_image)
	canvas.create_image(world_scale * (robot2.current_position[0] - world_xmin), world_scale * (robot2.current_position[1] - world_ymin), image = trajectory_image_2)
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
	if not robot1.is_finished:
		robot1.current_position, robot1.curent_velocity, robot1.current_state, robot1.current_patch, robot1.is_finished = robot1.move(robot1.current_position, robot1.curent_velocity,
		                                                                                                                              robot1.current_state, robot1.current_patch,
		                                                                                                                              dt = 0.02)
		if robot1.state_has_changed == True:
			robot1.current_state = robot1.current_state - 1
		
		r1_last_velocity = robot1.curent_velocity
		current_distance = np.sqrt((robot1.current_position[0] - robot2.current_position[0]) ** 2 + (robot1.current_position[1] - robot2.current_position[1]) ** 2)
		
		if current_distance < 2.0:
			robot1.curent_velocity = robot1.collision_check(robot1.current_position, robot2.current_position, robot1.curent_velocity, robot2.curent_velocity, robot1.agent_id,
			                                                robot2.agent_id)
		if robot1.state_has_changed == True:
			robot1.current_state = robot1.current_state + 1
	
	if not robot2.is_finished:
		robot2.current_position, robot2.curent_velocity, robot2.current_state, robot2.current_patch, robot2.is_finished = robot2.move(robot2.current_position, robot2.curent_velocity,
		                                                                                                                              robot2.current_state, robot2.current_patch,
		                                                                                                                              dt = 0.02)
		
		if robot2.state_has_changed == True:
			robot2.current_state = robot2.current_state - 1
		
		if current_distance < 2.0:
			robot2.curent_velocity = robot2.collision_check(robot2.current_position, robot1.current_position, robot2.curent_velocity, r1_last_velocity, robot2.agent_id,
			                                                robot1.agent_id)
		
		if robot2.state_has_changed == True:
			robot2.current_state = robot2.current_state + 1
# =====================================================================================================================================================================================
# read geometry
# =====================================================================================================================================================================================
def readScenario(geometry, scalex = 1., scaley = 1.):
	x_min = 0 * scalex - 2.
	y_min = 0 * scaley - 1.
	x_max = 10 * scalex + 2.
	y_max = 10 * scaley + 2.
	return x_min, x_max, y_min, y_max
# =====================================================================================================================================================================================
# simulate and draw frames
# =====================================================================================================================================================================================
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
geometry = robot1.geometry_name
world_xmin, world_xmax, world_ymin, world_ymax = readScenario(geometry)
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
quad = PhotoImage(file = 'quad1.gif')
quad = quad.subsample(35, 35)
trajectory_image = PhotoImage(file = 'trajectory_im.gif')
trajectory_image = trajectory_image.subsample(800, 800)
trajectory_image_2 = PhotoImage(file = 'trajectory_im_2.gif')
trajectory_image_2 = trajectory_image_2.subsample(800, 800)
uav_1_image = canvas.create_image(200, 100, image = quad)
uav_2_image = canvas.create_image(200, 100, image = quad)
mainloop()

