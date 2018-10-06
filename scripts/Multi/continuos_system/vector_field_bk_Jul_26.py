import numpy as np
import math
from interval import interval
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import copy
import time
from Tkinter import *
from operator import add
import JarvisMarch
# import discrete_path_plannar
import os
import subprocess
import os.path
from PIL import Image, ImageDraw

r1_geometry = 'robot1'
r2_geometry = 'robot2'
r1_pos_init = [1.1, 0.3]
r2_pos_init = [1.1, 6.9]
r1_goals = 'goals_robot1'
r2_goals = 'goals_robot2'

parent_dir = os.path.dirname(__file__) + '/../..'
parent_directory = os.path.abspath(os.path.join(os.getcwd(), os.pardir))


def discrete_planner(geometry_name, init_position, goals_file = 'goals', goal_list = [], temp_obstacle = 0):
	os.system('triangle -Bnevgu {}.poly'.format(parent_directory + '/geometry/' + geometry_name))  # -n is for producing the neighbor
	
	with open(parent_directory + '/geometry/{}.1.node'.format(geometry_name)) as vpos:
		vertices_pos = vpos.readlines()
	vpos.close()
	del (vertices_pos[-1])
	vertices_coords_temp = []
	
	for i in xrange(1, len(vertices_pos)):
		vertice = vertices_pos[i].strip().split(' ')
		vertice = filter(None, vertice)
		vertice = map(float, vertice[1:3])
		vertices_coords_temp.append(vertice)
		
		with open(parent_directory + '/geometry/{}.1.ele'.format(geometry_name)) as elements:
			triangle_vertice_list = elements.readlines()
		
		elements.close()
		
		del (triangle_vertice_list[-1])
		
		tri_nodes_list = []
		# Triangles numbers and Vertices numbers
		for i in xrange(1, len(triangle_vertice_list)):
			triangle_nodes = triangle_vertice_list[i].strip().split(' ')
			triangle_nodes = filter(None, triangle_nodes)
			triangle_nodes = map(int, triangle_nodes[1:])
			tri_nodes_list.append(triangle_nodes)
		
		number_of_triangles = len(tri_nodes_list)
		
		with open(parent_directory + '/geometry/{}.1.neigh'.format(geometry_name)) as neighbors_text:
			neighbors = neighbors_text.readlines()
		neighbors_text.close()
		
		del (neighbors[-1])
		
		neighbors_list_temp = []
		
		for i in xrange(1, len(neighbors)):
			neighbor_num = neighbors[i].strip().split(' ')
			neighbor_num = filter(None, neighbor_num)
			neighbors_list_temp.append(neighbor_num)
	
	for tri_num, tri_vrtx_num in enumerate(tri_nodes_list):
		p = init_position  # np.array([x,y])
		v = []
		for i in range(0, 3):
			v.append(np.array(vertices_coords_temp[tri_vrtx_num[i] - 1]))
		triangle_number = tri_num + 1
		vector0 = v[2] - v[0]
		vector1 = v[1] - v[0]
		vector2 = p - v[0]
		dot00 = np.dot(vector0, vector0)
		dot01 = np.dot(vector0, vector1)
		dot02 = np.dot(vector0, vector2)
		dot11 = np.dot(vector1, vector1)
		dot12 = np.dot(vector1, vector2)
		invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
		u = (dot11 * dot02 - dot01 * dot12) * invDenom
		v = (dot00 * dot12 - dot01 * dot02) * invDenom
		if (u >= 0) and (v >= 0) and (u + v <= 1):
			init_triangle = triangle_number
			break
	
	if goal_list == []:
		with open(parent_directory + '/discrete_transition/{}.txt'.format(goals_file), 'r') as g:
			goals = g.readlines()
		g.close()
		goals = [int(value.strip()) for value in goals]
	else:
		goals = goal_list
	
	if os.path.isfile('output_file.smv'):  # remove previous output file
		os.remove('output_file.smv')
	
	with open(parent_directory + '/geometry/{}.1.neigh'.format(geometry_name)) as f:  # opens input file (C.1.neigh)
		lines = f.readlines()
	f.close()
	
	del lines[-1]
	state_number = len(lines)
	text_file = open(parent_directory + '/discrete_transition/output_file.smv', 'w')
	text_file.write('MODULE main\nVAR\nx : grid;\n')
	text_file.write('LTLSPEC !( (')
	if temp_obstacle == 0:
		text_file.write(' &'.join(' F (x.state = ' + str(gg) + ')' for gg in goals) + ' ) )')
	else:
		text_file.write(' &'.join(' F (x.state = ' + str(gg) + ')' for gg in goals) + ' & !G (x.state = {} )'.format(temp_obstacle) + ' ) )')
	text_file.write('\nMODULE grid\nVAR\nstate : 1..')
	text_file.write(state_number.__str__())
	text_file.write(';\nASSIGN\ninit(state) := {};\nnext(state) := \ncase\n'.format(init_triangle))
	for i in xrange(1, state_number):
		pp = lines[i].strip().split(' ')
		pp = filter(None, pp)
		pp = pp[1:]
		
		try:
			pp.remove('-1')
		except ValueError:
			None
		
		try:
			pp.remove('-1')
		except ValueError:
			None
		
		text_file.write('state = %d : {' % i + ', '.join(p for p in pp) + '};\n')
	
	text_file.write('TRUE : state ;\nesac;')
	text_file.close()
	###### Runs NuSMV and outputs the file state_order ######
	output = subprocess.check_output('NuSMV ' + parent_directory + '/discrete_transition/output_file.smv', shell = True)  # run NuSMV
	text_file = open(parent_directory + '/discrete_transition/smv_output.txt', 'w')
	text_file.write(output)
	text_file.close()
	
	with open(parent_directory + '/discrete_transition/smv_output.txt', 'r') as path:
		paths = path.readlines()
		del paths[0:19]
	
	state = []
	not_added_goals = goals
	
	for lines in paths:
		if 'x.state' in lines and len(not_added_goals) != 0:
			state.append(lines[11:].strip())
			if int(state[-1]) in not_added_goals:
				not_added_goals.remove(int(state[-1]))
			# print(len(not_added_goals))
	
	text_file2 = open(parent_directory + '/discrete_transition/state_order_{}.txt'.format(geometry_name), 'w')
	text_file2.write('\n'.join(state))
	text_file2.close()


discrete_planner(r1_geometry, r1_pos_init, r1_goals)
discrete_planner(r2_geometry, r2_pos_init, r2_goals)


class Vertice(object):
	def __init__(self, geometry_name):
		with open(parent_directory + '/geometry/{}.1.node'.format(geometry_name)) as vpos:
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
		with open(parent_directory + '/geometry/{}.1.ele'.format(geometry_name)) as elements:
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
		
		with open(parent_directory + '/geometry/{}.1.neigh'.format(geometry_name)) as neighbors_text:
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
		with open(parent_directory + '/discrete_transition/state_order_{}.txt'.format(geometry_name)) as state:
			self.state_list = state.readlines()
		state.close()
		self.state_total_number = len(self.state_list)
		# if self.sta
		self.last_state = self.state_list[-1]
		self.gs = []
		self.global_vertex_index = []
		self.vector_field = dict()
		self.last_valid_patch = []
	
	def triangle_number(self, state_number):
		return self.state_list[state_number - 1]
	
	def vertices(self, state_num):
		return super(State, self).vertices(int(self.triangle_number(state_num)))
	
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
		# #print self.gs, 'gs'
	
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
		# #print self.global_vertex_index, 'global vertex index'
	
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
					# #print 'patch number {} is ready!'.format(len(self.patch_list)), self.patch_list[-1]
					self.vector_field = dict()
					state_iter = state_iter - 1
					break
				last_valid_patch = copy.deepcopy(self.vector_field)
			if state_iter == self.state_total_number:
				self.patch_list.append(self.vector_field)
			state_iter = state_iter + 1
		
		# #print 'final patch list', self.patch_list
		# #print 'number of patches is {}'.format(len(self.patch_list))
	
	def vector_field_point(self, point, patch_num, state_num, v_max):
		last = False
		last_patches_total_length = 0
		for i in range(0, patch_num):
			last_patches_total_length += len(self.patch_list[i]) - 2
		local_state = state_num + patch_num - 1 - last_patches_total_length
		g1_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[0]].midpoint
		g2_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[1]].midpoint
		g3_angle = self.patch_list[patch_num - 1][self.v1v2v3(state_num)[2]].midpoint
		# print 'g1 g2 g3=', g1, g2, g3
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
			# #print 'default gs', default_gs
			
			if len(default_gs) == 2:
				g_6points.append([[np.cos(math.radians(default_gs[0][1])), np.sin(math.radians(default_gs[0][1]))], [np.cos(math.radians(default_gs[1][0])), np.sin(math.radians(
					default_gs[1][0]))]])
			else:
				g_6points.append([[np.cos(math.radians(default_gs[0][0])), np.sin(math.radians(default_gs[0][0]))], [np.cos(math.radians(default_gs[0][1])), np.sin(math.radians(
					default_gs[0][1]))]])
		
		# #print 'g6points', g_6points
		v = [self.position(self.v1v2v3(self.current_state)[0]), self.position(self.v1v2v3(self.current_state)[1]), self.position(self.v1v2v3(self.current_state)[2])]
		W = [[v[0][0], v[1][0], v[2][0]], [v[0][1], v[1][1], v[2][1]], [1.0, 1.0, 1.0]]
		inv_W = np.matrix(W) ** -1
		x, y = point_xy[0], point_xy[1]
		lambdas = np.dot(v_max, np.array(inv_W * [[x], [y], [1]]))  # .transpose()
		# lambdas = np.array(inv_W * [[x], [y], [1]])
		# #print 'lambda', lambdas
		g_total_333 = [[[0., 0.], g_6points[0][0], g_6points[0][1]], [[0., 0.], g_6points[1][0], g_6points[1][1]], [[0., 0.], g_6points[2][0], g_6points[2][1]]]
		# #print 'g total 333', g_total_333
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
				# #print 'l1', l1, 'l2', l2, 'l3', l3
				[x, y] = l1 * np.array(v1) + l2 * np.array(v2) + l3 * np.array(v3)
				# plt.plot(x,y, '*')
				v = np.array(GW * [[x], [y], [1]]).transpose()[0]
				# #print 'vx vy x y', v[0], v[1], x, y
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
	def __init__(self, geometry_name, agent_radius, agent_id, v_max, goals_file, init_pos = [0., 0.]):
		super(Robot, self).__init__(geometry_name)
		super(Robot, self).initial_gs()
		super(Robot, self).vertex_ijk()
		super(Robot, self).patches()
		self.temp_obs = 0
		self.current_state = 1
		v1 = self.position(self.v1v2v3(self.current_state)[0])
		v2 = self.position(self.v1v2v3(self.current_state)[1])
		v3 = self.position(self.v1v2v3(self.current_state)[2])
		
		if init_pos == [0., 0.]:
			x_init = v1[0] + v2[0] + v3[0]
			y_init = v1[1] + v2[1] + v3[1]
			x_init /= 3.
			y_init /= 3.
			self.initial_postion = np.array([x_init, y_init])
		else:
			self.initial_postion = np.array(init_pos)
		
		self.waypoint = [0., 0.]
		self.auto = True
		self.reinitial = False
		self.v_max = v_max
		self.agent_id = agent_id
		self.agent_radius = agent_radius
		self.current_triangle = int(self.triangle_number(self.current_state))
		self.current_patch = 1
		self.is_finished = False
		self.current_position = self.initial_postion
		self.curent_velocity = np.array(self.vector_field_point(self.current_position, self.current_patch, self.current_state, self.v_max)[0])
		goal_temp = []
		self.goal_list = []
		self.goals_file = goals_file
		self.poly_pix = []
		with open(parent_directory + '/discrete_transition/{}.txt'.format(goals_file)) as elements:
			goal_temp = elements.readlines()
		elements.close()
		for i, value in enumerate(goal_temp):
			self.goal_list.append(int(value.strip()))
		self.not_visted_goals = self.goal_list
		# print 'goal list ', self.goal_list, self.agent_id
		self.visited_goals = []
		self.not_created = True
	
	def ispoint_in_tri(self, point, state_num):
		v1v2v3_poses = []
		self.is_new_state = False
		
		for i in range(0, 3):
			v1v2v3_poses.append(np.array(self.position(self.v1v2v3(state_number = state_num)[i])))
		
		p = np.array(point)
		[v1, v2, v3] = v1v2v3_poses
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
		# #print '*********************************************'
		# #print 'transition'
		return self.is_new_state
	
	def move(self, current_pos, current_vel, current_state = 1, current_patch = 1, dt = 0.05):
		if self.reinitial == True:
			not_visited_goals_file = open(parent_directory + '/discrete_transition/{}_newgoals.txt'.format(self.geometry_name), 'w')
			for item in self.not_visted_goals:
				not_visited_goals_file.write("%s\n" % item)
			not_visited_goals_file.close()
			discrete_planner(self.geometry_name, self.current_position, goals_file = '{}_newgoals'.format(self.geometry_name), temp_obstacle = self.temp_obs)
			self.current_patch = 1
			self.current_state = 1
			self.__init__(self.geometry_name, self.agent_radius, self.agent_id, self.v_max, '{}_newgoals'.format(self.geometry_name), init_pos = self.current_position)
			current_vel = self.curent_velocity
			current_pos = self.current_position
			current_patch = self.current_patch
			current_state = self.current_state
			# #print 'current state', current_state
			self.reinitial = False
			initWorld(canvas)
		
		if int(self.triangle_number(self.current_state)) in self.goal_list:
			self.not_visted_goals.remove(int(self.triangle_number(self.current_state)))
			poly_xy = self.vertices_poses(self.current_state)
			self.poly_pix.append(((poly_xy[0][0] - world_xmin) * world_scale, (poly_xy[0][1] - world_ymin) * world_scale, (poly_xy[1][0] - world_xmin) * world_scale,
			                      (poly_xy[1][1] - world_ymin) * world_scale,
			                      (poly_xy[2][0] - world_xmin) * world_scale, (poly_xy[2][1] - world_ymin) * world_scale))
			if self.agent_id == 1:
				visited_targets = canvas.create_polygon(self.poly_pix[-1], fill = '#ffffcc', outline = 'black')
			if self.agent_id == 2:
				visited_targets = canvas.create_polygon(self.poly_pix[-1], fill = '#66ff66', outline = 'black')
			
			self.visited_goals.append(self.current_state)
			# print 'not visitied and visited and total', self.not_visted_goals, self.visited_goals, self.goal_list, self.agent_id
			if len(self.not_visted_goals) == 0:
				self.is_finished = True
			# #print 'not visited goals', self.not_visted_goals
		
		self.next_posotion = [current_pos[0] + current_vel[0] * dt, current_pos[1] + current_vel[1] * dt]
		self.state_has_changed = False
		self.state_has_changed = self.ispoint_in_tri(self.next_posotion, current_state)  # check if the next point is out of the current state, equivalently, the robot has moved to next
		last_patches_total_length = 0
		if current_patch > 1:
			for i in range(1, current_patch):
				last_patches_total_length += len(self.patch_list[i - 1]) - 2
			# #print 'last patch total length', last_patches_total_length
		
		local_state = current_state + current_patch - 1 - last_patches_total_length
		# #print 'local state', local_state, 'len(self.patch_list[current_patch - 1])', len(self.patch_list[current_patch - 1]) - 2
		is_last_state_in_patch = False
		
		if len(self.patch_list[current_patch - 1]) - 2 == local_state:
			is_last_state_in_patch = True
		
		if is_last_state_in_patch == True:
			if np.sqrt(current_vel[0] ** 2 + current_vel[1] ** 2) < 0.1:
				self.curent_velocity = np.array([0., 0.])
				if self.current_patch == len(self.patch_list):
					self.is_finished = True
				else:
					self.current_patch = current_patch + 1
			else:
				self.curent_velocity = np.array(self.vector_field_point(self.next_posotion, self.current_patch, self.current_state, self.v_max)[0])
		else:
			self.curent_velocity = np.array(self.vector_field_point(self.next_posotion, self.current_patch, self.current_state, self.v_max)[0])
		
		if self.state_has_changed == True:
			# #print 'transition'
			self.current_state = current_state + 1
		
		self.current_position = self.next_posotion
		return self.current_position, self.curent_velocity, self.current_state, self.current_patch, self.is_finished
	
	def collision_check(self, p1, p2, v1, v2, agent_id, teammate_id):  # Velocity Obstacle method
		g_set_1 = []
		radius_total = self.agent_radius * 3.
		distance_u2u = np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
		# #print 'distance between the UAVs:::::', distance_u2u
		theta_half = math.asin(radius_total / max(distance_u2u, radius_total))
		theta_forward = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
		theta_right = theta_forward - theta_half
		theta_left = theta_forward + theta_half
		p_ab = v2
		v_ref = self.curent_velocity
		p_vo_right = (p_ab + np.array([math.cos(theta_right), math.sin(theta_right)]) * 4. * self.v_max + v1) / 2.
		p_vo_left = (p_ab + np.array([math.cos(theta_left), math.sin(theta_left)]) * 4. * self.v_max + v1) / 2.
		
		# y = ax + b right line , a_r, b_r , left line a_l, b_l
		# #print 'p_ab', p_ab, p_vo_right, p_vo_left, 'pvo right left '
		
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
		# #print 'p1, patch state', self.agent_id, self.current_position, self.current_patch, self.current_state
		g_set_1 = self.gConstraints(p1, self.current_patch, self.current_state, self.v_max)
		# #print 'gset1', g_set_1
		dist = []
		
		for i in range(0, len(g_set_1)):
			dist.append(abs(g_set_1[i][0]) + abs(g_set_1[i][1]))
		
		zero_point_index = dist.index(min(dist))
		g_set_ordered = []
		
		for i in range(0, len(g_set_1)):
			g_set_ordered.append(g_set_1[np.mod(zero_point_index + i, len(g_set_1))])
		# #print 'gset ordered', g_set_ordered
		length = 0.
		
		if is_point_in_tri(v1, [p_ab, p_vo_right, p_vo_left]):
			# print 'collision detected!'
			for i in range(0, len(g_set_ordered)):
				if not is_point_in_tri(g_set_ordered[i], [p_ab, p_vo_right, p_vo_left]):
					if length < np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2):
						length = np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2)
						best_vertex_index = i
			try:
				g_set_ordered[best_vertex_index]
			except:
				# print 'no solution!, replan'
				best_vertex_index = 0
			v_optimal = g_set_ordered[best_vertex_index]
		else:
			v_optimal = v1
		
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
			self.i1 = self.poly = canvas.create_polygon(tuple(polygon_gs), fill = '', outline = 'black', )
			self.i2 = self.tri_vo = canvas.create_polygon(polygon_vo, fill = '', outline = 'blue')
			self.i3 = self.velocity_arrow_image = canvas.create_line(velocity_arrow_coords, fill = 'yellow')
			self.i4 = self.velocity_ref_image = canvas.create_line(velocity_ref_coords, fill = 'red')
			self.not_created = False
		
		canvas.coords(self.velocity_arrow_image, velocity_arrow_coords)
		canvas.coords(self.velocity_ref_image, velocity_ref_coords)
		canvas.coords(self.poly, tuple(polygon_gs))
		canvas.coords(self.tri_vo, polygon_vo)
		
		return v_optimal
	
	def manual_drive(self, dt):
		if self.waypoint != [0., 0.]:
			er_pos = [self.waypoint[0] - self.current_position[0], self.waypoint[1] - self.current_position[1]]
			# #print 'error pos', er_pos, 'waypoint', self.waypoint, self.current_position, 'current pos'
			er_pos_mag = np.sqrt(er_pos[0] ** 2 + er_pos[1] ** 2)
			
			vel_dir = [value / er_pos_mag for value in er_pos]
			vel_vector = [value * self.v_max * 4. for value in vel_dir]
			self.curent_velocity = vel_vector
			self.current_position = [self.current_position[0] + self.curent_velocity[0] * dt, self.current_position[1] + self.curent_velocity[1] * dt]
			if er_pos_mag < 0.1:
				self.waypoint = [0., 0.]
				self.reinitial = True


# =====================================================================================================================================================================================
# Simulation parameteres
# =====================================================================================================================================================================================
trajectory = []
pixelsize = 780
framedelay = 1
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
# Collision Avoidance
# =====================================================================================================================================================================================
def collision_avoidance(agent_1, agent_2):
	radius_total = (agent_1.agent_radius + agent_2.agent_radius) * 1.2
	
	def collision_free_velocity(r1, r2):
		is_collision = False
		p1 = r1.current_position
		p2 = r2.current_position
		distance_u2u = np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
		theta_half = math.asin(radius_total / max(distance_u2u, radius_total))
		theta_forward = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
		theta_right = theta_forward - theta_half
		theta_left = theta_forward + theta_half
		p_ab = r2.curent_velocity
		v_ref = r1.curent_velocity
		v1 = r1.curent_velocity
		if agent_2.auto == True:
			p_vo_right = (p_ab + np.array([math.cos(theta_right), math.sin(theta_right)]) * 4. * r1.v_max + v1) / 2.
			p_vo_left = (p_ab + np.array([math.cos(theta_left), math.sin(theta_left)]) * 4. * r1.v_max + v1) / 2.
		else:
			p_vo_right = (p_ab + np.array([math.cos(theta_right), math.sin(theta_right)]) * 4. * r1.v_max)
			p_vo_left = (p_ab + np.array([math.cos(theta_left), math.sin(theta_left)]) * 4. * r1.v_max)
		
		def is_point_in_tri(point, triangle):
			p = np.array(point)
			[vrx1, vrx2, vrx3] = triangle
			vector0 = vrx3 - vrx1
			vector1 = vrx2 - vrx1
			vector2 = p - vrx1
			dot00 = np.dot(vector0, vector0)
			dot01 = np.dot(vector0, vector1)
			dot02 = np.dot(vector0, vector2)
			dot11 = np.dot(vector1, vector1)
			dot12 = np.dot(vector1, vector2)
			if abs(dot00 * dot11 - dot01 * dot01) < 10. ** -16:
				invDenom = 10. ** 16
			else:
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
		g_set_1 = r1.gConstraints(p1, r1.current_patch, r1.current_state, r1.v_max)
		dist = []
		
		for i in range(0, len(g_set_1)):
			dist.append(abs(g_set_1[i][0]) + abs(g_set_1[i][1]))
		
		zero_point_index = dist.index(min(dist))
		g_set_ordered = []
		
		for i in range(0, len(g_set_1)):
			g_set_ordered.append(g_set_1[np.mod(zero_point_index + i, len(g_set_1))])
		
		max_r = 0.
		max_l = 0.
		
		v_sides = [False, False]
		# print '*', is_point_in_tri(v1, [p_ab, p_vo_right, p_vo_left]), v1, p_ab, p_vo_right, p_vo_left, r1.agent_id
		if is_point_in_tri(v1, [p_ab, p_vo_right, p_vo_left]):  # if the current velocity is in the RVO, collision will happen
			v_optimal = [[], []]
			# is_collision = False
			print 'collision detected'
			for i in range(1, len(g_set_ordered)):
				if not is_point_in_tri(g_set_ordered[i], [p_ab, p_vo_right, p_vo_left]):  # if vertex of the g_polyhedral is outside of the RVO trinagle it is collision-free
					vector_test = v_ref - g_set_ordered[i]
					[px, py] = [g_set_ordered[i][0], g_set_ordered[i][1]]  # px, py coordinates of the g vertex
					ccw_vortex_vector = np.array([-py / (px ** 2 + py ** 2), px / (px ** 2 + py ** 2)])
					if np.dot(vector_test, ccw_vortex_vector) >= 0:
						if max_r < np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2):
							max_r = np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2)
							v_optimal[0] = g_set_ordered[i]
						v_sides[0] = True
					else:
						if max_l < np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2):
							max_l = np.sqrt(g_set_ordered[i][0] ** 2 + g_set_ordered[i][1] ** 2)
							v_optimal[1] = g_set_ordered[i]
						v_sides[1] = True
			
			if v_optimal == [[], []]:
				print 'no solution'
				if r1.auto == True:
					r1.temp_obs = int(r2.triangle_number(r2.current_state))
					r1.reinitial = True
					v_optimal = [0., 0.]
		else:
			v_optimal = v1
		
		return v_optimal, v_sides, is_collision, [g_set_ordered, p_ab, p_vo_right, p_vo_left]
	
	def decide_velocity(v1_opt, v2_opt, v1_dir, v2_dir):
		if v1_dir == [False, False]:
			agent_1.current_velocity = [0., 0.]
			if v2_dir == [False, False]:
				print('no solution available')
				if agent_1.auto == True:
					agent_1.temp_obs = int(agent_2.triangle_number(agent_2.current_state))
					agent_1.reinitial = True
					agent_1.current_velocity = [0., 0.]
				if agent_2.auto == True:
					agent_2.temp_obs = int(agent_1.triangle_number(agent_1.current_state))
					agent_2.reinitial = True
					agent_2.current_velocity = [0., 0.]
			elif v2_dir[0] == True:
				agent_2.current_velocity = v2_opt[0]
			else:
				agent_2.current_velocity = v2_opt[1]
		elif v1_dir[0] == True:
			if v2_dir[0] == True:
				agent_1.curent_velocity = v1_opt[0]
				agent_2.curent_velocity = v2_opt[0]
			elif v2_dir[1] == False:
				agent_1.current_velocity = v1_dir[0]
		elif v1_dir[1] == True:
			if v2_dir[1] == True:
				agent_1.curent_velocity = v1_opt[1]
				agent_2.curent_velocity = v2_opt[1]
			elif v2_dir[0] == False:
				agent_1.current_velocity = v1_dir[1]
	
	v1_optimal, v1_sidechecks, is_colliding, plot_param1 = collision_free_velocity(agent_1, agent_2)  # v1_optimal = [v_r_optimal, v_l_optimal], if n/a v_r/l_optimal = [],
	
	if is_colliding and agent_1.reinitial == False:  # this is to avoid check collision for the agent_2 if no collision is detected
		v2_optimal, v2_sidechecks, is_colliding, plot_param2 = collision_free_velocity(agent_2, agent_1)
		old_v1 = agent_1.curent_velocity
		old_v2 = agent_2.curent_velocity
		decide_velocity(v1_optimal, v2_optimal, v1_sidechecks, v2_sidechecks)
		for i in range(2):
			if i == 0:
				r1 = agent_1
				r2 = agent_2
				v_optimal = old_v1
				p1 = r1.current_position
				p2 = r2.current_position
				v1 = r1.curent_velocity
				[g_set_ordered, p_ab, p_vo_right, p_vo_left] = plot_param1
			else:
				r1 = agent_2
				r2 = agent_1
				v_optimal = old_v2
				p1 = r2.current_position
				p2 = r1.current_position
				v1 = r2.curent_velocity
				[g_set_ordered, p_ab, p_vo_right, p_vo_left] = plot_param2
			polygon_gs = []
			
			for j, g_vertex in enumerate(g_set_ordered):  # building the polygon of the g-sets for plotting
				x_pos = round((g_vertex[0] - world_xmin + p1[0]) * world_scale)
				y_pos = round((g_vertex[1] - world_ymin + p1[1]) * world_scale)
				polygon_gs.append(x_pos)
				polygon_gs.append(y_pos)
			
			polygon_vo = tuple([round((p1[0] + p_ab[0] - world_xmin) * world_scale), round((p1[1] + p_ab[1] - world_ymin) * world_scale), round((p1[0] + p_vo_right[0] - world_xmin) *
			                                                                                                                                    world_scale), \
			                    round((p1[1] + p_vo_right[1] - world_ymin) * world_scale), round((p1[0] + p_vo_left[0] - world_xmin) * world_scale),
			                    round((p1[1] + p_vo_left[1] - world_ymin) *
			                          world_scale)])
			x1_v = round((r1.current_position[0] - world_xmin) * world_scale)
			y1_v = round((r1.current_position[1] - world_ymin) * world_scale)
			x2_v = round((r1.current_position[0] - world_xmin + v_optimal[0]) * world_scale)
			y2_v = round((r1.current_position[1] - world_ymin + v_optimal[1]) * world_scale)
			x1_v_ref = round((r1.current_position[0] - world_xmin) * world_scale)
			y1_v_ref = round((r1.current_position[1] - world_ymin) * world_scale)
			x2_v_ref = round((r1.current_position[0] - world_xmin + v1[0]) * world_scale)
			y2_v_ref = round((r1.current_position[1] - world_ymin + v1[1]) * world_scale)
			velocity_arrow_coords = tuple([x1_v, y1_v, x2_v, y2_v])
			velocity_ref_coords = tuple([x1_v_ref, y1_v_ref, x2_v_ref, y2_v_ref])
			
			if r1.not_created:
				r1.i1 = r1.poly = canvas.create_polygon(tuple(polygon_gs), fill = 'white', outline = 'black', )
				canvas.lift(r1.i1)
				r1.i2 = r1.tri_vo = canvas.create_polygon(polygon_vo, fill = '', outline = 'blue')
				r1.i3 = r1.velocity_arrow_image = canvas.create_line(velocity_arrow_coords, fill = 'yellow')
				r1.i4 = r1.velocity_ref_image = canvas.create_line(velocity_ref_coords, fill = 'red')
				r1.not_created = False
			
			canvas.coords(r1.velocity_arrow_image, velocity_arrow_coords)
			canvas.coords(r1.velocity_ref_image, velocity_ref_coords)
			canvas.coords(r1.poly, tuple(polygon_gs))
			canvas.coords(r1.tri_vo, polygon_vo)
	return is_colliding


# =====================================================================================================================================================================================
# initialize the agents
# =====================================================================================================================================================================================
def initWorld(canvas):
	global r1_goals_polygons, r2_goals_polygons, r1_poly_pix, r2_poly_pix, image1
	for i, node in enumerate(robot1.triangle_nodes):
		triangle = [robot1.position(node[0]), robot1.position(node[1]), robot1.position(node[2])]
		Grid.append(canvas.create_polygon(triangle[0][0], triangle[0][1], triangle[1][0], triangle[1][1],
		                                  triangle[1][0], triangle[1][1], triangle[2][0], triangle[2][1],
		                                  triangle[2][0], triangle[2][1], triangle[0][0], triangle[0][1], outline = "black", fill = "gray"))
	
	for i, node in enumerate(robot1.triangle_nodes):
		triangle = [robot1.position(node[0]), robot1.position(node[1]), robot1.position(node[2])]
		canvas.coords(Grid[i], world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin),
		              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
		              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
		              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
		              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
		              world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
	
	r1_state_numbers_of_goals = [i + 1 for i, value in enumerate(robot1.state_list) if int(value.strip()) in robot1.not_visted_goals]
	r2_state_numbers_of_goals = [i + 1 for i, value in enumerate(robot2.state_list) if int(value.strip()) in robot2.not_visted_goals]
	r1_poly_pix = []
	r2_poly_pix = []
	
	for i, goals in enumerate(robot1.not_visted_goals):
		r1_poly_xy = robot1.vertices_poses(r1_state_numbers_of_goals[i])
		r1_poly_pix.append(((r1_poly_xy[0][0] - world_xmin) * world_scale, (r1_poly_xy[0][1] - world_ymin) * world_scale, (r1_poly_xy[1][0] - world_xmin) * world_scale,
		                    (r1_poly_xy[1][1] - world_ymin) * world_scale,
		                    (r1_poly_xy[2][0] - world_xmin) * world_scale, (r1_poly_xy[2][1] - world_ymin) * world_scale))
		r1_goals_polygons = canvas.create_polygon(r1_poly_pix[i], fill = '#ffff00', outline = 'black')
	
	for i, goals in enumerate(robot2.not_visted_goals):
		r2_poly_xy = robot2.vertices_poses(r2_state_numbers_of_goals[i])
		r2_poly_pix.append(((r2_poly_xy[0][0] - world_xmin) * world_scale, (r2_poly_xy[0][1] - world_ymin) * world_scale, (r2_poly_xy[1][0] - world_xmin) * world_scale, (r2_poly_xy[1][
			                                                                                                                                                                  1] - world_ymin) * world_scale,
		                    (r2_poly_xy[2][0] - world_xmin) * world_scale, (r2_poly_xy[2][1] - world_ymin) * world_scale))
		r2_goals_polygons = canvas.create_polygon(r2_poly_pix[i], fill = '#006600', outline = 'black')
	
	win.update()
	canvas.postscript(file = 'snap.ps', colormode = 'color')
	os.system('convert snap.ps snap.gif')


# ======================================================================================================================
# draw the agents
# ======================================================================================================================
def drawWorld():
	global r1_goals_polygons, r2_goals_polygons, r1_poly_pix, r2_poly_pix
	var_r1_auto.set('Auto={}'.format(robot1.auto))
	var_r2_auto.set('Auto={}'.format(robot2.auto))
	# for i, node in enumerate(robot1.triangle_nodes):
	# 	triangle = [robot1.position(node[0]), robot1.position(node[1]), robot1.position(node[2])]
	# 	canvas.coords(Grid[i], world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin),
	# 	              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
	# 	              world_scale * (triangle[1][0] - world_xmin), world_scale * (triangle[1][1] - world_ymin),
	# 	              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
	# 	              world_scale * (triangle[2][0] - world_xmin), world_scale * (triangle[2][1] - world_ymin),
	# 	              world_scale * (triangle[0][0] - world_xmin), world_scale * (triangle[0][1] - world_ymin))
	
	goals_init_image = PhotoImage(file = 'snap.gif')
	initial_image = canvas.create_image(pixelsize / 2, -int(pixelsize * world_height / world_width) / 2, image = goals_init_image)
	canvas.coords(initial_image, pixelsize / 2, -int(pixelsize * world_height / world_width) / 2)
	# canvas.lower(initial_image)
	canvas.coords(uav_1_image, world_scale * (robot1.current_position[0] - world_xmin), world_scale * (robot1.current_position[1] - world_ymin))
	canvas.coords(uav_2_image, world_scale * (robot2.current_position[0] - world_xmin), world_scale * (robot2.current_position[1] - world_ymin))
	canvas.lift(uav_1_image)
	canvas.lift(uav_2_image)
	traj_size = .02
	traj1 = (world_scale * (robot1.current_position[0] - traj_size - world_xmin), world_scale * (robot1.current_position[1] - traj_size - world_ymin), world_scale * (
		robot1.current_position[0] - world_xmin + traj_size), world_scale * (robot1.current_position[1] - world_ymin + traj_size))
	traj2 = (world_scale * (robot2.current_position[0] - traj_size - world_xmin), world_scale * (robot2.current_position[1] - traj_size - world_ymin), world_scale * (
		robot2.current_position[0] - world_xmin + traj_size), world_scale * (robot2.current_position[1] - world_ymin + traj_size))
	traj1_canvas = canvas.create_rectangle(traj1, fill = 'yellow', outline = 'yellow')
	traj2_canvas = canvas.create_rectangle(traj2, fill = 'green', outline = 'green')
	canvas.lift(traj1_canvas)
	canvas.lift(traj2_canvas)


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
	
	if event.keysym == "1":
		if robot1.auto == False:
			robot1.reinitial = True
			robot1.auto = True
		else:
			if robot2.auto == False:
				robot2.reinitial = True
				robot2.auto = True
			robot1.auto = False
	
	if event.keysym == "2":
		if robot2.auto == False:
			robot2.reinitial = True
			robot2.auto = True
		else:
			if robot1.auto == False:
				robot1.reinitial = True
				robot1.auto = True
			robot2.auto = False
	
	if event.keysym == "i":
		if robot1.auto == False:
			robot1.reinitial = True
		
		if robot2.auto == False:
			robot2.reinitial = True


# ======================================================================================================================
# update the simulation
# ======================================================================================================================
def updateSim(dt):
	current_distance = np.sqrt((robot1.current_position[0] - robot2.current_position[0]) ** 2 + (robot1.current_position[1] - robot2.current_position[1]) ** 2)
	# print 'distance', current_distance
	if not robot1.is_finished and robot1.auto == True:
		robot1.current_position, robot1.curent_velocity, robot1.current_state, robot1.current_patch, robot1.is_finished = robot1.move(robot1.current_position, robot1.curent_velocity,
		                                                                                                                              robot1.current_state, robot1.current_patch,
		                                                                                                                              dt = 0.05)
	elif robot1.auto == False:
		robot1.manual_drive(dt)
	
	if not robot2.is_finished and robot2.auto == True:
		robot2.current_position, robot2.curent_velocity, robot2.current_state, robot2.current_patch, robot2.is_finished = robot2.move(robot2.current_position, robot2.curent_velocity,
		                                                                                                                              robot2.current_state, robot2.current_patch,
		                                                                                                                              dt = 0.05)
	elif robot2.auto == False:
		robot2.manual_drive(dt)
	
	# if is_collision == False:
	# 	try:
	# 		canvas.delete(robot1.i1)
	# 		canvas.delete(robot1.i2)
	# 		canvas.delete(robot1.i3)
	# 		canvas.delete(robot1.i4)
	# 	except:
	# 		robot1.i1 = canvas.create_line(0, 0, 0, 0)
	# 		robot1.i2 = canvas.create_line(0, 0, 0, 0)
	# 		robot1.i3 = canvas.create_line(0, 0, 0, 0)
	# 		robot1.i4 = canvas.create_line(0, 0, 0, 0)
	#
	# 	try:
	# 		canvas.delete(robot2.i1)
	# 		canvas.delete(robot2.i2)
	# 		canvas.delete(robot2.i3)
	# 		canvas.delete(robot2.i4)
	# 	except:
	# 		robot2.i1 = canvas.create_line(0, 0, 0, 0)
	# 		robot2.i2 = canvas.create_line(0, 0, 0, 0)
	# 		robot2.i3 = canvas.create_line(0, 0, 0, 0)
	# 		robot2.i4 = canvas.create_line(0, 0, 0, 0)
	
	if current_distance < 4.0:
		is_collision = collision_avoidance(robot1, robot2)
	
	
	# robot1.curent_velocity = robot1.collision_check(robot1.current_position, robot2.current_position, robot1.curent_velocity, robot2.curent_velocity, robot1.agent_id,
	#                                                 robot2.agent_id)
	# robot2.curent_velocity = robot2.collision_check(robot2.current_position, robot1.current_position, robot2.curent_velocity, r1_last_velocity, robot2.agent_id,
	#                                                 robot1.agent_id)
	#
	#
	# if not robot1.is_finished and robot1.auto == True:
	# 	robot1.current_position, robot1.curent_velocity, robot1.current_state, robot1.current_patch, robot1.is_finished = robot1.move(robot1.current_position, robot1.curent_velocity,
	# 	                                                                                                                              robot1.current_state, robot1.current_patch,
	# 	                                                                                                                              dt = 0.05)
	# 	if robot1.state_has_changed == True:
	# 		robot1.current_state = robot1.current_state - 1
	#
	# 	if current_distance < 2.0:
	# 		robot1.curent_velocity = robot1.collision_check(robot1.current_position, robot2.current_position, robot1.curent_velocity, robot2.curent_velocity, robot1.agent_id,
	# 		                                                robot2.agent_id)
	# 	else:
	# 		try:
	# 			canvas.delete(robot1.i1)
	# 			canvas.delete(robot1.i2)
	# 			canvas.delete(robot1.i3)
	# 			canvas.delete(robot1.i4)
	# 		except:
	# 			#print('*1')
	# 			robot1.i1 = canvas.create_line(0, 0, 0, 0)
	# 			robot1.i2 = canvas.create_line(0, 0, 0, 0)
	# 			robot1.i3 = canvas.create_line(0, 0, 0, 0)
	# 			robot1.i4 = canvas.create_line(0, 0, 0, 0)
	# 		# canvas.delete(robot1.i1)
	# 		# canvas.delete(robot1.i2)
	# 		# canvas.delete(robot1.i3)
	# 		# canvas.delete(robot1.i4)
	#
	# 	if robot1.state_has_changed == True:
	# 		robot1.current_state = robot1.current_state + 1
	# elif robot1.auto == False:
	# 	robot1.manual_drive(dt)
	#
	# r1_last_velocity = robot1.curent_velocity
	#
	# if not robot2.is_finished and robot2.auto == True:
	# 	robot2.current_position, robot2.curent_velocity, robot2.current_state, robot2.current_patch, robot2.is_finished = robot2.move(robot2.current_position, robot2.curent_velocity,
	# 	                                                                                                                              robot2.current_state, robot2.current_patch,
	# 	                                                                                                                              dt = 0.05)
	#
	# 	if robot2.state_has_changed == True:
	# 		robot2.current_state = robot2.current_state - 1
	#
	# 	if current_distance < 2.0:
	# 		robot2.curent_velocity = robot2.collision_check(robot2.current_position, robot1.current_position, robot2.curent_velocity, r1_last_velocity, robot2.agent_id,
	# 		                                                robot1.agent_id)
	# 	else:
	# 		try:
	# 			canvas.delete(robot2.i1)
	# 			canvas.delete(robot2.i2)
	# 			canvas.delete(robot2.i3)
	# 			canvas.delete(robot2.i4)
	# 		except:
	# 			robot2.i1 = canvas.create_line(0,0,0,0)
	# 			robot2.i2 = canvas.create_line(0,0,0,0)
	# 			robot2.i3 = canvas.create_line(0,0,0,0)
	# 			robot2.i4 = canvas.create_line(0,0,0,0)
	#
	# 	if robot2.state_has_changed == True:
	# 		robot2.current_state = robot2.current_state + 1
	# elif robot2.auto == False:
	# 	robot2.manual_drive(dt)


# =====================================================================================================================================================================================
# read geometry
# =====================================================================================================================================================================================
def readScenario(geometry, scalex = 1., scaley = 1.):
	global x_in, y_min
	x_min = 0 * scalex - 1.
	y_min = 0 * scaley - 1.
	x_max = 10 * scalex + 1.
	y_max = 10 * scaley + 1.
	return x_min, x_max, y_min, y_max


# =====================================================================================================================================================================================
# simulate and draw frames
# =====================================================================================================================================================================================
def drawFrame(dt):
	global start_time, step, paused, ittr, globalTime
	if ittr > maxIttr or QUIT:  # Simulation Loop
		# print("%s itterations ran ... quitting" % ittr)
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
robot1 = Robot(r1_geometry, .1, 1, 2., r1_goals, r1_pos_init)
robot2 = Robot(r2_geometry, .1, 2, 2., r2_goals, r2_pos_init)
# robot1.plot_patch_vf()
geometry = robot1.geometry_name
world_xmin, world_xmax, world_ymin, world_ymax = readScenario(geometry)


def callback(event):
	point = [event.x / world_scale + world_xmin, event.y / world_scale + world_ymin]
	# print 'clicked at ', point
	if robot1.auto == False:
		robot1.waypoint = point
	if robot2.auto == False:
		robot2.waypoint = point


world_width = world_xmax - world_xmin
world_height = world_ymax - world_ymin
world_scale = pixelsize / world_width
win = Tk()
frame = Frame(win, width = 1200, height = 900, bg = '#ffffff')
frame.pack()
quad1 = PhotoImage(file = 'quad1.gif')
quad1 = quad1.subsample(35, 35)
quad2 = PhotoImage(file = 'quad2.gif')
quad2 = quad2.subsample(35, 35)
label_r1 = Label(frame, image = quad1, relief = RAISED)
label_r1.pack()
label_r1.place(x = 0, y = 300, width = 80, height = 50)
label_r2 = Label(frame, image = quad2, relief = RAISED)
label_r2.pack()
label_r2.place(x = 0, y = 350, width = 80, height = 50)
var_r1_auto = StringVar()
var_r2_auto = StringVar()
label_r1_auto = Label(frame, textvariable = var_r1_auto, relief = RAISED, width = 30)
label_r1_auto.pack()
label_r1_auto.place(x = 80, y = 300, width = 80, height = 50)
label_r2_auto = Label(frame, textvariable = var_r2_auto, relief = RAISED, width = 30)
label_r2_auto.pack()
label_r2_auto.place(x = 80, y = 350, width = 80, height = 50)

win.bind("<Button-1>", callback)
win.bind("<space>", on_key_press)
win.bind("<space>", on_key_press)
win.bind("s", on_key_press)
win.bind("<Escape>", on_key_press)
win.bind("v", on_key_press)
win.bind("i", on_key_press)
win.bind("1", on_key_press)
win.bind("2", on_key_press)
canvas = Canvas(frame, width = pixelsize, height = pixelsize * world_height / world_width, background = "#ffffff")
canvas.pack()
canvas.place(x = 200, y = 0)
white = (255, 255, 255)
image1 = Image.new("RGB", (pixelsize / 2, int(pixelsize * world_height / world_width) / 2))
draw = ImageDraw.Draw(image1)
initWorld(canvas)
win.title('UniCycle Simulation')
start_time = time.time()
win.after(framedelay, lambda: drawFrame(dt))
trajectory_image = PhotoImage(file = 'trajectory_im.gif')
trajectory_image = trajectory_image.subsample(800, 800)
trajectory_image_2 = PhotoImage(file = 'trajectory_im_2.gif')
trajectory_image_2 = trajectory_image_2.subsample(800, 800)
uav_1_image = canvas.create_image(200, 100, image = quad1)
uav_2_image = canvas.create_image(200, 100, image = quad2)

mainloop()

