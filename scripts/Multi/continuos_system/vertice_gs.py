# from interval import interval
import vector_field
import numpy as np
import math
# build the triangle
# get 3 points from the user (mouse input)
# v = [[x1,y1],[x2,y2],[x3,y3]]
def default_gs(v):
	v1, v2, v3 = np.array([v[0], v[1], v[2]])
	[n1, n2, n3] = vector_field.find_norms(v1, v2, v3)
	# print 'n1 n2 n3:', n1, n2, n3
	r1 = vector_field.find_range_v(n1, 1)
	r2 = vector_field.find_range_v(n2, -1)
	r3 = vector_field.find_range_v(n3, -1)
	p_new = list()
	p_new.append(r1 & r2 & r3)
	p_new.append(r1 & r3)
	p_new.append(r1 & r2)
	# print 'angle_radians', p_new[0]
	g1 = [p_new[0][0].inf, p_new[0][0].sup]
	g2 = [p_new[1][0].inf, p_new[1][0].sup]
	g3 = [p_new[2][0].inf, p_new[2][0].sup]
	# g = [g1, g2, g3]
	g_triangle_1 = [[math.cos(math.radians(g1[0])), math.sin(math.radians(g1[0]))], [math.cos(math.radians(g1[1])),\
	                                                                                 math.sin(math.radians(g1[1]))]]
	g_triangle_2 = [[math.cos(math.radians(g2[0])), math.sin(math.radians(g2[0]))], [math.cos(math.radians(g2[1])),\
	                                                                             math.sin(math.radians(g2[1]))]]
	g_triangle_3 = [[math.cos(math.radians(g3[0])), math.sin(math.radians(g3[0]))], [math.cos(math.radians(g3[1])),\
	                                                                                 math.sin(math.radians(g3[1]))]]
	g_triangles_xy = [g_triangle_1, g_triangle_2, g_triangle_3]
	return g_triangles_xy
G = []
#calculates the point velocity in a given triangle for default g values and current position
def simple_point_vel(current_position,triangle):
	global G
	g = default_gs(triangle)
	# print 'g',g
	pos = current_position
	p = np.array([pos[0], pos[1]])
	v1 = np.array(triangle[0])
	v2 = np.array(triangle[1])
	v3 = np.array(triangle[2])
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
		a1 = (np.array(g[0][0]) + np.array(g[0][1]))
		# print 'a1', a1, np.array(g[0][0]), np.array(g[0][1])
		angle_g1 = math.atan2(a1[1],a1[0])
		# print 'agnle g1', angle_g1*180/np.pi
		a2 = (np.array(g[1][0]) + np.array(g[1][1]))
		# print 'a2',a2, np.array(g[1][0]), np.array(g[1][1])
		angle_g2 = math.atan2(a2[1],a2[0])
		# print 'agnle g1', angle_g2 * 180 / np.pi
		a3 = (np.array(g[2][0]) + np.array(g[2][1]))
		angle_g3 = math.atan2(a3[1],a3[0])
		# print 'agnle g1', angle_g3 * 180 / np.pi
		g1 = np.array([np.cos(angle_g1), np.sin(angle_g1)])
		g2 = np.array([np.cos(angle_g2), np.sin(angle_g2)])
		g3 = np.array([np.cos(angle_g3), np.sin(angle_g3)])
		# print 'g1', g1, 'g2', g2, 'g3', g3
		Glocal = np.matrix([g1, g2, g3]).transpose()
		W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
		inv_W = np.matrix(W) ** -1
		GW = Glocal * inv_W
		velocity =  np.array(GW * [[pos[0]], [pos[1]], [1]]).transpose()
	else:
		velocity = None
	# print velocity , 'refrence velocity'
	return velocity
	
	
# simple_point_vel([0.,1.],[[0.,4.],[-2.,0.],[2.,0.]])
	
	