import math
# import matplotlib.pyplot as plt

import JarvisMarch
from operator import add
import numpy as np
from scipy.spatial import Delaunay
import vertice_gs



def in_hull(p, hull):
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)
    return hull.find_simplex(p)>=0

def max_admis_vel(theta):
	for i in np.linspace(0., vel_max, 100):
		# last_point_inside = i
		if ~(in_hull([i * math.cos(theta), i * math.sin(theta)], L)):
			max_u1u2 = [last_point_inside * math.cos(theta), last_point_inside * math.sin(theta)]
			print '\n', 'max admissible velocity', max_u1u2, '0 ---> i/100 --> v_max: ', i
			break
		last_point_inside = i
	
def point_g_calculator(point_xy, vertice1, common_facet_points, vel_max):
	# common facet points are the vertices 2 & 3
	v = list([vertice1, common_facet_points[0], common_facet_points[1]])  # vertices
	default_gs = vertice_gs.default_gs(v)
	# print 'default gs', default_gs
	W = [[v[0][0], v[1][0], v[2][0]], [v[0][1], v[1][1], v[2][1]], [1.0, 1.0, 1.0]]
	inv_W = np.matrix(W) ** -1
	# p = point_xy
	(x, y) = (point_xy[0], point_xy[1])
	# lambdas = np.dot(vel_max,np.array(inv_W * [[x],[y], [1]]))  # .transpose()
	print '(x,y)', x,y, vertice1, common_facet_points
	lambdas = np.array(inv_W * [[x],[y], [1]])
	# print 'lambdas', lambdas
	g_total_333 = [[[0.,0.],default_gs[0][0],default_gs[0][1]],[[0.,0.],default_gs[1][0],default_gs[1][1]],[[0.,0.],default_gs[2][0],default_gs[2][1]]]
	t1 = lambdas[0] * g_total_333[0]
	t2 = lambdas[1] * g_total_333[1]
	t3 = lambdas[2] * g_total_333[2]
	# print  'g_total_333', g_total_333, 'lambdas', lambdas
	points = list()
	it = 0
	for i in t1:
		for j in t2:
			for k in t3:
				it += 1
				points.append(map(add, map(add, i, j), k))
	# print map(add, p1, p2, p3)
	# print 'points', points, 'number of points', len(points)
	try:
		JarvisMarch.main(np.array(points))
	except ValueError:
		# print 'points', points
		L = JarvisMarch.main(np.array(points))
	L = JarvisMarch.main(np.array(points))
	return L

def lambda_calc(point_xy, vertice1, common_facet_points):
	# common facet points are the vertices 2 & 3
	v = list([vertice1, common_facet_points[0], common_facet_points[1]])  # vertices
	default_gs = vertice_gs.default_gs(v)
	W = [[v[0][0], v[1][0], v[2][0]], [v[0][1], v[1][1], v[2][1]], [1.0, 1.0, 1.0]]
	inv_W = np.matrix(W) ** -1
	# p = point_xy
	(x, y) = (point_xy[0], point_xy[1])
	vel_max = 1.
	lambdas = np.dot(vel_max,np.array(inv_W * [[x],[y], [1]]))
	return lambdas


