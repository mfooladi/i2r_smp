import numpy as np
import math
from interval import interval
import matplotlib.pyplot as plt
import copy
import itertools
import triangle_plot
import os.path
# from PIL import _imaging
parent_dir = os.path.dirname(__file__)+'/../'
# control input bound
V_max = 25
colors = itertools.cycle(["r", "b", "y"])
# G = []
# States = []
triangles = []
vertices_poses = []
# function for finding outer normal vectors given 3 vertices of a triangle
def find_norms(v1, v2, v3):
    # print 'v1 v2 v3', v1, v2, v3
    midpoint13 = (v1 + v3)/2.
    midpoint23 = (v2 + v3)/2.
    midpoint12 = (v1 + v2)/2.

    v1_to_midpoint = midpoint23 - v1
    v2_to_midpoint = midpoint13 - v2
    v3_to_midpoint = midpoint12 - v3
    if (v3-v2)[1] != 0:
        if np.dot(v1_to_midpoint, [1, -(v3-v2)[0]/(v3-v2)[1]])>0:
            n_1 = [1, -(v3-v2)[0]/(v3-v2)[1]]
        else:
            n_1 = [-1, (v3-v2)[0]/(v3-v2)[1]]
    else:
        n_1 = [0, np.sign(v1_to_midpoint[1])]

    if (v1-v3)[1] != 0:
        if np.dot(v2_to_midpoint, [1, -(v1-v3)[0]/(v1-v3)[1]])>0:
            n_2 = [1, -(v1-v3)[0]/(v1-v3)[1]]
        else:
            n_2 = [-1, (v1-v3)[0]/(v1-v3)[1]]
    else:
        n_2 = [0, np.sign(v2_to_midpoint[1])]
    if (v1-v2)[1] != 0:
        if np.dot(v3_to_midpoint, [1, -(v1-v2)[0]/(v1-v2)[1]])>0:
            n_3 = [1, -(v1-v2)[0]/(v1-v2)[1]]
        else:
            n_3 = [-1, (v1-v2)[0]/(v1-v2)[1]]
    else:
        n_3 = [0, np.sign(v3_to_midpoint[1])]
    return[n_1,n_2,n_3]

# find the range of the slope of the admissible vector function at front vertices
def find_range_v(n, sgn=1):
    # sgn =1 for v1
    # sgn =-1 for v2 and v3
    sgn_b = np.sign(n[0])
    r1 = math.atan2(n[0], -n[1])
    r2 = math.atan2(-n[0], n[1])
    r1_degree = r1*180/np.pi
    r2_degree = r2*180/np.pi

    if sgn_b == 0:
        if sgn*n[1] > 0:
            r = [0,np.pi]
            angle = interval([0, 180])
        else:
            r = [-np.pi, 0]
            angle = interval([-180, 0])
        return angle
    if sgn > 0:
        if sgn_b > 0:
            r = [[r2, 0], [0, r1]]
            angle = interval([r2_degree, 0]) | interval([0, r1_degree])
        else:
            r = [[-np.pi, r1], [r2, np.pi]]
            angle = interval([-180, r1_degree]) | interval([r2_degree, 180])
    else:
        if sgn_b > 0:
            r = [[-np.pi, r2],[r1, np.pi]]
            angle = interval([-180, r2_degree]) | interval([r1_degree, 180])
        else:
            r = [[r1, 0], [0,r2]]
            angle = interval([r1_degree, 0]) | interval([0,r2_degree])
    # return np.array(r)*180/np.p
    return angle
# Returns the vector field for a point is within a specified triangle
# Returns None if point is not in the triangle
def vector_field_tri(x, y, tri, section):
    # print 'x, y', x, y
    # print 'tri, section', tri, section,
    p = np.array([x, y])
    v1id = triangles[tri-1][0]
    v2id = triangles[tri-1][1]
    v3id = triangles[tri-1][2]
    # print 'v1id, v2id, v3id:', v1id, v2id, v3id
    v1 = np.array(vertices_poses[v1id-1])
    v2 = np.array(vertices_poses[v2id-1])
    v3 = np.array(vertices_poses[v3id-1])
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
        # G matrix
        g1 = np.array([np.cos(G[section][v1id]*np.pi/180.), np.sin(G[section][v1id]*np.pi/180.)])
        # print 'G', G
        g2 = np.array([np.cos(G[section][v2id]*np.pi/180.), np.sin(G[section][v2id]*np.pi/180.)])
        g3 = np.array([np.cos(G[section][v3id]*np.pi/180.), np.sin(G[section][v3id]*np.pi/180.)])
        Glocal = np.matrix([g1, g2, g3]).transpose()
        # W matrix
        W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
        inv_W = np.matrix(W) ** -1
        # inv_W = W_matrix ** -1
        # GW matrix
        GW = Glocal * inv_W
        velocity =  np.array(GW * [[x], [y], [1]]).transpose()
    else:
        velocity = None
    return velocity

# Returns Vector field for any 2D-point
# def vector_field(x, y, section, States, G):
def vector_field(x, y, section):
    last = False
    for i, triangle in enumerate(States[section]):
        # print 'stage, states', section, States
        # velocity = vector_field_tri(x, y, triangle, section, G)
        velocity = vector_field_tri(x, y, triangle, section)
        if np.any(velocity):
            if i == len(States[section])-1:
                last = True
            return [velocity[0], last, triangle]
    # return [None, last ]
    return [None, last ]

def init_field(geometry_name):
    global G, States, triangles, vertices_poses
    print 'Initializing the Vector Field ...'
    # read the states' order
    with open(parent_dir + 'discrete_transition/state_order_{}.txt'.format(geometry_name)) as state:
        all_state = state.readlines()
    state.close()
    print 'all states', all_state

    # Triangles numbers and Vertices numbers
    with open(parent_dir+'geometry/{}.1.ele'.format(geometry_name)) as elements:
        neighbor = elements.readlines()
    elements.close()
    del(neighbor[-1])
    triangles = []# triangles = [[#ver.1, #ver.2, #ver.3],[#ver.1, #ver.2, #ver.3], ..., [#ver.1, #ver.2, #ver.3]]

    for i in xrange(1, len(neighbor)):
        triangle_nodes = neighbor[i].strip().split(' ')
        triangle_nodes = filter(None, triangle_nodes)
        triangle_nodes = map(int, triangle_nodes[1:])
        triangles.append(triangle_nodes)
    # print 'triangles', triangles

    # vertices positions
    with open(parent_dir+'geometry/{}.1.node'.format(geometry_name)) as vpos:
        vertices_pos = vpos.readlines()
    vpos.close()
    del(vertices_pos[-1])
    # print 'ver. pos.:', vertices_pos
    vertices_poses = []
    for i in xrange(1, len(vertices_pos)):
        vertice = vertices_pos[i].strip().split(' ')
        vertice = filter(None, vertice)
        vertice = map(float, vertice[1:3])
        vertices_poses.append(vertice)
    j = 1
    maz=1
    G = []
    States = []
    maz *= .85
    while j < len(all_state):
        V1 = []
        common_vertices = []
        Norms = []
        P = dict()
        Pnew = dict()
        C = dict()
        g = dict()
        states = []
        seen_vertice = []
        check_emptiness = True
        remained_state = all_state[j-1:]
        # print 'remained_state',remained_state
        firstG = True
        for i, current_triangle in enumerate(remained_state):
            # if current_triangle != remained_state[-1]:
            states.append(int(current_triangle))
            if current_triangle != all_state[-1]:
                next_triangle = remained_state[i+1]
                # print current_triangle, next_triangle
                common_vertice = list(set(triangles[int(current_triangle)-1][:]).intersection(triangles[int(next_triangle)-1][:]))
                v1 = list(set(triangles[int(current_triangle) - 1]) - set(common_vertice))[0]
                V1.append(v1)
                common_vertices.append(common_vertice)
                n = find_norms(np.array(vertices_poses[v1-1]), np.array(vertices_poses[common_vertice[0]-1]), np.array(vertices_poses[common_vertice[1]-1]))
                for i, vertice in enumerate(common_vertice):
                    if vertice in C:
                        C[vertice] = map(lambda x,y:x+y, n[0],C[vertice])
                    else:
                        C[vertice] = n[0]
                if v1 not in seen_vertice:
                    P[v1] = interval[-180, 180]
                    seen_vertice.append(v1)
                if common_vertice[0] not in seen_vertice:
                    P[common_vertice[0]] = interval[-180, 180]
                    seen_vertice.append(common_vertice[0])
                if common_vertice[1] not in seen_vertice:
                    P[common_vertice[1]] = interval[-180, 180]
                    seen_vertice.append(common_vertice[1])
                r1 = find_range_v(n[0], 1)
                r2 = find_range_v(n[1], -1)
                r3 = find_range_v(n[2], -1)
                Pnew[v1] = r1 & r2 & r3 & P[v1]
                Pnew[common_vertice[0]] = r1 & r3 & P[common_vertice[0]]
                Pnew[common_vertice[1]] = r1 & r2 & P[common_vertice[1]]
                if firstG:
                    g[v1] = Pnew[v1].midpoint[0][0]
                    # print('first G', g)
                    firstG = False
                key_items = list(Pnew.keys())
                length_ranges = {key: len(value) for key, value in Pnew.items()}
                check_emptiness = all(value != 0 for value in length_ranges.values())
                if check_emptiness:
                    # print 'check empty P=', P
                    P = copy.copy(Pnew)
                    Norms.append(n)
            if not check_emptiness:
                break
            j += 1

        # last triangle
        last_triangle = all_state[j-2]
        if current_triangle == all_state[-1]:
            last_triangle = all_state[j-2]
        else:
            common_vertice = common_vertices[-2]
            last_triangle = all_state[j-1]
        last_vs = triangles[int(last_triangle)-1]
        # print "last vertices = ", last_vs
        # print "triangle # = ", last_triangle
        # solo vertices
        last_v = list(set(triangles[int(last_triangle)-1])-set(common_vertice))[0]
        # n = find_norms(np.array(vertices_poses[last_vs[0]-1]), np.array(vertices_poses[last_vs[1]-1]), np.array(vertices_poses[last_vs[2]-1]))
        n = find_norms(np.array(vertices_poses[last_v-1]), np.array(vertices_poses[common_vertice[0]-1]), np.array(vertices_poses[common_vertice[1]-1]))
        # print('n= ', n)
        for v in last_vs:
            if v not in seen_vertice:
                P[v] = interval[-180, 180]
                seen_vertice.append(v)
        r1 = find_range_v(n[0], -1)
        r2 = find_range_v(n[1], -1)
        r3 = find_range_v(n[2], -1)
        P[last_v] = r2 & r3 & P[last_v]
        P[common_vertice[0]] = r1 & r3 & P[common_vertice[0]]
        P[common_vertice[1]] = r1 & r2 & P[common_vertice[1]]
        Norms.append(n)
        # Calculate g
        for i, c_i in enumerate(C.values()):
            # key = vertice number in C & P dictionaries
            key = C.keys()[i]
            Ct = C.values()[i]
            NCt = [-x for x in Ct]
            NCt = NCt[::-1]
            # print 'Ct', Ct, 'NCt', NCt
            # m1 & m2 are the slopes to the constraints
            if len(P[key]) == 1:
                m1 = P[key][0][0]
                m2 = P[key][0][1]
            elif len(P[key]) == 2:
                m1 = P[key][0][1]
                m2 = P[key][1][0]
            normal_angle = math.atan2(Ct[1], Ct[0])*180/np.pi
            # print 'normal angle', normal_angle
            if normal_angle <= m2 and normal_angle >= m1:
                opt_angle = normal_angle
            elif normal_angle > m2:
                opt_angle = m2
            else:
                opt_angle = m1
            # print 'opt_angle', opt_angle\
            g[key] = opt_angle
            # G[41] = 1
            # G[60] = 1
        if len(P[last_v]) == 1:
            g[last_v] = P[last_v].midpoint[0][0]
        elif len(P[last_v]) == 2:
            angle = (P[last_v].midpoint[0][0]+P[last_v].midpoint[1][0])*.5+180
            if angle > 180:
                angle-=360
            elif angle < -180:
                angle+=360
            g[last_v]=angle

        # print 'G', g
        G.append(g)
        States.append(states)
    num_stages = len(States)
    print "Vector Field is ready!"
    print "Number of stages: ", num_stages
    return num_stages, States, G

def plot_vector_field(gemotry_name = 'robot2', all_sections=False):
    # globals States, G
    init_field(gemotry_name)
    x_feasible = []
    y_feasible = []
    v_feasible = []
    ################Plot###################
    print ('G total = ', G)
    print ('States = ', States)

    if all_sections:
        f = plt.figure(figsize=(7.5, 9), dpi=100)
        plt.gca().invert_yaxis()
        for sec in range(len(States)):
            mycolor = next(colors)
            a = np.linspace(0, 10, 60)
            for i, x in enumerate(a):
                for j, y in enumerate(a):
                    [v, last, triangle] = vector_field(x, y, sec)
                    if np.any(v):
                        x_feasible.append(x)
                        y_feasible.append(y)
                        v_feasible.append(v)
                        # print(x, y, v)
                        # plt.plot(vertices_poses[vertice-1][0], vertices_poses[vertice-1][1],'.')
                        # plt.quiver(x, y, np.cos(v*np.pi/180), np.sin(v*np.pi/180),  width=0.003*maz, color=mycolor)
                        plt.quiver(x, y, float(v[0]*1.75), -float(v[1]*1.75), scale=10, scale_units='xy', width=0.0015)
        triangle_plot.plot_triangles(show_path=True)
        plt.show()
        f.savefig(parent_dir + 'figures/sectio_{}.png'.format(gemotry_name), bbox_inches='tight')
        f.savefig(parent_dir + 'figures/sectio_{}.pdf'.format(gemotry_name), bbox_inches='tight')
    else:
        for sec in range(len(States)):
            f = plt.figure(figsize=(7.5, 9), dpi=100)
            plt.gca().invert_yaxis()
            # mycolor = next(colors)
            a = np.linspace(0, 10, 60)
            for i, x in enumerate(a):
                for j, y in enumerate(a):
                    # v, last = vector_field(x, y, sec, States_v, G_v)
                    vv = vector_field(x, y, sec)
                    v = vv[0]
                    print 'v', v
                    if np.any(v):
                        x_feasible.append(x)
                        y_feasible.append(y)
                        v_feasible.append(v)
                        # print(x, y, v)
                        plt.quiver(x, y, float(v[0]*1.75), -float(v[1]*1.75), scale=10, scale_units='xy', width=0.0015)
            triangle_plot.plot_triangles(show_path=False)
            plt.show()
            # f.savefig(parent_dir+'figures/section_{}_{}.png'.format(gemotry_name, sec), bbox_inches='tight')
    return x_feasible, y_feasible, v_feasible
            # f.savefig("section_{}.pdf".format(sec), bbox_inches='tight')


# if __name__ == "__main__":
#     plot_vector_field('dscc_simulation', False)

if __name__ == '__main__':
    print