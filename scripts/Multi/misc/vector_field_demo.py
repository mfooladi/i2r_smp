import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


fig3 = plt.figure()
ax3 = fig3.add_subplot(111, aspect='equal')
# vertices v1 = [v1_x, v1_y]
v1 = np.array([1.0, 1.0])
v2 = np.array([2.0, 5.0])
v3 = np.array([6.0, 3.0])
# velocity @ vertices g1 = [g1_x, g1_y]
g1 = np.array([3.5, 3.0])
g2 = np.array([1., -2.])
g3 = np.array([-1.5, .0])

# vertices = [(1.0, 1.0), (2.0, 5.0), (6.0, 3.0)]
# print vertices
# G matrix
G = np.matrix([g1, g2, g3])

G = G.transpose()
# print 'G', G
# W matrix
W = [[v1[0], v2[0], v3[0]], [v1[1], v2[1], v3[1]], [1.0, 1.0, 1.0]]
W_matrix = np.matrix(W)
inv_W = W_matrix**-1
# GW matrix
GW = G * inv_W
# print 'gw', GW
#the outward normal vector to the Facet #1
midpoint13 = (v1 + v3)/2
midpoint23 = (v2 + v3)/2
midpoint12 = (v1 + v2)/2

v1_to_midpoint = midpoint23 - v1
v2_to_midpoint = midpoint13 - v2
v3_to_midpoint = midpoint12 - v3

if np.dot(v1_to_midpoint, [1, -(v3-v2)[0]/(v3-v2)[1]])>0:
    n_1 = [1, -(v3-v2)[0]/(v3-v2)[1]]
else:
    n_1 = [-1, (v3-v2)[1]/(v3-v2)[0]]

if np.dot(v2_to_midpoint, [1, -(v1-v3)[0]/(v1-v3)[1]])>0:
    n_2 = [1, -(v1-v3)[0]/(v1-v3)[1]]
else:
    n_2 = [-1, (v1-v3)[0]/(v1-v3)[1]]

if np.dot(v3_to_midpoint, [1, -(v1-v2)[0]/(v1-v2)[1]])>0:
    n_3 = [1, -(v1-v2)[0]/(v1-v2)[1]]
else:
    n_3 = [-1, (v1-v2)[0]/(v1-v2)[1]]

n1g1 = np.dot(n_1,g1)
n1g2 = np.dot(n_1,g2)
n1g3 = np.dot(n_1,g3)
n2g1 = np.dot(n_2, g1)
n2g2 = np.dot(n_2, g2)
n2g3 = np.dot(n_2, g3)
n3g1 = np.dot(n_3, g1)
n3g2 = np.dot(n_3, g2)
n3g3 = np.dot(n_3, g3)

print 'n1g1', n1g1, bool(n1g1 > 0)
print 'n2g1', n2g1, bool(n2g1 <= 0)
print 'n3g1', n3g1, bool(n3g1 <= 0)
print 'n1g2', n1g2, bool(n1g2 > 0)
print 'n2g2', n2g2 #bool(n2g2 <= 0)
print 'n3g2', n3g2, bool(n3g2 <= 0)
print 'n1g3', n1g3, bool(n1g3 > 0)
print 'n2g3', n2g3, bool(n2g3 <= 0)
print 'n3g3', n3g3, #bool(n3g3 <= 0)

# temporary,
a = np.linspace(0, 8, 30)
for i, item_x in enumerate(a):
    for j, item_y in enumerate(a):
        p = np.array([item_x, item_y])
        vector0 = v3 - v1
        vector1 = v2 - v1
        vector2 = p - v1
        # print vector2
        dot00 = np.dot(vector0, vector0)
        dot01 = np.dot(vector0, vector1)
        dot02 = np.dot(vector0, vector2)
        dot11 = np.dot(vector1, vector1)
        dot12 = np.dot(vector1, vector2)
        invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * invDenom
        v = (dot00 * dot12 - dot01 * dot02) * invDenom

        if (u>=0) and (v>=0) and (u+v < 1):
            velocity_p = GW * [[item_x], [item_y], [1]]
            print(item_x, item_y, velocity_p)
            # print velocity_p
            # print 'u', u, 'v', v
            # print 'x', item_x, 'y', item_y, 'vx', float(velocity_p[0]), 'vy', float(velocity_p[1])
            plt.quiver(item_x, item_y, float(velocity_p[0]), float(velocity_p[1]), scale=10, scale_units = 'xy', width = 0.001)
plt.plot([1, 2, 6, 1], [1, 5, 3, 1])
plt.plot([6, 5, 2], [3, 6, 5],'b')
plt.axis([0, 8, 0, 8])


plt.quiver(v1[0], v1[1], g1[0], g1[1])
plt.quiver(v2[0], v2[1], g2[0], g2[1])
plt.quiver(v3[0], v3[1], g3[0], g3[1])
plt.quiver(midpoint23[0], midpoint23[1], n_1[0], n_1[1])
plt.quiver(midpoint12[0], midpoint12[1], n_3[0], n_3[1])
plt.quiver(midpoint13[0], midpoint13[1], n_2[0], n_2[1])

p0 =[[2.2], [2.2], [1]]
p = p0
dt = 0.01
# uncomment the following "for loop" for plotting the fully actuated robot trajectory
# for i in range(0,250):
#     pos = GW * p
#     p[0][0] = float(pos[0]) * dt + float(p[0][0])
#     p[1][0] = float(pos[1]) * dt + float(p[1][0])
#     plt.plot(float(p[0][0]), float(p[1][0]), '.')
#     plt.pause(0.00001)

e_1 = [[0, -1], [1, 0]]
e_2 = [[1, 0], [0, 0.2]]
theta = -1.
pos =[4., 2.]
p0 =[[2.2], [2.2], [1]]
# uncomment the following "for loop" for plotting the under-actuated robot trajectory
for j in range(0,100):
    vel = GW * p
    r_theta = [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]

    u = np.matrix(e_2)**-1 * np.matrix(r_theta).transpose() * vel
    # if u[0]>1:
    #     u[0] = 1
    # if u[1] > 4:
    #     u[1] = 4

    d_dot = np.matrix(r_theta) * np.matrix([[float(u[0])], [0]])
    # print 'ddot', d_dot[0][0]
    # r_dot = r_theta * np.matrix(e_1) * float(u[1])
    # r_theta = r_dot * dt + r_theta
    theta = float(u[1]) * dt +theta
    pos[0] = float(d_dot[0][0]) * dt + pos[0]
    pos[1] = float(d_dot[1][0]) * dt + pos[1]
    # print 'pos', pos
    p[0][0]=np.cos(theta)*
    # p[0][0] = pos[0] * dt + float(p[0][0])
    # p[1][0] = pos[1] * dt + float(p[1][0])
    # print '1', p[0][0]
    plt.plot(float(p[0][0]), float(p[1][0]),'^')
    plt.quiver(float(p[0][0]), float(p[1][0]), np.cos(theta), np.sin(theta), scale=3, scale_units = 'xy', width = 0.001)
    xy =[float(p[0][0]), float(p[1][0])]

    ax3.add_patch(patches.Rectangle(xy, 0.5, 0.1, theta*180/np.pi, fill = False))
    plt.pause(0.000001)
plt.show()