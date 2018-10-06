import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import os.path
parent_dir = os.path.dirname(__file__)+'/../'
# geometry_name = 'dscc_simulation'
geometry_name='robot2'

# def plot_triangles(gemotry_name='Proposal', show_numbers = False, show_path=True):
def plot_triangles(show_numbers = True, show_path = True, ):
    global geometry_name
    with open(parent_dir+'geometry/{}.1.ele'.format(geometry_name)) as ele:
        tri_num = ele.readlines()
        # print 'tri num', tri_num
    ele.close()
    del tri_num[-1]
    with open(parent_dir+'geometry/{}.1.node'.format(geometry_name)) as node:
            node_num = node.readlines()
    node.close()
    del node_num[-1]

    with open(parent_dir + 'discrete_transition/state_order_{}.txt'.format(geometry_name)) as s_order:
        state_order = s_order.readlines()
    s_order.close()

    with open(parent_dir + 'geometry/{}.1.v.node'.format(geometry_name)) as node_v:
            triangle_center = node_v.readlines()
    node_v.close()
    del triangle_center[-1]

    ##################################################################
    triangle_node_list = []
    nodes_xy_list = []
    for i in xrange(1, len(tri_num)):
        triangle_nodes = tri_num[i].strip().split(' ')
        triangle_nodes = filter(None, triangle_nodes)
        triangle_node_list.append(triangle_nodes)

    for i in xrange(1, len(node_num)):
        nodes_xy = node_num[i].strip().split(' ')
        nodes_xy = filter(None, nodes_xy)
        nodes_xy_list.append(nodes_xy)
    tri_xy_list=[]
    for i in xrange(len(triangle_center)):
        tri_center_xy = triangle_center[i].strip().split(' ')
        tri_center_xy = filter(None, tri_center_xy)
        tri_xy_list.append(tri_center_xy)
    # print nodes_xy_list.index(?
    # print 'tri node list', triangle_node_list
    x = [xx[1] for xx in nodes_xy_list]
    y = [xx[2] for xx in nodes_xy_list]
    # print len(triangle_node_list)
    x_node = [xnode[0] for xnode in nodes_xy_list]
    # print x_node

    Center_points=[]
    for i in xrange(len(triangle_node_list)):
        index = triangle_node_list[i]
        x1 = nodes_xy_list[int(index[1])-1][1]
        y1 = nodes_xy_list[int(index[1])-1][2]
        plt.plot(float(x1),float(y1),'.k')
        x2 = nodes_xy_list[int(index[2]) - 1][1]
        y2 = nodes_xy_list[int(index[2]) - 1][2]
        plt.plot(float(x2), float(y2), '.k')
        x3 = nodes_xy_list[int(index[3]) - 1][1]
        y3 = nodes_xy_list[int(index[3]) - 1][2]
        plt.plot(float(x3), float(y3), '.k')
        plt.plot([float(x1), float(x2)],[float(y1), float(y2)],'k')
        plt.plot([float(x1), float(x3)], [float(y1), float(y3)], 'k')
        plt.plot([float(x2), float(x3)], [float(y2), float(y3)], 'k')
        x_avg = (float(x1)+float(x2)+float(x3))/3.
        y_avg = (float(y1)+float(y2)+float(y3))/3.
        if show_numbers:
            plt.text(x_avg,y_avg, str(i+1))
        Center_points.append([index, x_avg, y_avg])
    tri_center=[]
    for j in xrange(len(state_order)):
        # print 'j',j, 'len state order', len(state_order), 'len center points', len(Center_points)
        x_state = Center_points[int(state_order[j])-1][1]
        y_state = Center_points[int(state_order[j])-1][2]
        if state_order[j] == 12:
            y_state = 3
        if state_order[j] ==152:
            x_state = 9.5
        tri_center.append([x_state,y_state])
    x_ctr =[x[0] for x in tri_center]
    y_ctr = [x[1] for x in tri_center]
    if show_path:
        plt.plot(x_ctr, y_ctr)
    # plt.gca().invert_yaxis()
    plt.show()
    triangle_xy = []
    return x_ctr, y_ctr


# def read_triangles(gemotry_name='Proposal'):
def read_triangles(geometry_name):
    # global geometry_name
    ##################################################################
    with open(parent_dir+'geometry/{}.1.ele'.format(geometry_name)) as ele:
        tri_num = ele.readlines()
    ele.close()
    del tri_num[-1]

    with open(parent_dir+'/geometry/{}.1.node'.format(geometry_name)) as node:
            node_num = node.readlines()
    node.close()
    del node_num[-1]



    ##################################################################
    triangle_node_list = []
    nodes_xy_list = []
    for i in xrange(1, len(tri_num)):
        triangle_nodes = tri_num[i].strip().split(' ')
        triangle_nodes = filter(None, triangle_nodes)
        triangle_node_list.append(triangle_nodes)

    for i in xrange(1, len(node_num)):
        nodes_xy = node_num[i].strip().split(' ')
        nodes_xy = filter(None, nodes_xy)
        nodes_xy_list.append(nodes_xy)

    X=[]
    for i in xrange(len(triangle_node_list)):
        index = triangle_node_list[i]
        xx = []
        for i in range(1,4):
            x = nodes_xy_list[int(index[i])-1][1]
            y = nodes_xy_list[int(index[i])-1][2]
            xx.append([float(x),float(y)])
        X.append(xx)

    return X

if __name__ == "__main__":
    # grid_triangles = read_triangles('Proposal')
    grid_triangles = read_triangles(geometry_name)
    f = plt.figure(figsize=(7.5, 9), dpi=100)
    # plt.gca().invert_yaxis()
    # print grid_triangles
    for i, triangle in enumerate(grid_triangles):
        for j in range(3):
            # plt.gca().invert_yaxis()
            plt.plot([triangle[j][0], triangle[(j+1)%3][0]], [triangle[j][1], triangle[(j+1)%3][1]], '-r')
    plot_triangles(show_numbers=True)
    
    f.savefig("triangles.pdf", bbox_inches='tight')
    f.savefig("triangles.png", bbox_inches='tight')
    plt.show()