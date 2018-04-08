
from __future__ import division

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import struct
import networkx as nx
from networkx import Graph
from math import sqrt
from sklearn.linear_model import RANSACRegressor, LinearRegression
import numpy as np

def argmin(list):
    '''
    return the index of the smallest item in the list
    '''
    return sorted([[val, idx] for idx, val in enumerate(list)])[0][1]

def argmax(list):
    '''
    return the index of the largest item in the list
    '''
    return sorted([[val, idx] for idx, val in enumerate(list)], reverse=True)[0][1]

def local_min(list, idx):
    '''
    return the index and the value of the local minimum following descent initialized at the specified index
    '''
    if idx==0 and list[0]>list[1]:
        idx += 1
    length = len(list)
    if idx==length-1 and list[length-2]<list[length-1]:
        idx -= 1
    while (not idx==0) and (not idx==length-1) and not(list[idx-1]>=list[idx] and list[idx+1]>=list[idx]):
        if list[idx]>list[idx-1]:
            idx -= 1
        else:
            idx += 1
    return idx, list[idx]

def select(list, idxs):
    '''
    idxs: a list of indices
    return a list that consists of only those in the original list with indices in idxs
    '''
    return [list[i] for i in idxs]

def select_each(list_of_list, idxs):
    '''
    apply select on each list in list_of_list
    return the new list_of_list with each list being selected by idxs
    '''
    return [select(l, idxs) for l in list_of_list]

def filter_val_idx(predicate, list):
    qualified = filter(lambda p: predicate(p[1]), enumerate(list))
    idxs = [i for (i, _) in qualified]
    vals = [v for (_, v) in qualified]
    return vals, idxs

def quantile(list, q):
    '''
    q: a float between 0 and 1 specifying the quantile
    return the element in the list at the (q*100)th quantile
    '''
    return list[int(len(list)*q)]

def f_addr_to_i(f):
    return struct.unpack('I', struct.pack('f', f))[0]
    
def i_addr_to_f(i):
    return struct.unpack('f', struct.pack('I', i))[0]

def rgb_to_pcl_float(r, g, b):
    i = r<<16 | g<<8 | b
    return i_addr_to_f(i)

def pcl_float_to_rgb(f):
    i = f_addr_to_i(f)
    r = i >> 16 & 0x0000ff
    g = i >> 8 & 0x0000ff
    b = i >> 0 & 0x0000ff
    return r,g,b

def render_3d_scatter(points, proportion=1, xlabel="x", ylabel="y", zlabel="z"):
    '''
    render 3d points. the points are represented by a list of lists (points) i.e. [[x1,y1,z1],[x2,y2,z2],...,[xn,yn,zn]]
    return the axis handler of the image (which, for example, can be used to change window limit by set_xlim, set_ylim, set_zlim)
    '''
    if len(points[0])==4:
        ax = render_3d_scatter_with_rgb(points, proportion, xlabel, ylabel, zlabel)
        return ax
    every = int(1/proportion)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter([x for i,(x,_,_) in enumerate(points) if i%every==0], 
        [y for i,(_,y,_) in enumerate(points) if i%every==0], zs=[z for i,(_,_,z) in enumerate(points) if i%every==0])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_aspect('equal')
    return ax

def render_3d_scatter_with_rgb(points, proportion=1, xlabel="x", ylabel="y", zlabel="z"):
    '''
    render 3d points. the points are represented by a list of lists (points with rgb) i.e. [[x1,y1,z1,rgb1],[x2,y2,z2,rgb2],...,[xn,yn,zn,rgbn]]
    return the axis handler of the image (which, for example, can be used to change window limit by set_xlim, set_ylim, set_zlim)
    '''
    every = int(1/proportion)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    rgb = [c for _,_,_,c in points]
    rgb_int = [struct.unpack('I', struct.pack('f', c))[0] for c in rgb]
    r = [c >> 16 & 0x0000ff for c in rgb_int]
    g = [c >> 8 & 0x0000ff for c in rgb_int]
    b = [c >> 0 & 0x0000ff for c in rgb_int]
    rgb = [[r[i]/255, g[i]/255, b[i]/255] for i in xrange(len(r))]
    x_selected = [x for i,(x,_,_,_) in enumerate(points) if i%every==0]
    y_selected = [y for i,(_,y,_,_) in enumerate(points) if i%every==0]
    z_selected = [z for i,(_,_,z,_) in enumerate(points) if i%every==0]
    rgb_selected = [c for i,c in enumerate(rgb) if i%every==0]
    ax.scatter(x_selected, y_selected, zs=z_selected, c=rgb_selected, linewidths=0)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_aspect('equal')
    return ax

def remove_plane(point_cloud, coord, plane, tolerance=0.03):
    '''
    point_cloud format: [[x1,y1,z1],[x2,y2,z2],...,[xn,yn,zn]]
    '''
    if plane=="xy":
        return [(x,y,z) for x,y,z in point_cloud if abs(z-coord)>tolerance]
    elif plane=="yz":
        return [(x,y,z) for x,y,z in point_cloud if abs(x-coord)>tolerance]
    elif plane=="xz":
        return [(x,y,z) for x,y,z in point_cloud if abs(y-coord)>tolerance]
    else:
        raise Exception("Unrecognized plane name")

def remove_plane_rgb(point_cloud, coord, plane, tolerance=0.03):
    '''
    point_cloud format: [[x1,y1,z1,rgb1],[x2,y2,z2,rgb2],...,[xn,yn,zn,rgbn]]
    rgb is a float packed from three numbers using PCL's encoding scheme
    '''
    if plane=="xy":
        return [(x,y,z,rgb) for x,y,z,rgb in point_cloud if abs(z-coord)>tolerance]
    elif plane=="yz":
        return [(x,y,z,rgb) for x,y,z,rgb in point_cloud if abs(x-coord)>tolerance]
    elif plane=="xz":
        return [(x,y,z,rgb) for x,y,z,rgb in point_cloud if abs(y-coord)>tolerance]
    else:
        raise Exception("Unrecognized plane name")

def remove_plane_idx(point_cloud, coord, plane, fit=True, tolerance=0.03):
    '''
    return the indices of the points on the plane to be removed
    '''
    if fit:
        if plane=="xy":
            points_to_be_removed = [p for p in point_cloud if abs(p[2]-coord)<=tolerance]
            a, b, c, d = fit_plane(points_to_be_removed)
            return [i for i,p in enumerate(point_cloud) if dist_point_plane(p, a, b, c, d)>tolerance]
        elif plane=="yz":
            points_to_be_removed = [p for p in point_cloud if abs(p[0]-coord)<=tolerance]
            a, b, c, d = fit_plane(points_to_be_removed)
            return [i for i,p in enumerate(point_cloud) if dist_point_plane(p, a, b, c, d)>tolerance]
        elif plane=="xz":
            points_to_be_removed = [p for p in point_cloud if abs(p[1]-coord)<=tolerance]
            a, b, c, d = fit_plane(points_to_be_removed)
            return [i for i,p in enumerate(point_cloud) if dist_point_plane(p, a, b, c, d)>tolerance]
        else:
            raise Exception("Unrecognized plane name")
    
    else:
        if plane=="xy":
            return [i for i,p in enumerate(point_cloud) if abs(p[2]-coord)>tolerance]
        elif plane=="yz":
            return [i for i,p in enumerate(point_cloud) if abs(p[0]-coord)>tolerance]
        elif plane=="xz":
            return [i for i,p in enumerate(point_cloud) if abs(p[1]-coord)>tolerance]
        else:
            raise Exception("Unrecognized plane name")

def read_pcd_file(f, data):
    if isinstance(f, basestring):
        f = open(f)
    pointsxyzrgb = []
    pointsxyz = []
    pointsxy = []
    pointsyz = []
    pointsxz = []
    all_x = []
    all_y = []
    all_z = []
    for l in f:
        try:
            float(l.strip().split()[0])
        except:
            continue
        x, y, z, rgb = map(float, l.strip().split())
        pointsxyzrgb.append([x,y,z,rgb])
        pointsxyz.append([x,y,z])
        pointsxy.append([x,y])
        pointsyz.append([y,z])
        pointsxz.append([x,z])
        all_x.append(x)
        all_y.append(y)
        all_z.append(z)
    ret = []
    for d in data:
        if d=='rgb' or d=='xyzrgb':
            ret.append(pointsxyzrgb)
        elif d=='xyz':
            ret.append(pointsxyz)
        elif d=='xy':
            ret.append(pointsxy)
        elif d=='yz':
            ret.append(pointsyz)
        elif d=='xz':
            ret.append(pointsxz)
        elif d=='x':
            ret.append(all_x)
        elif d=='y':
            ret.append(all_y)
        elif d=='z':
            ret.append(all_z)
        else:
            raise Exception("Unrecgonized data format"+str(d))
    return ret

def write_pcd_file(point_cloud, f):
    if isinstance(f, basestring):
        f = open(f, 'w')
    tot_num = len(point_cloud)
    has_rgb = len(point_cloud[0])==4
    if has_rgb:
        f.write("VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        f.write("WIDTH "+str(tot_num)+"\n")
        f.write("HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS "+str(tot_num)+"\n")
        f.write("DATA ascii\n")
    else:
        f.write("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write("WIDTH "+str(tot_num)+"\n")
        f.write("HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS "+str(tot_num)+"\n")
        f.write("DATA ascii\n")
    for p in point_cloud:
        f.write(" ".join(map(str,p))+"\n")
    f.close()

def euclidian_2d_dist(p1, p2):
    return sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )

def euclidian_3d_dist(p1, p2):
    return sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2 )

def make_graph(points, neighbor_max_dist=0.01):
    graph = Graph()
    graph.add_nodes_from(range(len(points)))
    for i in xrange(len(points)):
        for j in xrange(i+1, len(points)):
            if euclidian_3d_dist(points[i], points[j])<neighbor_max_dist:
                graph.add_edge(i,j)
    return graph

def get_largest_cc(points, neighbor_max_dist=0.01, eligible_condition=None):
    graph = make_graph(points, neighbor_max_dist)
    if eligible_condition is None:
        idxs = sorted(nx.connected_components(graph), key=len, reverse=True)[0]
        return select(points, idxs)
    else:
        ccs = list(nx.connected_components(graph))
        max_count = None
        max_cc_idx = None
        for i in xrange(len(ccs)):
            idxs = ccs[i]
            tmp_points = select(points, idxs)
            eligible_count = len(filter(eligible_condition, tmp_points))
            if max_count is None or eligible_count>max_count:
                max_count = eligible_count
                max_cc_idx = i
        return select(points, ccs[max_cc_idx])
    
def fit_plane(points):
    '''
    fit a plane through a list of 3d points and return a, b, c, d that represents the plane as ax+by+cz+d=0
    '''
    X = [[p[0], p[1]] for p in points]
    X = np.matrix(X)
    y = [p[2] for p in points]
    model = RANSACRegressor(LinearRegression())
    model.fit(X, y)
    d = list(model.estimator_.intercept_.flatten())[0]
    a, b = list(model.estimator_.coef_.flatten())
    c = -1
    return a, b, c, d

def dist_point_plane(p, a, b, c, d):
    '''
    distance between point p: (p[0], p[1], p[2]) and plane ax+by+cz+d=0
    '''
    return abs(a*p[0]+b*p[1]+c*p[2]+d)/sqrt(a**2+b**2+c**2)
    
def normalize(vec):
    '''
    normalize vector
    '''
    return list(np.divide(vec, float(euclidian_3d_dist(vec, [0,0,0]))).flatten())

def project(vec1, vec2):
    '''
    project vec1 down to vec2 and return the projected vector
    '''
    assert len(vec1)==3 and len(vec2)==3
    coef = np.dot(vec1, vec2)/np.dot(vec2, vec2)
    return list(np.multiply(vec2, coef).flatten())

def project_to_plane(point, a, b, c):
    '''
    project point down to plane ax+by+cz=0
    return the coordinate of the point of projection in the plane
    '''
    assert 3<=len(point)<=4
    point_3d = [point[0], point[1], point[2]]
    n = (a, b, c)
    projected = project(point_3d, n)
    if len(point)==3:
        return [point[0]-projected[0], point[1]-projected[1], point[2]-projected[2]]
    else:
        return [point[0]-projected[0], point[1]-projected[1], point[2]-projected[2], point[3]]

def get_basis_for_plane(a, b, c):
    '''
    find a set of basis vectors for the plane ax+by+cz=0
    '''
    basis1 = normalize(project_to_plane([1,0,0], a, b, c))
    attempt = [0, 1, 0]
    attempt_on_plane = project_to_plane(attempt, a, b, c)
    attempt_on_plane_on_basis1 = project(attempt_on_plane, basis1)
    basis2 = np.array(attempt_on_plane)-np.array(attempt_on_plane_on_basis1)
    basis2 = normalize(list(basis2.flatten()))
    return basis1, basis2

def basis_decompose(vec, basis1, basis2):
    '''
    decompose vec into a*basis1+b*basis2 and return (a, b)
    basis1 and basis2 must be perpendicular to each other
    '''
    assert np.dot(basis1, basis2)<1e-6
    assert 3<=len(vec)<=4
    if len(vec)==4:
        v = [vec[0], vec[1], vec[2]]
    else:
        v = vec
    a = np.dot(v, basis1) / np.dot(basis1, basis1)
    b = np.dot(v, basis2) / np.dot(basis2, basis2)
    return a, b

