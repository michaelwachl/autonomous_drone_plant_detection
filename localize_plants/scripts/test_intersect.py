import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def perpendicular_distance(point, line_points, projectors):
    diff = line_points-point[:, 0]  # (a - p)
    temp = np.matmul(projectors, diff[:, :, np.newaxis])    # (I - nnT)(a - p)
    squared_distances = np.matmul(diff[:, np.newaxis], temp)    # (a -p)T(I - nnT)(a - p)
    distances = np.sqrt(squared_distances)
    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    print('mean: %s  std: %s' % (mean_distance, std_distance))
    return distances, mean_distance, std_distance


def intersect(P0, P1):
    """P0 and P1 are NxD arrays defining N lines.
    D is the dimension of the space. This function
    returns the least squares intersection of the N
    lines from the system given by eq. 13 in
    http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf.
    """
    # generate all line direction vectors
    n = (P1-P0)/np.linalg.norm(P1-P0, axis=1)[:, np.newaxis]    # normalized
    print("direction vectors: ", n, " shape: ", n.shape)
    # generate the array of all projectors
    # print('nt*n', np.matmul(n[:, np.newaxis], n[:, :, np.newaxis]))
    projs = np.eye(n.shape[1]) - n[:, :, np.newaxis]*n[:, np.newaxis]  # I - n*n.T
    print("Projection: ", projs.shape)
    print("---------------------------------------")
    # see fig. 1
    print("Projection: ", projs)

    # generate R matrix and q vector
    R = projs.sum(axis=0)
    print("R: ", R, '  shape:', R.shape)
    qtemp = projs @ P0[:, :, np.newaxis]
    print('points: ', P0[:, :, np.newaxis])
    print('test q: ', qtemp, '  shape:', qtemp.shape)
    q = (projs @ P0[:, :, np.newaxis]).sum(axis=0)
    print("q: ", R, '  shape:', q.shape)

    # solve the least squares problem for the
    # intersection point p: Rp = q
    p = np.linalg.lstsq(R, q, rcond=None)[0]

    perpendicular_distance(p, P0, projs)
    return p


n = 6
P0 = np.stack((np.array([5, 5, 5]) + 5*np.random.random(size=3) for i in range(n)))
print("start points: ", P0)
a = np.linspace(0, 2*np.pi, n) + np.random.random(size=n)*np.pi/5.0
P1 = np.array([5+5*np.sin(a), 5+5*np.cos(a), 5+5*np.sin(a)]).T
print("end points: ", P1)

# test intersection
"""
p_beg = np.zeros((2, 3), dtype=np.float32)
p_end = np.zeros((2, 3), dtype=np.float32)

p_beg[0, :] = [0.0, 0.0, 0.0]
p_beg[1, :] = [2.0, 0.0, 0.0]

p_end[0, :] = [0.0, 3.0, 0.0]
p_end[1, :] = [-2.0, 4.0, 0.0]

intersection = self.intersect_lines(p_beg, p_end)
print(intersection)
"""

# test 2 intersection
""""
p_beg = np.zeros((2, 3), dtype=np.float32)
p_end = np.zeros((2, 3), dtype=np.float32)

p_beg[0, :] = [0.0, 0.0, 0.0]
p_beg[1, :] = [2.0, 0.0, 0.0]

p_end[0, :] = [0.0, 3.0, 0.0]
p_end[1, :] = [-2.0, 4.0, 0.0]
"""

inters = intersect(P0, P1)
print('Intersection: ', inters)


# Plot figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Least-Squares Intersection')
plt.grid()

lines = []
for i in range(len(P0)):
    lines += plt.plot([P0[i, 0], P1[i, 0]], [P0[i, 1], P1[i, 1]], [P0[i, 2], P1[i, 2]], 'r-', label='3D Lines')

ls_inter = plt.plot(inters[0], inters[1], inters[2], 'bo', label='Closest Intersection')
legend_elements = [Line2D([0], [0], color='r', lw=2, label='3D Lines'),
                   Line2D([0], [0], marker='o', color='w', markerfacecolor='b', label='Closest Intersection',
                          markersize=9)]

plt.legend(handles=legend_elements, loc='upper right')

plt.show()
