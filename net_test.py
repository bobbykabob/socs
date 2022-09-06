import time
from math import pi

import matplotlib.pyplot as plt
import numpy as np
from descartes import PolygonPatch
from matplotlib import animation
from scipy.interpolate import CubicSpline
from shapely.geometry import LineString

import generate_new_position
from constants import NUM_OF_ROBOTS
from figures import BLUE
from net import net
from zmqRemoteApi import RemoteAPIClient

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through


prev_time = int(round(time.time() * 1000))

# loops through simulation in seconds

nets = []
num_of_nets = NUM_OF_ROBOTS
# net(sim)

x_position = []
y_position = []
u = []
for n in range(num_of_nets):
    new_position = generate_new_position.oval_opening(n, 0, 0, 2, 2, -pi / 3, 1 * pi / 8)

    x_position.append(new_position[0])
    y_position.append(new_position[1])
    u.append(n)

xy_position = np.c_[x_position, y_position]

print("xy_position" + str(xy_position))
print("u" + str(u))
spline = CubicSpline(u, xy_position, extrapolate=False)
fig, ax = plt.subplots(figsize=(6.5, 4))

xs = (num_of_nets + 1) * np.linspace(0, 1, 5000)
ax.plot(x_position, y_position, 'o', label='data')
spline_x_position = spline(xs)[:, 0]
spline_y_position = spline(xs)[:, 1]

ax.plot(spline_x_position, spline_y_position, label='spline')

# convert from a scipy-specific array into a shapely-specific array
line = []
for n in range(len(spline_x_position)):
    line.append((spline_x_position[n], spline_y_position[n]))

# shapely buffer code

line = LineString(line)
dilated = line.buffer(0.4, resolution=1, cap_style=1)

patch1 = PolygonPatch(dilated, fc=BLUE, ec=BLUE, alpha=0.1, zorder=2)

# print vertices
print("Polygon vertices")
patch_vertices_x, patch_vertices_y = dilated.exterior.coords.xy

print(len(patch_vertices_x))
print(patch_vertices_x)

vertices_3d = []
list_of_indices = []
for n in range(int(len(patch_vertices_x) * 0.205)):
    current_index = n % len(patch_vertices_x)
    z_position = 0.1
    if n >= (len(patch_vertices_x) - 1):
        z_position = 0.1
    vertices_3d.append(patch_vertices_x[current_index])
    vertices_3d.append(patch_vertices_y[current_index])
    vertices_3d.append(z_position)
    list_of_indices.append(n)
print(vertices_3d)
print(len(vertices_3d))
print(list_of_indices)
print(len(list_of_indices))
net_object = sim.createMeshShape(2, 20.0 * 3.1415 / 180.0, vertices_3d, list_of_indices)
vertices, indices = sim.importMesh(0, '/Users/harris/Desktop/CoppeliaSim/net.stl', 1, 0, 0.03)

print(vertices)
print(len(vertices[0]))
print(len(patch_vertices_x))
test_vertices_x = []
test_vertices_y = []

fig1 = plt.figure(figsize=(5, 5))
for n in range(int(len(patch_vertices_x) * 0.205)):
    print(str(patch_vertices_x[n]) + ", " + str(patch_vertices_y[n]))
    test_vertices_x.append(patch_vertices_x[n])
    test_vertices_y.append(patch_vertices_y[n])


def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

l, = plt.plot([], [], 'r-')
plt.xlim([-5, 5])
plt.ylim([-5, 5])
plt.xlabel('x')
plt.title('test')

print(len(test_vertices_x))
print(len(test_vertices_y))
data = np.array([test_vertices_x, test_vertices_y])

line_ani = animation.FuncAnimation(fig1, update_line, 2097, fargs=(data, l),
                                   interval=5, blit=True)

old_vert_x = []
old_vert_y = []
modified_vertices = vertices[0]
vert_x = []
vert_y = []
for n in range(int(len(vertices_3d))):
    if (n % 3) == 0:
        vert_x.append(vertices_3d[n])
    elif (n % 3) == 1:
        vert_y.append(vertices_3d[n])
for n in range(int(len(modified_vertices) / 2)):
    if (n % 3) == 0:
        old_vert_x.append(modified_vertices[n])
    elif (n % 3) == 1:
        old_vert_y.append(modified_vertices[n])
net_handle = sim.createMeshShape(0, 20.0 * 3.1415 / 180.0, modified_vertices, indices[0])
# ax.plot(old_vert_x, old_vert_y)
ax.plot(vert_x, vert_y)
# display in 2d
# ax.add_patch(patch1)
# ax.plot(patch_vertices_x, patch_vertices_y)
ax.axes.set_aspect('equal')
plt.show()

while (t := sim.getSimulationTime()) < 50:
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
