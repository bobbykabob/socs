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
    new_position = generate_new_position.oval_opening(n, 15, 10, 2, 5, pi / 4, 3 * pi / 8)

    x_position.append(new_position[0])
    y_position.append(new_position[1])
    u.append(n)

xy_position = np.c_[x_position, y_position]

print("xy_position" + str(xy_position))
print("u" + str(u))
spline = CubicSpline(u, xy_position, extrapolate=False)

xs = (num_of_nets + 1) * np.linspace(0, 1, 5000)
spline_x_position = spline(xs)[:, 0]
spline_y_position = spline(xs)[:, 1]

# convert from a scipy-specific array into a shapely-specific array
line = []
for n in range(len(spline_x_position)):
    line.append((spline_x_position[n], spline_y_position[n]))

# shapely buffer code

line = LineString(line)
dilated = line.buffer(0.01, resolution=1000, cap_style=1, join_style=2)

# print vertices
patch_vertices_x, patch_vertices_y = dilated.exterior.coords.xy

vertices_3d = []
point_cloud = sim.createPointCloud(10000, 5, 16, 1)

for n in range(int(len(patch_vertices_x))):
    current_index = n % len(patch_vertices_x)
    z_position = 0.1

    vertices_3d.append(patch_vertices_x[current_index])
    vertices_3d.append(patch_vertices_y[current_index])
    vertices_3d.append(z_position)
    z_position = 0.3
    vertices_3d.append(patch_vertices_x[current_index])
    vertices_3d.append(patch_vertices_y[current_index])
    vertices_3d.append(z_position)
sim.insertPointsIntoPointCloud(point_cloud, 0, vertices_3d)

while (t := sim.getSimulationTime()) < 1000:
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
