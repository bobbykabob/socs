import time
from scipy.interpolate import CubicSpline
from zmqRemoteApi import RemoteAPIClient
from net import net
import generate_new_position
import matplotlib.pyplot as plt
import numpy as np
from math import pi
from constants import NUM_OF_ROBOTS
from shapely.geometry import LineString, Polygon
from descartes import PolygonPatch
from figures import SIZE, BLUE, GRAY, set_limits, plot_line

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
for n in range(num_of_nets):
    a_net = net(sim)
    nets.append(a_net)

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
dilated = line.buffer(0.1, cap_style=3)
patch1 = PolygonPatch(dilated, fc=BLUE, ec=BLUE, alpha=0.5, zorder=2)

# print verticies
print("Polygon vertices")
print(patch1.get_verts())

# display in 2d
ax.add_patch(patch1)
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
