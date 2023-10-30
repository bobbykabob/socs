import math
import random
import time
from math import radians

import matplotlib.style as mplstyle
import matplotlib.pyplot as plt
import numpy

from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b
from full_robot import full_robot
from coppeliasim_zmqremoteapi_client import *
from sklearn.cluster import KMeans

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()


x = [-15, -15, 15, 15]
y = [-15, 15, -15, 15]
figure, ax = plt.subplots(figsize=(3.0, 3.0))
line1, = ax.plot(x, y, 'bo', markersize=1)
k_means_lines = []
line2, = ax.plot(0, 0, 'yo', markersize=1)
line3, = ax.plot(0, 0, 'go', markersize=1)
line4, = ax.plot(0, 0, 'ro', markersize=1)
plt.show(block=False)
mplstyle.use('fast')
figure.canvas.draw()
background = figure.canvas.copy_from_bbox(ax.bbox)
# empty arrays to cycle through
prev_time = int(round(time.time() * 1000))

robots = []
target_angle = radians(-45)
target_pos = [11, -11, target_angle]
constants = [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b]

for i in range(3):
    robots.append(full_robot(sim, '/robot[' + str(i) + ']'))
    robots[i].move_robot(target_pos, constants)
robots[1].move_robot(target_pos, [3, 0.5, ROBOT_TRACK_WIDTH, b])
robots[2].move_robot(target_pos, [10, 3, ROBOT_TRACK_WIDTH, b])

collated_points = []
while (t := sim.getSimulationTime()) < 1000:
    x = []
    y = []
    for i in range(3):
        a_x = []
        a_y = []
        a_x, a_y = robots[i].get_global_coordinates()

        if not len(a_x) == 0:
            x = numpy.concatenate([x, a_x])
            y = numpy.concatenate([y, a_y])


    line1.set_xdata(x)
    line1.set_ydata(y)

    # process the user information - cycle time & simulation times
    s = f'Simulation time: {t:.3f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
