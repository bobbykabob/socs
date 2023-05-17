import time
from math import radians

import matplotlib.style as mplstyle
import matplotlib.pyplot as plt
import numpy

from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b
from full_robot import full_robot
from obstacle import obstacle
from coppeliasim_zmqremoteapi_client import *

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

an_obstacle = obstacle(sim)
b_obstacle = obstacle(sim)
an_obstacle.set_obstacle_pos([5, -5])
b_obstacle.set_obstacle_pos([0,1])
b_obstacle.set_velocity([.01,0])

an_obstacle.set_velocity([0, .02])
x = [-10, -10, 10, 10]
y = [-10, 10, -10, 10]
figure, ax = plt.subplots(figsize=(5.0, 5.0))
line1, = ax.plot(x, y, 'bo')
plt.show(block=False)
mplstyle.use('fast')
figure.canvas.draw()
background = figure.canvas.copy_from_bbox(ax.bbox)
# empty arrays to cycle through
prev_time = int(round(time.time() * 1000))

robots = []
target_angle = radians(0)
target_pos = [10, 0, target_angle]
constants = [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b]
for i in range(2):

    robots.append(full_robot(sim, '/robot[' + str(i) + ']'))
    robots[i].move_robot(target_pos, constants)

while (t := sim.getSimulationTime()) < 40:
    x = []
    y = []
    for i in range(2):
        a_x = []
        a_y = []
        a_x, a_y = robots[i].get_global_coordinates()

        x = numpy.concatenate([x, a_x])
        y = numpy.concatenate([y, a_y])
    an_obstacle.update()
    b_obstacle.update()
    line1.set_xdata(x)
    line1.set_ydata(y)
    figure.canvas.restore_region(background)
    # figure.canvas.blit(ax.bbox)

    ax.draw_artist(line1)

    figure.canvas.update()

    figure.canvas.flush_events()
    # perform calculations with the acquired data; we create a polar graph from the data
    # x represents the theta value

    # process the user information - cycle time & simulation times
    s = f'Simulation time: {t:.3f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
