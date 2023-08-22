import time
from math import radians

import matplotlib.style as mplstyle
import matplotlib.pyplot as plt
import numpy

from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b
from full_robot import full_robot
from obstacle import obstacle
from coppeliasim_zmqremoteapi_client import *
from sklearn.cluster import KMeans
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
line2, = ax.plot(0, 0, 'yo')
line3, = ax.plot(0, 0, 'go')
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

collated_points = None
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

    # calculate KMeans

    points = [x, y]
    print(points)
    points = numpy.rot90(points)
    if not numpy.count_nonzero(points) == numpy.size(points):
        if collated_points is None:
            collated_points = points
        else:
            collated_points = numpy.concatenate([collated_points, points], axis=0)
        print(collated_points)
        if not len(collated_points) < 100: #arbitary threshold
            kmeans = KMeans(n_clusters=3, random_state=42)
            kmeans.fit(collated_points)
            y_kmeans = kmeans.predict(collated_points)
            print("met threshold")
            print(kmeans.labels_)
            print(kmeans.cluster_centers_)
            filtered_label0 = collated_points[kmeans.labels_ == 0]
            line2.set_xdata(kmeans.cluster_centers_[:,0])
            line2.set_ydata(kmeans.cluster_centers_[:,1])
            line3.set_xdata(collated_points[:, 0])
            line3.set_ydata(collated_points[:, 1])
    ax.draw_artist(line3)

    ax.draw_artist(line1)
    ax.draw_artist(line2)
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




