import math
import random
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
b_obstacle.set_obstacle_pos([0, 1])
b_obstacle.set_velocity([.01, 0])

an_obstacle.set_velocity([0, .02])
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
for i in range(4):
    robots.append(full_robot(sim, '/robot[' + str(i) + ']'))
    robots[i].move_robot(target_pos, constants)

collated_points = []
while (t := sim.getSimulationTime()) < 1000:
    x = []
    y = []
    for i in range(4):
        a_x = []
        a_y = []
        a_x, a_y = robots[i].get_global_coordinates()

        if not len(a_x) == 0:
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
    # print(points)
    points = numpy.rot90(points)
    if not len(points) == 0:
        if len(collated_points) == 0:
            collated_points = points
        else:
            collated_points = numpy.concatenate([collated_points, points], axis=0)
        # print(collated_points)
        if len(collated_points) > 100:  # arbitary threshold
            num_clusters = int(len(collated_points) / 10)
            # num_clusters = 4
            for line_index in range(len(k_means_lines), num_clusters):
                rand_color = [random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)]
                k_means_lines.append(ax.plot(0,0, '-', color= rand_color, markersize=1)[0])
            modified_index = numpy.random.choice(range(len(collated_points)), size= num_clusters*10)
            modified_points = collated_points[modified_index, :]

            # perform Kmeans calculations

            kmeans = KMeans(n_clusters=num_clusters, random_state=42, n_init=8)
            kmeans.fit(modified_points)
            y_kmeans = kmeans.predict(modified_points)

            filtered_label0 = modified_points[kmeans.labels_ == 0]

            # 3.A Point Clustering
            # formatted as [MAD, m, b, tail-x, tail-y, head-x, head-y]
            line_segment_list = numpy.nan

            for line_index in range(num_clusters):

                # A. Point Clustering
                filtered_label = modified_points[kmeans.labels_ == line_index]
                # assemble matrix A
                x = filtered_label[:,0]
                A = numpy.vstack([x, numpy.ones(len(x))]).T

                # turn y into a column vector
                y = filtered_label[:,1]
                y_column_vec = y[:, numpy.newaxis]

                # calculating line segment using least-mean-squared error
                alpha = numpy.linalg.lstsq(A, y_column_vec, rcond=None)[0]

                f_x = x * alpha[0] + alpha[1]


                # y = mx + b
                # to find inverse,
                # x = my + b
                # y = (x-b) /m
                # f^-1(y) = (x - b) / m
                f_inverse_y = (y - alpha[1]) / alpha[0]

                # implementing Most General Hypothesis (MGH) and Most Specific Hypothesis (MSH)

                x_tail = 0
                y_tail = 0
                x_head = 0
                y_head = 0
                # MGH
                using_mgh = True

                if using_mgh:
                    x_tail = min(min(x), min(f_inverse_y))

                    if alpha[0] > 0:
                        y_tail = min(min(y), min(f_x))
                    else:
                        y_tail = max(max(y), max(f_x))

                    x_head = max(max(x), max(f_inverse_y))

                    if alpha[0] > 0:
                        y_head = max(max(y), max(f_x))
                    else:
                        y_head = min(min(y), min(f_x))

                # MSH

                if not using_mgh:
                    x_tail = min(x)
                    y_tail = x_tail * alpha[0] + alpha[1]

                    x_head = max(x)
                    y_head = x_head * alpha[0] + alpha[1]

                line_segment_x = numpy.arange(x_tail, x_head, .00001)
                # print(line_segment_x)
                line_segment_y = line_segment_x * alpha[0] + alpha[1]
                k_means_lines[line_index].set_xdata(line_segment_x)
                k_means_lines[line_index].set_ydata(line_segment_y)
                ax.draw_artist(k_means_lines[line_index])

                # calculating mean absolute deviation (MAD)
                mean_absolute_deviation = sum(abs((f_x - y))) / len(filtered_label)
                line_segment = [mean_absolute_deviation, float(alpha[0]), float(alpha[1]), x_tail, y_tail, x_head, y_head]

                if line_segment_list is numpy.nan:
                    line_segment_list = numpy.array([line_segment])
                else:
                    line_segment_list = numpy.concatenate((line_segment_list, [line_segment]), axis=0)

            # we are sorting the line segment list from the smallest MAD to the biggest; hence, the variable MAD_index contains the order
            line_segment_list = numpy.array(line_segment_list)
            MAD_index = numpy.argsort(line_segment_list[:,0])

            # we are calculating the number of clusters that are kept, at a ratio of 40% kept and 60% discarded
            num_clusters_kept = int(2 * len(MAD_index) / 5)
            MAD_index_kept = numpy.where(MAD_index < num_clusters_kept)[0]

            # we are now removing all the unnecessary clusters from the list of lines
            line_segment_list = line_segment_list[MAD_index_kept]

            # finds all the indices of the clustered points
            clustered_points_index = numpy.array([])
            for x in MAD_index_kept:

                clustered_points_index = numpy.append(clustered_points_index, numpy.where(kmeans.labels_ == x))
            clustered_points_index = clustered_points_index.astype(int)

            # creates an index of all points
            all_points_index = numpy.arange(0, len(kmeans.labels_), 1)

            # deletes the indices of points to keep to get the points to delete
            unclustered_points_index = numpy.delete(all_points_index, clustered_points_index)

            # just unassigned all the points that have bad clusters; time to reassign
            # 2.C

            all_max_dist_to_line = []
            unclustered_points = modified_points[unclustered_points_index]



            # calculate near_end, which is 2 * max j (max i d(Pi, Lj))


            for an_unclustered_point in unclustered_points:
                # Find lines j which d(P_i, end-points(Lj)) <= near_end OR P_i is betwenn end-points Lj)
                for a_line_segment in line_segment_list:
                    m = a_line_segment[1]
                    b = a_line_segment[2]

                    # distance = | - m * x_0 + y_0 - b |/ sqrt(m^2+1)
                    distance = abs(- m * unclustered_points[:, 0] + unclustered_points[:, 1] - b) / math.sqrt(math.pow(m, 2) + 1)
                    all_max_dist_to_line.append(distance)
            line_m_b = line_segment_list[:, 1:2]


            max_dist_to_line = numpy.amax(all_max_dist_to_line)


            line2.set_xdata(kmeans.cluster_centers_[:, 0])
            line2.set_ydata(kmeans.cluster_centers_[:, 1])
            # line3.set_xdata(collated_points[:, 0])
            # line3.set_ydata(collated_points[:, 1])
    # ax.draw_artist(line3)

    # ax.draw_artist(line1)
    # ax.draw_artist(line2)
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
