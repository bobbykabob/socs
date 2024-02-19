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
figure, axs = plt.subplots(figsize=(15.0, 3.0), ncols=5)

# plots the outside points that define the boundaries
[an_ax.plot(x ,y, 'wo', markersize=1) for an_ax in axs]

# lines is a matrix with...
# .. line1 line2 line3 line4 ... line_i
# ax0  ..   ..    ..    ..    ... ..
# ax1  ..   ..    ..    ..    ... ..
# ax2  ..   ..    ..    ..    ... ..
# ax3  ..   ..    ..    ..    ... ..
# ..   ..   ..    ..    ..    ... ..
# ..   ..   ..    ..    ..    ... ..
# ax_j ..   ..    ..    ..    ... ..

lines = numpy.array()

for an_ax in axs:
    lines.append([an_ax.plot])
plt.show(block=False)
mplstyle.use('fast')
figure.canvas.draw()
backgrounds = [figure.canvas.copy_from_bbox(an_ax.bbox) for an_ax in axs]


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

    [figure.canvas.restore_region(a_background) for a_background in backgrounds]
    # axs[0].draw_artist(axs[0].plot(x, y, 'o', color='b', markersize=1)[0])

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
                rand_color = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]
                # a_k_mean_line = axs[1].plot(line_segment_x, line_segment_y, '-', color=rand_color, markersize=1)[0]

                # axs[1].draw_artist(a_k_mean_line)

                # calculating mean absolute deviation (MAD)
                mean_absolute_deviation = sum(abs((f_x - y))) / len(filtered_label)
                line_segment = [mean_absolute_deviation, float(alpha[0]), float(alpha[1]), x_tail, y_tail, x_head, y_head]

                if line_segment_list is numpy.nan:
                    line_segment_list = numpy.array([line_segment])
                else:
                    line_segment_list = numpy.concatenate((line_segment_list, [line_segment]), axis=0)

            # we are sorting the line segment list from the smallest MAD to the biggest; hence, the variable
            # MAD_index contains the order
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

            kmeans.labels_[unclustered_points_index] = -1
            # just unassigned all the points that have bad clusters; time to reassign
            # ------------------2.C---------------------------------------------------

            point_to_line_segment = []


            # all max dist to line is a matrix formatted as...
            # .. P0 P1 P2 P3 ... Pi
            # L0 [                 ]
            # L1 [                 ]
            # L2 [                 ]
            # L3 [                 ]
            # .. [                 ]
            # .. [                 ]
            # .. [                 ]
            # Lj [                 ]
            unclustered_points = modified_points[unclustered_points_index]
            # calculate d(Pi, Lj)
            # TODO: Optimize the for loop to calculate everything in the matrix at once.. ?


            for a_line_segment in line_segment_list:
                m = a_line_segment[1]
                b = a_line_segment[2]

                # distance = | - m * x_0 + y_0 - b |/ sqrt(m^2+1)
                distance = abs(- m * unclustered_points[:, 0] + unclustered_points[:, 1] - b) / math.sqrt(
                    math.pow(m, 2) + 1)
                point_to_line_segment.append(distance)

            # near_end
            max_point_to_line = numpy.amax(point_to_line_segment)
            near_end = 2 * max_point_to_line

            # near line
            max_individual_point_to_line = numpy.max(point_to_line_segment, axis=1)
            near_line = numpy.sum(max_individual_point_to_line) / len(line_segment_list)

            # mean dist to line
            mean_dist_to_line = numpy.sum(point_to_line_segment) / (len(point_to_line_segment) * len(point_to_line_segment[0]))

            # TODO: visualize the line segments (for my own sanity!)

            # just a reminder that a line segment is written as: [MAD, m, b, tail-x, tail-y, head-x, head-y]
            for a_line_segment in line_segment_list:
                x = numpy.arange(a_line_segment[3], a_line_segment[5], .00001)
                y = x * a_line_segment[1] + a_line_segment[2]
                rand_color = [random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]
                a_k_mean_line = axs[2].plot(x,y, '-', color= rand_color, markersize=1)[0]
                axs[2].draw_artist(a_k_mean_line)
            # Do untill point allocation is possible or max. iteration reached
            # For each unclustered point Pi
            for an_unclustered_point_index_index in range(0, len(unclustered_points_index)):

                # I was definitely a bit mentally ill when I wrote this code

                # an_unlustered_point_index_index ranges from 0 -> length of unclustered_points_index and goes up
                # incrementally by the for loop an_unclustered_point_index is the index of unclustered points
                # relative to the modified_points
                # this is needed because the indices of a few calculations are a bit weird
                an_unclustered_point_index = unclustered_points_index[an_unclustered_point_index_index]

                # an_unclustered_point is the x,y coordinate of these points
                an_unclustered_point = modified_points[an_unclustered_point_index]

                # Find lines j which {d(Pi, end_points(Lj)) <= near_end OR Pi is between end_points(Lj)}

                # TODO: Optimize the for loop to calculate everything in the matrix at once.. ?
                # distance from the unclustered point to the endpoints of all lines
                # ......... L0 L1 L2 L3 .. Lj
                # distance [                ]

                # finds d(Pi, end_points(Lj))
                distance_unclustered_point_endpoint_lines = []
                for a_line_segment in line_segment_list:
                    a_line_segment_tail = numpy.array(a_line_segment[3:5])
                    a_line_segment_head = numpy.array(a_line_segment[5:7])
                    distance = math.dist(an_unclustered_point, a_line_segment_tail) + math.dist(an_unclustered_point, a_line_segment_head)
                    distance_unclustered_point_endpoint_lines.append(distance)

                # finds the lines j where d(Pi, end_points(Lj)) <= near_end
                line_segment_index = numpy.where(distance_unclustered_point_endpoint_lines < near_end)[0]

                # TODO: find lines j where Pi is between end_points(Lj)
                # find math.dist(end_points(Lj)) == math.dist(end_points(Lj, an_unclustered_point)

                # we need to calculate minj{d(Pi, Lj)}
                # these are the line segments that meet the above criterion

                # along with the line segments considered, we also look at the d(Pi, Lj) through the already-calculated point-to-line segment
                point_to_line_segments_considered = numpy.array(point_to_line_segment)[line_segment_index, an_unclustered_point_index_index]



                # If minj{d(Pi, Lj)} <= near_line
                if numpy.min(point_to_line_segments_considered) <= near_line:
                    kmeans.labels_[an_unclustered_point_index] = line_segment_index[numpy.argmin(point_to_line_segments_considered)]

                    # TODO: recalculate line-segment parameters and end-points
                    # this includes
                    # - calculating 2.A again
                    # - potiently recalculating point_to_line_segment again
                    # - or we can ignore recalculating point_to_line_segment again

            # TODO: graph out the new kmeans clusters and information by recalculating everything!



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
