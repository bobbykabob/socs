import time
from math import radians

import cv2
import matplotlib.pyplot as plt
import numpy

from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b
from full_robot import full_robot
from zmqRemoteApi import RemoteAPIClient

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()
x = [-20, 0, 0, -20]
y = [0, 0, 20, 20]
figure, ax = plt.subplots(figsize=(5.0, 5.0))
line1, = ax.plot(x, y, 'bo')

# empty arrays to cycle through
prev_time = int(round(time.time() * 1000))

a_robot = full_robot(sim, '/robot')
for i in range(1):
    target_angle = radians(0)
    target_pos = [-5, 5, target_angle]
    constants = [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b]
    a_robot.move_robot(target_pos, constants)

a_robot.init_local_graph()

while (t := sim.getSimulationTime()) < 40:
    x = []
    y = []
    a_robot.generate_full_img()
    a_robot.calculate_polar_coordinates()
    a_robot.update_local_graph()

    a_x, a_y = a_robot.get_global_coordinates()

    x = numpy.concatenate([a_x])
    y = numpy.concatenate([a_y])

    line1.set_xdata(x)
    line1.set_ydata(y)
    figure.canvas.draw()
    plt.pause(0.0000000001)
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
