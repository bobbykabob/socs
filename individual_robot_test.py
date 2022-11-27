import time
from math import radians

import cv2

from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b
from full_robot import full_robot
from zmqRemoteApi import RemoteAPIClient

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through
prev_time = int(round(time.time() * 1000))

a_robot = full_robot(sim, '/robot[0]')
b_robot = full_robot(sim, '/robot[1]')
for i in range(1):
    target_angle = radians(0)
    target_pos = [-10, 10, target_angle]
    constants = [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b]
    a_robot.move_robot(target_pos, constants)

a_robot.init_local_graph()
b_robot.init_local_graph()
while (t := sim.getSimulationTime()) < 40:
    a_robot.generate_full_img()
    a_robot.show_depth_view()
    a_robot.calculate_polar_coordinates()
    a_robot.update_local_graph()
    b_robot.generate_full_img()
    b_robot.show_depth_view()
    b_robot.calculate_polar_coordinates()
    b_robot.update_local_graph()

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
