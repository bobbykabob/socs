import random
import time

from math import pi
import generate_new_position
from constants import NUM_OF_ROBOTS
from full_robot import full_robot
from obstacle import obstacle
from zmqRemoteApi import RemoteAPIClient
from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b

import net
# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

an_obstacle = obstacle(sim)

current_pos = [0, 0, 0.125]
an_obstacle.set_obstacle_pos(current_pos)


# empty arrays to cycle through
robots = []
script_handle = []
prev_time = int(round(time.time() * 1000))

# connects robots to arrays

for i in range(NUM_OF_ROBOTS):
    robots.append(full_robot(sim, '/robot[' + str(i) + ']'))
# looping through all robots
constants = [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b]

for i in range(len(robots)):
    new_position = generate_new_position.oval_opening(i, -2, -2, 5, 3, 0, pi/8)

    robots[i].move_robot(target_pos, constants)
sim.startSimulation()
x_rand = random.randint(-5, 5) / 100
y_rand = random.randint(-5, 5) / 100
while (t := sim.getSimulationTime()) < 100:
    # get simulation times and print
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    if (sim.getSimulationTime() == 10):
        x_rand = random.randint(-5, 5) / 100
        y_rand = random.randint(-5, 5) / 100
    current_pos = [current_pos[0] + x_rand, current_pos[1] + y_rand, current_pos[2]]
    an_obstacle.set_obstacle_pos(current_pos)

    x = []
    y = []
    for i in range(10):
        a_x = []
        a_y = []
        a_x, a_y = robots[i].get_global_coordinates()

        x = numpy.concatenate([x, a_x])
        y = numpy.concatenate([y, a_y])

    line1.set_xdata(x)
    line1.set_ydata(y)
    figure.canvas.restore_region(background)
    # figure.canvas.blit(ax.bbox)

    ax.draw_artist(line1)

    figure.canvas.update()

    figure.canvas.flush_events()
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
