import time

from math import pi
import generate_new_position
from constants import NUM_OF_ROBOTS
from zmqRemoteApi import RemoteAPIClient
import move_robot
# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through
robots = []
robots_left_joint = []
robots_right_joint = []
prev_time = int(round(time.time() * 1000))

# connects robots to arrays
for i in range(0, NUM_OF_ROBOTS):
    name = '/robot[' + str(i) + ']'
    robots.append(sim.getObject(name))
    robots_left_joint.append(sim.getObject(name + '/leftjoint'))
    robots_right_joint.append(sim.getObject(name + '/rightjoint'))

# loops through simulation in seconds
while (t := sim.getSimulationTime()) < 50:

    # looping through all robots
    for i in range(len(robots)):
        # move robot to a set speed

        # move_robot.py(i, left_speed, right_speed)

        # get robot positions
        positions = sim.getObjectPosition(robots[i], sim.handle_world)
        orientation = sim.getObjectOrientation(robots[i], sim.handle_world)

        # create x y axis

        # prev method call
        # new_position = generate_new_position.oval(i, -2, -2, 5, 3, pi / 4)

        # new method call
        new_position = generate_new_position.oval_opening(i, -2, -2, 5, 3, 0, 3 * pi / 8)

        left_power, right_power = move_robot.move_robot([new_position[0], new_position[1], 0],
                                                        [positions[0], positions[1], orientation[2]])
        sim.setJointTargetVelocity(robots_left_joint[i], left_power)
        sim.setJointTargetVelocity(robots_right_joint[i], right_power)

        # create z axis
        new_position.append(0.1)
        # print(new_position)
        #sim.setObjectPosition(robots[i], sim.handle_world, new_position)
        # convert from radians to degrees
        # orientation_degree = numpy.degrees(orientation)
        # print("robot" + str(i) + ": ")
        # print("x: " + str(positions[0]))
        # print("y: " + str(positions[1]))
        # print("z: " + str(positions[2]))
        # print("theta: " + str(orientation_degree))

    # get simulation times and print
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
