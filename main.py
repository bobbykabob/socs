import time

from math import pi
import generate_new_position
from constants import NUM_OF_ROBOTS
from zmqRemoteApi import RemoteAPIClient

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


def move_robot(index: int, left_power: float, right_power: float):
    """Moves the robot
    :param index: index of robot based on array robots
    :param left_power: left power
    :param right_power: right power
    :return: nothing
    """

    sim.setJointTargetVelocity(robots_left_joint[index], left_power)
    sim.setJointTargetVelocity(robots_right_joint[index], right_power)


# connects robots to arrays
for i in range(0, NUM_OF_ROBOTS):
    name = '/robot[' + str(i) + ']'
    robots.append(sim.getObject(name))
    robots_left_joint.append(sim.getObject(name + '/leftjoint'))
    robots_right_joint.append(sim.getObject(name + '/rightjoint'))

# loops through simulation in seconds
while (t := sim.getSimulationTime()) < 10:

    # looping through all robots
    for i in range(len(robots)):
        # move robot to a set speed
        left_speed = -10
        right_speed = 10
        # move_robot.py(i, left_speed, right_speed)

        # get robot positions & print
        # positions = sim.getObjectPosition(robots[i], sim.handle_world)
        # orientation = sim.getObjectOrientation(robots[i], sim.handle_world)

        # create x y axis

        # prev method call
        # new_position = generate_new_position.oval(i, -2, -2, 5, 3, pi / 4)

        # new method call
        new_position = generate_new_position.oval_opening(i, -2, -2, 5, 3, 0, 3 * pi / 8)

        # line_endpoint_1 = np.array([2, 2])
        # line_endpoint_2 = np.array([-2, -2])
        # new_position = generate_new_position_line(i, line_endpoint_1, line_endpoint_2).tolist()

        # create z axis
        new_position.append(0.1)
        # print(new_position)
        sim.setObjectPosition(robots[i], sim.handle_world, new_position)
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
