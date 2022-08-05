import time
from math import cos
from math import pi
from math import sin
from math import tan

import numpy
import numpy as np
from sympy import Symbol
from sympy.solvers import solve

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
NUM_OF_ROBOTS = 15
oval_prev_constants = [None] * NUM_OF_ROBOTS
oval_robot_pos = [None] * NUM_OF_ROBOTS


def move_robot(index: int, left_power: float, right_power: float):
    """Moves the robot
    :param index: index of robot based on array robots
    :param left_power: left power
    :param right_power: right power
    :return: nothing
    """

    sim.setJointTargetVelocity(robots_left_joint[index], left_power)
    sim.setJointTargetVelocity(robots_right_joint[index], right_power)


def generate_new_position_line(index: int, endpoint_1, endpoint_2):
    """
    :param index: index of the robot based on array robots
    :param endpoint_1: xy position of first endpoint
    :param endpoint_2: xy position of second endpoint
    :return: list (x,y)
    """
    robot_point = (endpoint_2 - endpoint_1) * (index / (len(robots) - 1)) + endpoint_1
    return robot_point


def generate_new_position_oval(index: int, h, k, a, b, A):
    global oval_prev_constants
    """
    :param index: index of robot based on array robots
    :param h: x-shift
    :param k: y-shift
    :param a: x semi-axis
    :param b: y semi-axis
    :param A: angle measured from x axis
    :return: list (x,y)
    """
    current_constants = [h, k, a, b, A]

    if oval_prev_constants[index] == current_constants:
        return oval_robot_pos[index]
        # desired angle for current robot
    oval_prev_constants[index] = current_constants
    oval_angle = 2 * pi * index / NUM_OF_ROBOTS

    # creates symbolic symbols x, y
    x = Symbol('x')
    y = Symbol('y')

    # oval equation as provided
    oval_equation = (((x - h) * cos(A) + (y - k) * sin(A)) ** 2) / (a ** 2) + (
            ((x - h) * sin(A) - (y - k) * cos(A)) ** 2) / (b ** 2) - 1

    # accounts for limitations of tangent asymptotes
    if (index / NUM_OF_ROBOTS) == 0.25 or (index / NUM_OF_ROBOTS) == 0.75:
        line_equation = x - h
    else:
        # equation for line y = m(x-h) + k
        # m = tangent(oval_angle),
        # (h,k) represent the center of the ellipse
        line_equation = tan(oval_angle) * (x - h) + k - y

    # list of all solutions
    solutions = solve([oval_equation, line_equation], [x, y])

    # reformat solutions from tuple scmpy float -> list float
    sol1 = list(solutions[0])
    sol1 = [float(item) for item in sol1]

    sol2 = list(solutions[1])
    sol2 = [float(item) for item in sol2]
    ans = []
    # filter the outputs to choose the correct solution
    if index == 0:

        ans = sol2
    elif (index / NUM_OF_ROBOTS) < 0.5:
        # finding the solution that is higher than the y center
        if sol1[1] > k:
            ans = sol1
        else:
            ans = sol2
    elif (index / NUM_OF_ROBOTS) == 0.5:
        ans = sol1
    elif (index / NUM_OF_ROBOTS) > 0.5:
        # finding the solution lower than the y center
        if sol1[1] < k:
            ans = sol1
        else:
            ans = sol2
    oval_robot_pos[index] = ans
    return ans


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
        #move_robot(i, left_speed, right_speed)

        # get robot positions & print
        # positions = sim.getObjectPosition(robots[i], sim.handle_world)
        # orientation = sim.getObjectOrientation(robots[i], sim.handle_world)

        # create x y axis
        new_position = generate_new_position_oval(i, -2, -2, 5, 3, pi / 4)

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
