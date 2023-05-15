from math import sin
from math import tan
from math import cos
from math import pi

import numpy as np
from sympy import Symbol
from sympy.solvers import solve
from constants import NUM_OF_ROBOTS

oval_prev_constants = [None] * NUM_OF_ROBOTS
oval_robot_pos = [None] * NUM_OF_ROBOTS
oval_prev_constants_opening = [None] * NUM_OF_ROBOTS
oval_robot_pos_opening = [None] * NUM_OF_ROBOTS


def line(index: int, endpoint_1, endpoint_2):
    """
    :param index: index of the robot based on array robots
    :param endpoint_1: xy position of first endpoint
    :param endpoint_2: xy position of second endpoint
    :return: list (x,y)
    """
    robot_point = (endpoint_2 - endpoint_1) * (index / (NUM_OF_ROBOTS - 1)) + endpoint_1
    return robot_point


def oval(index: int, h, k, a, b, A):
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


def oval_opening(index: int, h, k, a, b, A, B):
    global oval_prev_constants_opening
    """
    :param index: index of robot based on array robots
    :param h: x-shift
    :param k: y-shift
    :param a: x semi-axis
    :param b: y semi-axis
    :param A: angle measured from x axis in radians
    :param B: angle measure of opening in radians
    :return: list (x,y)
    """
    current_constants = [h, k, a, b, A, B]

    if oval_prev_constants_opening[index] == current_constants:
        return oval_robot_pos_opening[index]
        # desired angle for current robot

    oval_prev_constants_opening[index] = current_constants

    oval_angle = (2 * pi - 2 * B) * index / (NUM_OF_ROBOTS - 1)

    # creates symbolic symbols x, y
    x = Symbol('x')
    y = Symbol('y')

    # oval equation as provided
    oval_equation = (((x - h) * cos(A) + (y - k) * sin(A)) ** 2) / (a ** 2) + (
            ((x - h) * sin(A) - (y - k) * cos(A)) ** 2) / (b ** 2) - 1

    # accounts for limitations of tangent asymptotes
    tangent_calculation = (oval_angle + A + B)

    if tangent_calculation == pi / 2 or tangent_calculation == 3 * pi / 2:
        line_equation = x - h
    else:
        # equation for line y = m(x-h) + k
        # m = tangent(oval_angle),
        # (h,k) represent the center of the ellipse
        line_equation = tan(oval_angle + A + B) * (x - h) + k - y

    # list of all solutions
    solutions = solve([oval_equation, line_equation], [x, y])

    # reformat solutions from tuple scmpy float -> list float
    sol1 = list(solutions[0])
    sol1 = [float(item) for item in sol1]

    sol2 = list(solutions[1])
    sol2 = [float(item) for item in sol2]

    ans = []
    # filter the outputs to choose the correct solution
    ratio = index / NUM_OF_ROBOTS
    sol1eq = -sol1[1] + tan(A) * (sol1[0] - h) + k
    if 0.00001 >= sol1eq >= -0.00001:
        prev_distance = oval_robot_pos_opening[index - 1]
        prev_distance_np = np.array([prev_distance[0], prev_distance[1]])
        sol1_np = np.array([sol1[0], sol1[1]])
        sol2_np = np.array([sol2[0], sol2[1]])
        print(prev_distance_np)
        print(sol1_np)
        sol1dist = np.linalg.norm(prev_distance_np - sol1_np)
        sol2dist = np.linalg.norm(prev_distance_np - sol2_np)
        if sol1dist > sol2dist:
            ans = sol2
        else:
            ans = sol1
    else:
        if ratio < 0.5:
            if sol1eq < 0:
                ans = sol1
            else:
                ans = sol2
        else:
            if sol1eq < 0:
                ans = sol2
            else:
                ans = sol1
    oval_robot_pos_opening[index] = ans
    print('robot' + str(index) + str(sol1) + str(sol2))
    print('chosen' + str(oval_robot_pos_opening[index]) + 'because sol1eq: ' + str(sol1eq))
    return ans
