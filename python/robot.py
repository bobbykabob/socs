# python
# code on each individual robot
from math import cos, sin, sqrt, atan, pi
from typing import List
import numpy
import cv2
import matplotlib
import matplotlib.style as mplstyle
import matplotlib.pyplot as plt

# import sys
# import platform

matplotlib.use('TkAgg')
left_joint = None
right_joint = None
robot = None
q_target = []
constants = []
orientation = []
positions = []
has_run = False
vision_sensors = []
circle_x = []
circle_y = []
lines = []
p1 = None
p2 = None
p3 = None
figure = None
background = None
ax = []
velo_line = None
current_velocity = [0, 0]
difference_rays = []


def update_actuation(l_q_target: List[float], l_constants: List[float]):
    global q_target, constants, has_run
    # updating the global variables
    q_target = l_q_target
    constants = l_constants
    has_run = True
    pass


def sysCall_init():
    # initiate robot

    global left_joint, right_joint, robot, vision_sensors
    # print (sys.version)
    robot = sim.getObject('.')

    left_joint = sim.getObject('./leftjoint')
    right_joint = sim.getObject('./rightjoint')
    vision_sensors.append(sim.getObject('./Cylinder/Vision_sensor[0]'))
    vision_sensors.append(sim.getObject('./Cylinder/Vision_sensor[1]'))
    vision_sensors.append(sim.getObject('./Cylinder/Vision_sensor[2]'))
    vision_sensors.append(sim.getObject('./Cylinder/Vision_sensor[3]'))
    init_local_graph()


def sysCall_actuation():
    # calculate and move robot
    # constants order:
    # c1, c2, B, b
    global q_target, constants, orientation, positions, has_run, current_velocity
    if not has_run or len(orientation) == 0:
        return
    c_1 = constants[0]
    c_2 = constants[1]
    B = constants[2]
    b = constants[3]
    angle = orientation[2]
    q_current = [positions[0], positions[1], angle]
    x = q_current[0] * cos(q_target[2]) + q_current[1] * sin(q_target[2]) - \
        q_target[0] * cos(q_target[2]) - q_target[1] * sin(q_target[2])

    y = -q_current[0] * sin(q_target[2]) + q_current[1] * cos(q_target[2]) + \
        q_target[0] * sin(q_target[2]) - q_target[1] * cos(q_target[2])

    # phi
    phi_r = q_current[2] - q_target[2]

    # equation 3
    d = sqrt(x ** 2 + y ** 2)

    s_x = sign(x)

    # eta
    # -pi/ 2 <= eta <= pi/2
    eta = s_x * atan(y / abs(x))

    # psi
    psi_1 = cos(eta - phi_r)
    psi_2 = sin(eta - phi_r)

    # xi
    # 1 < b < 2
    phi_a = (2.0 / b) * eta

    xi = phi_a - phi_r

    # translational velocity V_R
    v_r = - c_1 * s_x * (psi_1 / ((psi_1 ** 2) + xi ** 2)) * d
    # rotational velocity
    # ? -> omega
    omega_r = c_2 * xi + (c_1 / ((psi_1 ** 2) + xi ** 2)) * ((2.0 / b) * psi_1 * psi_2 + xi * (d ** 2))
    current_velocity = [v_r, omega_r]
    # velocity of right wheel
    # positive omega -> turn left
    # negative omega -> turn right
    v_right = v_r + (B * omega_r) / 2

    # velocity of left wheel
    v_left = v_r - (B * omega_r) / 2

    sim.setJointTargetVelocity(left_joint, v_left)
    sim.setJointTargetVelocity(right_joint, v_right)
    pass


def sysCall_sensing():
    # update robot positions
    global positions, orientation, vision_sensors
    positions = sim.getObjectPosition(robot, sim.handle_world)
    orientation = sim.getObjectOrientation(robot, sim.handle_world)
    full_img = []

    for i in range(len(vision_sensors)):
        sim.handleVisionSensor(vision_sensors[i])
        img = sim.getVisionSensorDepth(vision_sensors[i], sim.handleflag_depthbuffer, [0, 0], [0, 0])
        img = img[0]
        img = sim.unpackFloatTable(img, 0, 65536, 0)
        img = numpy.array(img).reshape(1, 256, 1)

        img = cv2.flip(img, 0)
        full_img.append(img)

    circular_img = numpy.concatenate((full_img[2], full_img[3], full_img[1], full_img[0]), axis=1)
    full_img = circular_img
    calculate_polar_coordinates(full_img)
    update_local_graph()

    pass


def sysCall_cleanup():
    # do some clean-up here
    pass


# See the user manual or the available code snippets for additional callback functions and details
def sign(x):
    """

    :param x:
    :return: sign(x)

    """
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


def calculate_polar_coordinates(full_img):
    # TODO fix calculations correlating the image to the circle_x and circle_y
    global circle_x, circle_y
    circle_x = []
    circle_y = []
    offset = -3 * numpy.pi / 4
    data_points = 4 * 256
    for i in range(128):
        distance = full_img[0, i * 8]
        # currently, 0 ranges from 0 - 1, but we need to scale into meters
        distance *= 2
        theta = offset + 2 * numpy.pi * (data_points - i * 8) / data_points
        circle_x.append(theta)
        if abs(math.sin(theta)) > abs(math.cos(theta)):
            circle_y.append(distance / abs(math.sin(theta)))
        else:
            circle_y.append(distance / abs(math.cos(theta)))
    return circle_x, circle_y


def get_polar_coordinates():
    global circle_x, circle_y
    return circle_x, circle_y


def init_local_graph():
    global circle_x, circle_y, p2, p3, figure, background, ax, lines
    circle_x = []
    circle_y = []
    figure, ax = plt.subplots(nrows=1, ncols=2)
    ax[1] = plt.subplot(121)
    ax[1].set(xlim=(0, 2 * math.pi), ylim=(-4, 4))

    ax[0] = plt.subplot(122, projection='polar')

    r = numpy.arange(0, 2 * math.sqrt(2), 0.01)
    theta = numpy.pi / 2 + r * 0
    ax[0].plot(theta, r, color="green")
    # setting title
    ax[0].set_title("radial")
    figure.canvas.manager.set_window_title("robot")
    # setting x-axis label and y-axis label

    plt.show(block=False)
    mplstyle.use('fast')
    # draws the polar coordinates index 0
    lines.append(ax[0].plot(circle_x, circle_y, 'bo', markersize=3)[0])

    # draws the tangent rays index 1
    lines.append(ax[1].plot(0, 0, 'bo', markersize=3)[0])

    # draws the difference rays index 2
    lines.append(ax[1].plot(0, 0, 'ro', markersize=3)[0])
    ax[1].set_xlabel("angle (radians)")
    ax[1].set_ylabel("distance (meters)")
    ax[1].set_title("difference in tangent rays")
    # draws the direction of travel index 3
    lines.append(ax[0].plot(0, 0, color='yellow')[0])
    background = figure.canvas.copy_from_bbox(ax[0].bbox)
    figure.canvas.draw()

    # draws the critical point of the difference rays index 4
    lines.append(ax[0].plot(0, 0, 'yo', markersize=3)[0])

    # draws the middle two sets of lines
    # lines.append(ax[0].plot(0,0, 'go', markersize=3)[0])
    # lines.append(ax[0].plot(0,0, 'go', markersize=3)[0])


def rotate_coordinates():
    global orientation, circle_x, circle_y
    for i in range(len(circle_x)):
        circle_x[i] = circle_x[i] + math.pi / 2 + orientation[2]


def update_local_graph():
    global circle_x, circle_y, lines, figure, background, ax, current_velocity, difference_rays
    velo_r = numpy.arange(0, current_velocity[0], 0.01)
    velo_theta = current_velocity[1] + velo_r * 0
    lines[3].set_xdata(velo_theta)
    lines[3].set_ydata(velo_r)

    rotate_coordinates()
    new_x = []
    new_y = []
    difference_rays = []
    for i in range(int(len(circle_x))):
        new_x.append(circle_x[i])
        new_y.append(circle_y[i])
        if i != 0:
            difference_rays.append(new_y[i] - new_y[i - 1])
    process_difference_rays()
    lines[0].set_xdata(new_x)
    lines[0].set_ydata(new_y)

    # difference in tangent rays
    p2_xdata = numpy.arange(0, 2 * math.pi, 2 * math.pi / int(len(difference_rays)))
    lines[1].set_xdata(p2_xdata)
    lines[1].set_ydata(difference_rays)

    # tangent rays
    lines[2].set_xdata(p2_xdata)
    lines[2].set_ydata(new_y[0:-1])
    figure.canvas.restore_region(background)
    # figure.canvas.blit(ax.bbox)
    ax[0].draw_artist(lines[0])

    mid_ray_position = process_difference_rays()
    lines[4].set_xdata(mid_ray_position[0])
    lines[4].set_ydata(mid_ray_position[1])
    figure.canvas.draw()

    figure.canvas.flush_events()

    # p1.update(circle_x, circle_y, pen=None, symbol='o')


def process_difference_rays():
    global difference_rays, circle_x, circle_y
    # the +1 offset is to account for the difference rays' calculation method
    max_ray = difference_rays.index(max(difference_rays)) + 1
    min_ray = difference_rays.index(min(difference_rays)) + 1

    # TODO: There *may* be an indexing issue due to the int() call function
    mid_ray = int((max_ray - min_ray) / 2) + min_ray
    position = [circle_x[mid_ray], circle_y[mid_ray]]
    local_x = []
    local_y = []
    for i in range(min_ray + 2, max_ray - 2, 1):
        local_x.append(circle_y[i] * math.cos(circle_x[i]))
        local_y.append(circle_y[i] * math.sin(circle_x[i]))

    edge_index = [[]]
    edge_slope = []
    prev_slope = None

    for i in range(1, len(local_x), 1):
        # slope = delta y - delta x
        slope_y = local_y[i] - local_y[i - 1]
        slope_x = local_x[i] - local_x[i - 1]
        # we need to account for purely vertical lines
        if slope_x == 0:
            slope_y = 9999
            slope_x = 1

        slope = slope_y / slope_x

        if slope > 5:
            slope = 9999
        if slope < 0.4:
            slope = 0
        # we need to account for the first index
        if prev_slope is not None:
            # TODO: Change value of tolerance
            tolerance = 1
            if prev_slope - tolerance < slope and prev_slope + tolerance > slope:
                pass
                # if equal, then we are still looking at the same line segment
            else:
                # if not equal, then we are looking at a vertex
                edge_slope.append(slope)
                edge_index[-1].append(i)
                edge_index.append([])
                edge_index[-1].append(i)

        else:
            edge_index[0].append(0)
            edge_slope.append(slope)
        # finish the array by appending the last value
        if i == len(local_x) - 1:
            edge_index[-1].append(i)
        prev_slope = slope

    index = min_ray + 2 + edge_index[int(len(edge_index) / 2)][0]
    position = [circle_x[index], circle_y[index]]
    print("edge index:" + str(edge_index))
    print("edge slope:" + str(edge_slope))
    return position


def show_depth_view():
    global full_img
    circular_img = full_img

    for i in range(50):
        full_img = numpy.concatenate((full_img, circular_img), axis=0)
    cv2.imshow('depth view', full_img)


def get_global_coordinates():
    global circle_x, circle_y, positions
    global_x = [0, 0]
    global_y = [0, 0]

    # self.x = theta
    # self.y = r

    # we utilize the x = rcostheta & y = rsintheta to convert into local rectangular coordinates
    # we then call the position of our own robot, to convert into global rectangular coordinates
    if len(orientation) == 0:
        return [0, 0], [0, 0]
    current_pos = positions

    robot_x = current_pos[0]
    robot_y = current_pos[1]
    for i in range(len(circle_x)):
        if circle_y[i] < 1.9:
            local_x = circle_y[i] * math.cos(circle_x[i])
            local_y = circle_y[i] * math.sin(circle_x[i])
            global_x.append(robot_x + local_x)
            global_y.append(robot_y + local_y)

    return global_x, global_y


def get_position():
    global positions
    if positions == []:
        positions = sim.getObjectPosition(robot, sim.handle_world)
    return positions


def get_results():
    # get results will return the vectors of proposed objects in motion
    # and positions in local coordinates

    return None