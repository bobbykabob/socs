# python
# code on each individual robot
from math import cos, sin, sqrt, atan, pi
from typing import List
import numpy
import cv2
import matplotlib
import matplotlib.style as mplstyle
import matplotlib.pyplot as plt

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
p1 = None
figure = None
background = None
ax = None


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
    global q_target, constants, orientation, positions, has_run
    if not has_run or len(positions) == 0:
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
    global circle_x, circle_y
    circle_x = []
    circle_y = []
    offset = -3 * numpy.pi / 4
    data_points = 4 * 256

    for i in range(128):
        distance = full_img[0, i * 8]
        # currently, 0 ranges from 0 - 1, but we need to scale into meters
        distance *= 2
        circle_x.append(offset + 2 * numpy.pi * (data_points - i * 8) / data_points)
        circle_y.append(distance)

    return circle_x, circle_y


def get_polar_coordinates():
    global circle_x, circle_y
    return circle_x, circle_y


def init_local_graph():
    global circle_x, circle_y, p1, figure, background, ax
    circle_x = []
    circle_y = []
    figure, ax = plt.subplots(figsize=(2.5, 4), subplot_kw={'projection': 'polar'})

    r = numpy.arange(0, 2, 0.01)
    theta = numpy.pi / 2 + r * 0
    ax.plot(theta, r, color="green")
    # setting title
    title = "radial view: "
    plt.title(title)
    figure.canvas.set_window_title(title)

    # setting x-axis label and y-axis label
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.show(block=False)
    mplstyle.use('fast')
    p1, = ax.plot(circle_x, circle_y, 'bo')
    figure.canvas.draw()
    background = figure.canvas.copy_from_bbox(ax.bbox)


def rotate_coordinates():
    global orientation, circle_x, circle_y
    for i in range(len(circle_x)):
        circle_x[i] = circle_x[i] + math.pi / 2 + orientation[2]


def update_local_graph():
    global circle_x, circle_y, p1, figure, background, ax
    rotate_coordinates()
    new_x = []
    new_y = []
    for i in range(int(len(circle_x))):
        new_x.append(circle_x[i])
        new_y.append(circle_y[i])
    p1.set_xdata(new_x)
    p1.set_ydata(new_y)
    figure.canvas.restore_region(background)
    # figure.canvas.blit(ax.bbox)

    ax.draw_artist(p1)

    figure.canvas.update()

    figure.canvas.flush_events()

    # p1.update(circle_x, circle_y, pen=None, symbol='o')


def show_depth_view():
    global full_img
    circular_img = full_img

    for i in range(50):
        full_img = numpy.concatenate((full_img, circular_img), axis=0)
    cv2.imshow('depth view', full_img)


