# python
# code on each individual robot
from math import cos, sin, sqrt, atan, pi
from typing import List
import numpy
import cv2
import matplotlib
import matplotlib.style as mplstyle
import matplotlib.pyplot as plt

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
p1 = None
p2 = None
figure = None
background = None
ax = []
velo_line = None
current_velocity = [0, 0]


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
    global circle_x, circle_y, p1, p2, figure, background, ax, velo_line
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
    p1, = ax[0].plot(circle_x, circle_y)
    p2, = ax[1].plot(0, 0)
    ax[1].set_xlabel("angle (radians)")
    ax[1].set_ylabel("distance (meters)")
    ax[1].set_title("difference in tangent rays")
    velo_line, = ax[0].plot(0, 0, color='yellow')
    figure.canvas.draw()
    background = figure.canvas.copy_from_bbox(ax[0].bbox)


def rotate_coordinates():
    global orientation, circle_x, circle_y
    for i in range(len(circle_x)):
        circle_x[i] = circle_x[i] + math.pi / 2 + orientation[2]


def update_local_graph():
    global circle_x, circle_y, p1, p2, figure, background, ax, current_velocity, velo_line
    velo_r = numpy.arange(0, current_velocity[0], 0.01)
    velo_theta = current_velocity[1] + velo_r * 0
    velo_line.set_xdata(velo_theta)
    velo_line.set_ydata(velo_r)

    rotate_coordinates()
    new_x = []
    new_y = []
    difference_rays = []
    for i in range(int(len(circle_x))):
        new_x.append(circle_x[i])
        new_y.append(circle_y[i])
        if i != 0:
            difference_rays.append(new_y[i] - new_y[i - 1])
    p1.set_xdata(new_x)
    p1.set_ydata(new_y)

    # tangent rays
    p2.set_xdata(numpy.arange(0, 2 * math.pi, 2 * math.pi / int(len(difference_rays))))
    p2.set_ydata(difference_rays)

    figure.canvas.restore_region(background)
    # figure.canvas.blit(ax.bbox)
    ax[0].draw_artist(p1)

    figure.canvas.draw()

    figure.canvas.flush_events()

    # p1.update(circle_x, circle_y, pen=None, symbol='o')


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