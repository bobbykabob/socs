# python
# code on each individual robot
from math import cos, sin, sqrt, atan2
from typing import List

left_joint = None
right_joint = None
robot = None
q_target = []
constants = []
orientation = []
positions = []
has_run = False


def update_actuation(l_q_target: List[float], l_constants: List[float]):
    global q_target, constants, has_run
    # updating the global variables
    q_target = l_q_target
    constants = l_constants
    has_run = True
    pass


def sysCall_init():
    # initiate robot
    global left_joint, right_joint, robot
    robot = sim.getObject('.')
    left_joint = sim.getObject('./leftjoint')
    right_joint = sim.getObject('./rightjoint')
    pass


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
    q_current = [positions[0], positions[1], orientation[2]]
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
    eta = s_x * atan2(y, abs(x))

    # psi
    psi_1 = cos(eta - phi_r)
    psi_2 = sin(eta - phi_r)

    # xi
    # 1 < b < 2
    phi_a = (2 / b) * eta
    xi = phi_a - phi_r

    # translational velocity V_R
    v_r = -c_1 * s_x * (psi_1 / ((psi_1 ** 2) + xi ** 2)) * d
    # rotational velocity
    # ? -> omega
    omega_r = c_2 * xi * (c_1 / ((psi_1 ** 2) + xi ** 2)) * ((2 / b) * psi_1 * psi_2 + xi * (d ** 2))

    # equation 2

    # velocity of right wheel
    v_right = v_r + (B * omega_r) / 2

    # velocity of left wheel
    v_left = v_r - (B * omega_r) / 2
    sim.setJointTargetVelocity(left_joint, v_left)
    sim.setJointTargetVelocity(right_joint, v_right)
    pass


def sysCall_sensing():
    # update robot positions
    global positions, orientation
    positions = sim.getObjectPosition(robot, sim.handle_world)
    orientation = sim.getObjectOrientation(robot, sim.handle_world)

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
