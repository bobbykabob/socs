from math import cos, sin, sqrt, atan
from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH

from typing import List

b = 1.5


def move_robot(q_target: List[float], q_current: List[float]):
    """

    :param q_target: list of floats from arbitrary configuration [x'_r, y'_r, phi'_r],
    :param q_current: list of floats from current configuration [x_r, y_r, phi_r]

    :return:
    """

    # equation 4
    x = q_current[0] * cos(q_target[2]) + q_current[1] * sin(q_target[2]) - q_target[0] * cos(q_target[2]) - q_target[
        1] * sin(q_target[2])

    y = -q_current[0] * sin(q_target[2]) + q_current[1] * cos(q_target[2]) + q_target[0] * sin(q_target[2]) - q_target[
        1] * cos(q_target[2])

    # Φ -> phi
    phi_r = q_current[2] - q_target[2]

    # equation 3
    d = sqrt(x ** 2 + y ** 2)

    s_x = sign(x)

    # η -> eta
    # -pi/ 2 <= eta <= pi/2
    eta = s_x * atan(y / abs(x))

    # ψ -> psi
    psi_1 = cos(eta - phi_r)
    psi_2 = sin(eta - phi_r)

    c_1 = ROBOT_C1
    c_2 = ROBOT_C2

    # ξ -> xi
    # 1 < b < 2
    phi_a = (2 / b) * eta
    xi = phi_a - phi_r

    # translational velocity V_R
    v_r = -c_1 * s_x * (psi_1 / ((psi_1 ** 2) + xi ** 2)) * d
    # rotational velocity
    # ω -> omega
    omega_r = c_2 * xi * (c_1 / ((psi_1 ** 2) + xi ** 2)) * ((2 / b) * psi_1 * psi_2 + xi * (d ** 2))

    # equation 2

    B = ROBOT_TRACK_WIDTH
    # velocity of right wheel
    v_right = v_r - (B * omega_r) / 2

    # velocity of left wheel
    v_left = v_r + (B * omega_r) / 2
    return v_left, v_right


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
