# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np


def quat_evolve(q, w):
    # quat_evolve returns the time derivative of the quaternion attitude vector
    # as a function of the current attitude q and the rotation w expressed in
    # the same frame as q.

    # correction is to correct for integration errors
    # (c.f. Modeling and Simulation of aerospace
    # vehicle dynamics, second edition p.126, Peter H. Zipfel)

    correction = np.array([[0, w[2], -w[1], w[0]],
                           [-w[2], 0, w[0], w[1]],
                           [w[1], -w[0], 0, w[2]],
                           [-w[0], -w[1], -w[2], 0]])

    return (1 - np.linalg.norm(q)) * q + np.sum(0.5 * correction * q, 1)
