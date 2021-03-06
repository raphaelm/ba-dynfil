import numpy as np
import rbdl

from . import analytical, numerical
from .utils import ik_derivs_fd, interpolate_savgol


def inverse(model, q_ini, chest, lsole, rsole, method='numerical'):
    """
    Like inverse(), but returns a tuple of (q, qdot, qddot)
    """
    if method == 'numerical':
        return numerical.ik_trajectory(model, q_ini, chest, lsole, rsole)
    else:
        return analytical.ik_trajectory(model, q_ini, chest, lsole, rsole)[0]


def inverse_with_derivatives(model, q_ini, chest, lsole, rsole, times, method='numerical', interpolate='none'):
    """
    Like inverse(), but returns a tuple of (q, qdot, qddot)
    """
    if method == 'numerical':
        q = numerical.ik_trajectory(model, q_ini, chest, lsole, rsole)

        qdot = ik_derivs_fd(q, times)
        qddot = ik_derivs_fd(qdot, times)
    else:
        res = analytical.ik_trajectory(model, q_ini, chest, lsole, rsole)
        q = res[0]
        qdot = res[1][:, :, 0]
        qddot = res[2][:, :, 0]

    if interpolate == 'savgol':
        interpolate_savgol(qddot)

    return q, qdot, qddot


def com_trajectory(model, chest, q):
    com = np.zeros((len(q), 3))
    for t in range(len(q)):
        com_tmp = np.zeros(3)
        rbdl.CalcCenterOfMass(model.model, q[t], np.zeros(model.dof_count), com_tmp)
        com[t] = com_tmp
    return com
