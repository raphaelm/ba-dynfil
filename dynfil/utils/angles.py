import numpy as np


def rotx(rad):
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array([
        [1., 0., 0.],
        [0., c, s],
        [0., -s, c]
    ])


def roty(rad):
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array([
        [c, 0., -s],
        [0., 1., 0.],
        [s, 0., c]
    ])


def rotz(rad):
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array([
        [c, s, 0.],
        [-s, c, 0.],
        [0., 0., 1.]
    ])


def matrix_from_euler_xyz(angles, order):
    res = np.zeros([3, 3])
    if order == "123":
        R1 = rotx(angles[0])
        R2 = roty(angles[1])
        R3 = rotz(angles[2])
        res[...] = R1.dot(R2).dot(R3)
    elif order == "321":
        R1 = rotx(angles[2])
        R2 = roty(angles[1])
        R3 = rotz(angles[0])
        res[...] = R3.dot(R2).dot(R1)
    else:
        raise NotImplementedError("Unimplemented rotation order {}".format(order))

    return res


def euler_from_matrix(rotmat, order):
    angles = np.zeros([3,])
    if order == "123":
        angles[0] = np.arctan2(rotmat[1, 2], rotmat[2, 2])
        angles[1] = -np.arcsin(rotmat[0, 2])
        angles[2] = np.arctan2(rotmat[0, 1], rotmat[0, 0])
    elif order == "321":
        angles[0] = np.arctan2(-rotmat[1, 0], rotmat[0, 0])
        angles[1] = np.arcsin(rotmat[2, 0])
        angles[2] = np.arctan2(-rotmat[2, 1], rotmat[2, 2])
    else:
        raise NotImplementedError("Unimplemented rotation order {}".format(order))

    return None
