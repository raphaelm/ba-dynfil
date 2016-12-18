# This unused import is important and needs to be executed before the matplotlib import
# Otherwise the 3D projection is not available
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm3


def rotate_vector(vec, x, y, z):
    res = []
    for _x, _y, _z in zip(x, y, z):
        Mx = expm3(np.cross(np.eye(3), np.array([1,0,0])* _x * np.pi / 180))
        My = expm3(np.cross(np.eye(3), np.array([0,1,0])* _y * np.pi / 180))
        Mz = expm3(np.cross(np.eye(3), np.array([0,0,1])* _z * np.pi / 180))
        res.append(np.dot(Mz, np.dot(My, np.dot(Mx, vec))))
    return np.array(res)


def plot_trajectories(trajectories, labels, filename, rotations=None, show=False):
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    for traj, label in zip(trajectories, labels):
        ax.plot(traj[:,0], traj[:,1], traj[:,2], label=label)

    """
    if rotations:
        rotsteps = 15
        for traj, rots in zip(trajectories, rotations):
            ux = rotate_vector(np.array([1, 0, 0]), rots[::rotsteps,0], rots[::rotsteps,1], rots[::rotsteps,2])
            uy = rotate_vector(np.array([0, 1, 0]), rots[::rotsteps,0], rots[::rotsteps,1], rots[::rotsteps,2])
            uz = rotate_vector(np.array([0, 0, 1]), rots[::rotsteps,0], rots[::rotsteps,1], rots[::rotsteps,2])
            ax.quiver(traj[::rotsteps,0], traj[::rotsteps,1], traj[::rotsteps,2], ux[:,0], ux[:,1], ux[:,2],
                      length=0.05, pivot='tail', color='red')
            ax.quiver(traj[::rotsteps,0], traj[::rotsteps,1], traj[::rotsteps,2], uy[:,0], uy[:,1], uy[:,2],
                      length=0.05, pivot='tail', color='green')
            ax.quiver(traj[::rotsteps,0], traj[::rotsteps,1], traj[::rotsteps,2], uz[:,0], uz[:,1], uz[:,2],
                      length=0.05, pivot='tail', color='blue')
    """

    ax.legend()
    fig.savefig(filename)
    if show:
        plt.show()

