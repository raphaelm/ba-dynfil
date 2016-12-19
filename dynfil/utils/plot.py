# This unused import is important and needs to be executed before the matplotlib import
# Otherwise the 3D projection is not available
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt


def rotate_vector(vec, matarr):
    res = np.zeros((len(matarr), 3))
    vec = np.transpose([vec])
    for i, mat in enumerate(matarr):
        res[i] = np.dot(mat, vec)[:,0]
    return res


def plot_trajectories(trajectories, labels, filename, rotations=None, show=False):
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    for traj, label in zip(trajectories, labels):
        ax.plot(traj[:,0], traj[:,1], traj[:,2], label=label)

    """
    if rotations:
        rotsteps = 30
        for traj, rots in zip(trajectories, rotations):
            ux = rotate_vector(np.array([1, 0, 0]), rots[::rotsteps])
            uy = rotate_vector(np.array([0, 1, 0]), rots[::rotsteps])
            uz = rotate_vector(np.array([0, 0, 1]), rots[::rotsteps])
            ax.quiver(traj[::rotsteps,0], traj[::rotsteps,1], traj[::rotsteps,2], ux[:,0], ux[:,1], ux[:,2],
                      length=0.05, pivot='tail', color='red')
            ax.quiver(traj[::rotsteps,0], traj[::rotsteps,1], traj[::rotsteps,2], uy[:,0], uy[:,1], uy[:,2],
                      length=0.05, pivot='tail', color='green')
            ax.quiver(traj[::rotsteps,0], traj[::rotsteps,1], traj[::rotsteps,2], uz[:,0], uz[:,1], uz[:,2],
                      length=0.05, pivot='tail', color='blue')
    """

    ax.legend()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    fig.savefig(filename)
    if show:
        plt.show()


def plot_trajectories_from_top(trajectories, labels, filename, rotations=None, show=False):
    fig = plt.figure()
    ax = fig.gca()
    for traj, label in zip(trajectories, labels):
        ax.plot(traj[:,0], traj[:,1], label=label)
    ax.legend()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    fig.savefig(filename)
    if show:
        plt.show()
