# This unused import is important and needs to be executed before the matplotlib import
# Otherwise the 3D projection is not available
# noinspection PyUnresolvedReferences
from collections import namedtuple

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


PlotTrajectory = namedtuple('PlotTrajectory', 'positions rotations label color')


def plot_trajectories(trajectories, filename, show=False):
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    for traj in trajectories:
        ax.plot(traj.positions[:,0], traj.positions[:,1], traj.positions[:,2], label=traj.label, color=traj.color)

    """
    if rotations:
        rotsteps = 30
        for traj in trajectories:
            if not traj.rotations:
                continue
            ux = rotate_vector(np.array([1, 0, 0]), traj.rotations[::rotsteps])
            uy = rotate_vector(np.array([0, 1, 0]), traj.rotations[::rotsteps])
            uz = rotate_vector(np.array([0, 0, 1]), traj.rotations[::rotsteps])
            ax.quiver(traj.positions[::rotsteps,0], traj.positions[::rotsteps,1], traj.positions[::rotsteps,2],
                      ux[:,0], ux[:,1], ux[:,2],
                      length=0.05, pivot='tail', color='red')
            ax.quiver(traj.positions[::rotsteps,0], traj.positions[::rotsteps,1], traj.positions[::rotsteps,2],
                      uy[:,0], uy[:,1], uy[:,2],
                      length=0.05, pivot='tail', color='green')
            ax.quiver(traj.positions[::rotsteps,0], traj.positions[::rotsteps,1], traj.positions[::rotsteps,2],
                      uz[:,0], uz[:,1], uz[:,2],
                      length=0.05, pivot='tail', color='blue')
    """

    leg = ax.legend()
    for legobj in leg.legendHandles:
        legobj.set_linewidth(5.0)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    fig.savefig(filename)
    if show:
        plt.show()


def plot_trajectories_from_top(trajectories, filename, show=False):
    fig = plt.figure()
    ax = fig.gca()
    for traj in trajectories:
        ax.plot(traj.positions[:,0], traj.positions[:,1], label=traj.label)
    leg = ax.legend()
    for legobj in leg.legendHandles:
        legobj.set_linewidth(5.0)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    fig.savefig(filename)
    if show:
        plt.show()


def plot_q_values():

    fig = plt.figure()
    ax = fig.gca()

    for i in range(len(q_calc[0,:])):
        ax.plot(timesteps, qdot_calc[:,i], label='qdot_{}'.format(i), marker='.')

    leg = ax.legend()
    for legobj in leg.legendHandles:
        legobj.set_linewidth(5.0)

    ax.set_xlabel('t')
    ax.set_ylabel('value')
    plt.show()
