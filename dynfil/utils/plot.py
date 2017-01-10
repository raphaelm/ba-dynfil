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


def plot_q_values(times, q, qdot, qddot, filename, show=False):
    fig, axes = plt.subplots(len(q[0]), 3, sharex=True)

    for i in range(len(q[0])):
        ax_q, ax_qdot, ax_qddot = axes[i]
        ax_q.plot(times, q[:,i], label=r'$q_{}$'.format(i), marker='.', markersize=0.5)
        ax_qdot.plot(times, qdot[:,i], label=r'$\dot q_{}$'.format(i), marker='.', markersize=0.5)
        ax_qddot.plot(times, qddot[:,i], label=r'$\ddot q_{}$'.format(i), marker='.', markersize=0.5)

    axes[0, 0].set_title('q')
    axes[0, 1].set_title('qdot')
    axes[0, 2].set_title('qddot')
    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('time')
    axes[-1, 2].set_xlabel('time')

    fig.savefig(filename)
    if show:
        plt.show()
