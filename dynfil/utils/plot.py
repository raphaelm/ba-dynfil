# This unused import is important and needs to be executed before the matplotlib import
# Otherwise the 3D projection is not available
# noinspection PyUnresolvedReferences
from collections import namedtuple

from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches

from .. import constants


def rotate_vector(vec, matarr):
    res = np.zeros((len(matarr), 3))
    vec = np.transpose([vec])
    for i, mat in enumerate(matarr):
        res[i] = np.dot(mat, vec)[:,0]
    return res


PlotTrajectory = namedtuple('PlotTrajectory', 'positions rotations label color')
FootTrajectory = namedtuple('FootTrajectory', 'positions rotations color')
PlotResiduum = namedtuple('PlotResiduum', 'times values label color')


def plot_trajectories(trajectories, filename=None, show=False):
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    for traj in trajectories:
        if isinstance(traj, PlotTrajectory):
            ax.plot(traj.positions[:,0], traj.positions[:,1], traj.positions[:,2], label=traj.label, color=traj.color)
        elif isinstance(traj, FootTrajectory):
            hatch_alt = True
            for t in range(1, len(traj.positions)):
                if traj.positions[t][2] < 1e-13 and traj.positions[t-1][2] > 1e-13:
                    # First timestep with this foot on the ground, draw foot.
                    # TODO: Rotate foot
                    p = patches.Rectangle(
                        (
                            traj.positions[t][0] - constants.FOOT_LENGTH / 2.0,
                            traj.positions[t][1] - constants.FOOT_WIDTH / 2.0,
                        ),
                        constants.FOOT_LENGTH, constants.FOOT_WIDTH,
                        fill=False, hatch='\\' if hatch_alt else '/',
                        color=traj.color
                    )
                    ax.add_patch(p)
                    art3d.pathpatch_2d_to_3d(p, z=0, zdir="z")
                    hatch_alt = not hatch_alt

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
    if filename:
        fig.savefig(filename)
    if show:
        plt.show()


def plot_trajectories_from_top(trajectories, filename=None, show=False):
    fig = plt.figure()
    ax = fig.gca()
    for traj in trajectories:
        if isinstance(traj, PlotTrajectory):
            ax.plot(traj.positions[:,0], traj.positions[:,1], label=traj.label)
        elif isinstance(traj, FootTrajectory):
            hatch_alt = True
            for t in range(1, len(traj.positions)):
                if traj.positions[t][2] < 1e-13 and traj.positions[t-1][2] > 1e-13:
                    # First timestep with this foot on the ground, draw foot.
                    # TODO: Rotate foot
                    ax.add_patch(
                        patches.Rectangle(
                            (
                                traj.positions[t][0] - constants.FOOT_LENGTH / 2.0,
                                traj.positions[t][1] - constants.FOOT_WIDTH / 2.0,
                            ),
                            constants.FOOT_LENGTH, constants.FOOT_WIDTH,
                            fill=False, hatch='\\' if hatch_alt else '/',
                            color=traj.color
                        )
                    )
                    hatch_alt = not hatch_alt

    leg = ax.legend()
    for legobj in leg.legendHandles:
        legobj.set_linewidth(5.0)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    if filename:
        fig.savefig(filename)
    if show:
        plt.show()


def plot_q_values(times, q, qdot, qddot, filename=None, show=False, limit=None):
    if not isinstance(q, tuple):
        q = (q,)
    nplots = len(q[0][0])
    if limit:
        nplots = min(limit, nplots)
    fig, axes = plt.subplots(nplots, 3, sharex=True)

    for i in range(nplots):
        ax_q, ax_qdot, ax_qddot = axes[i]

        for k in range(len(q)):
            ax_q.plot(times, q[k][:,i], label=r'$q_{}$'.format(i), marker='x', markersize=2)
            ax_qdot.plot(times, qdot[k][:,i], label=r'$\dot q_{}$'.format(i), marker='x', markersize=2)
            ax_qddot.plot(times, qddot[k][:,i], label=r'$\ddot q_{}$'.format(i), marker='x', markersize=2)

    axes[0, 0].set_title('q')
    axes[0, 1].set_title('qdot')
    axes[0, 2].set_title('qddot')
    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('time')
    axes[-1, 2].set_xlabel('time')

    if filename:
        fig.savefig(filename)
    if show:
        plt.show()


def plot_q_interpolation(times, data_without, data_with, name='qddot', filename=None, limit=5, show=False):
    nplots = len(data_with[0])
    if limit:
        nplots = min(limit, nplots)
    fig, axes = plt.subplots(nplots, 2, sharex=True)

    for i in range(nplots):
        axes[i, 0].plot(times, data_without[:,i], marker='x', markersize=2)
        axes[i, 1].plot(times, data_with[:,i], marker='x', markersize=2)

    axes[0, 0].set_title('{} without interpolation'.format(name))
    axes[0, 1].set_title('{} with interpolation'.format(name))
    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('time')

    if filename:
        fig.savefig(filename)
    if show:
        plt.show()


def plot_residuums(data, filename=None, show=False):
    fig, axes = plt.subplots(2, 2)

    for i, row in enumerate(data):
        if isinstance(row, PlotResiduum):
            normed_data = (row.values * row.values).sum(axis=1) ** 0.5

            axes[i, 0].plot(row.times, normed_data, label=row.label, color=row.color)
            axes[i, 0].set_ylabel('residuum')
            axes[i, 0].set_title(row.label)

            axes[i, 1].hist(normed_data, label=row.label, color=row.color)

    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('residuum')
    plt.tight_layout()
    if filename:
        fig.savefig(filename)
    if show:
        plt.show()
