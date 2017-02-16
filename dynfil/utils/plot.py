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


DPI = 150  # TODO: Set to 300
PlotTrajectory = namedtuple('PlotTrajectory', 'positions rotations label color')
FootTrajectory = namedtuple('FootTrajectory', 'positions rotations color')
PlotResiduum = namedtuple('PlotResiduum', 'times values label color')


def plot_trajectories(trajectories, filename=None, title=None):
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure(figsize=(11.69, 8.27))
    if title:
        fig.suptitle(title)
    ax = fig.gca(projection='3d')
    ax.axis('equal')

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
        fig.savefig(filename, dpi=DPI)


def plot_trajectories_from_top(trajectories, filename=None, title=None):
    fig = plt.figure(figsize=(11.69, 8.27))
    ax = fig.gca()
    ax.axis('equal')
    if title:
        fig.suptitle(title)
    for traj in trajectories:
        if isinstance(traj, PlotTrajectory):
            ax.plot(traj.positions[:,0], traj.positions[:,1], label=traj.label,
                    marker='x', markersize=2)
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
        fig.savefig(filename, dpi=DPI)


def plot_trajectories_1d(times, trajectories, filename=None, title=None):
    fig = plt.figure(figsize=(11.69, 8.27))
    ax = fig.gca()
    if title:
        fig.suptitle(title)
    for traj in trajectories:
        if isinstance(traj, PlotTrajectory):
            ax.plot(times, traj.positions, label=traj.label)

    leg = ax.legend()
    for legobj in leg.legendHandles:
        legobj.set_linewidth(5.0)
    ax.set_xlabel('t')
    ax.set_ylabel('y')
    if filename:
        fig.savefig(filename, dpi=DPI)


def plot_trajectories_1d_axis(times, trajectories, filename=None, title=None):
    fig, axes = plt.subplots(3, len(trajectories), sharex=True, sharey='row', figsize=(8.27, 11.69))
    if title:
        fig.suptitle(title)
    for i, traj in enumerate(trajectories):
        if isinstance(traj, PlotTrajectory):
            if len(trajectories) > 1:
                a = axes[:, i]
            else:
                a = axes
            a[0].plot(times, traj.positions[:, 0], label='x', color=traj.color)
            a[1].plot(times, traj.positions[:, 1], label='y', color=traj.color)
            a[2].plot(times, traj.positions[:, 2], label='z', color=traj.color)
            a[0].set_title(traj.label)
            if i == 0:
                a[0].set_ylabel('x')
                a[1].set_ylabel('y')
                a[2].set_ylabel('z')

    if len(trajectories) > 1:
        for i in range(len(trajectories)):
            axes[-1, i].set_xlabel('t')
    else:
        axes[-1].set_label('t')
    if filename:
        fig.savefig(filename, dpi=DPI)


def plot_q_values(times, q, labels, filename=None, title=None):
    if not isinstance(q, tuple):
        q = (q,)
    nplots = len(q[0][0])
    fig, axes = plt.subplots(int(nplots / 4) + 1, 4, sharex=True, figsize=(8.27, 11.69))
    if title:
        fig.suptitle(title)

    for i in range(nplots):
        ax = axes[int(i / 4)][i % 4]
        ax.set_title('$q_{%d}$' % i)

        for k in range(len(q)):
            traj = np.array([v.tolist() for v in q[k]])
            ax.plot(times, traj[:, i], label=labels[k], marker='x', markersize=2)

        if i == 0:
            ax.legend(loc='upper left')

    axes[0, 0].set_ylabel('q')
    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('time')
    axes[-1, 2].set_xlabel('time')
    axes[-1, 3].set_xlabel('time')

    if filename:
        fig.savefig(filename, dpi=DPI)


def plot_q_derivs(times, q, qdot, qddot, labels, filename=None, limit=None, title=None):
    if not isinstance(q, tuple):
        q = (q,)
    nplots = len(q[0][0])
    if limit:
        nplots = min(limit, nplots)
    fig, axes = plt.subplots(nplots, 3, sharex=True, figsize=(8.27, 11.69))
    if title:
        fig.suptitle(title)

    for i in range(nplots):
        ax_q, ax_qdot, ax_qddot = axes[i]

        for k in range(len(q)):
            ax_q.plot(times, q[k][:,i], label=labels[k], marker='x', markersize=2)
            ax_qdot.plot(times, qdot[k][:,i], label=labels[k], marker='x', markersize=2)
            ax_qddot.plot(times, qddot[k][:,i], label=labels[k], marker='x', markersize=2)

        if i == 0:
            ax_q.legend()

    axes[0, 0].set_title('q')
    axes[0, 1].set_title('qdot')
    axes[0, 2].set_title('qddot')
    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('time')
    axes[-1, 2].set_xlabel('time')

    fig.tight_layout()
    if filename:
        fig.savefig(filename, dpi=DPI)


def plot_q_interpolation(times, data_without, data_with, name='qddot', filename=None, limit=5, title=None):
    nplots = len(data_with[0])
    if limit:
        nplots = min(limit, nplots)
    fig, axes = plt.subplots(nplots, 2, sharex=True, figsize=(8.27, 11.69))
    if title:
        fig.suptitle(title)

    for i in range(nplots):
        axes[i, 0].plot(times, data_without[:,i], marker='x', markersize=2)
        axes[i, 1].plot(times, data_with[:,i], marker='x', markersize=2)

    axes[0, 0].set_title('{} without interpolation'.format(name))
    axes[0, 1].set_title('{} with interpolation'.format(name))
    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('time')

    if filename:
        fig.savefig(filename, dpi=DPI)


def plot_residuums(data, filename=None, title=None):
    fig, axes = plt.subplots(len(data), 3, figsize=(11.69, 8.27))
    if title:
        fig.suptitle(title)

    prev_normed_data = None
    for i, row in enumerate(data):
        if isinstance(row, PlotResiduum):
            normed_data = (row.values * row.values).sum(axis=1) ** 0.5

            axes[i, 0].plot(row.times, normed_data, label=row.label, color=row.color)
            axes[i, 0].set_ylabel('residuum')
            axes[i, 0].set_title(row.label)

            axes[i, 1].hist(normed_data, label=row.label, color=row.color)

            if prev_normed_data is not None:
                improved_cnt = np.sum(np.less(normed_data, prev_normed_data))
                equal_cnt = np.sum(np.equal(normed_data, prev_normed_data))
                degr_cnt = len(normed_data) - improved_cnt - equal_cnt

                axes[i, 2].pie(
                    [improved_cnt, equal_cnt, degr_cnt],
                    labels=[
                        'better ({})'.format(improved_cnt),
                        'equal ({})'.format(equal_cnt),
                        'worse ({})'.format(degr_cnt),
                    ],
                    colors=['g', 'y', 'r'],
                )
            else:
                axes[i, 2].axis('off')


            prev_normed_data = normed_data

    axes[-1, 0].set_xlabel('time')
    axes[-1, 1].set_xlabel('residuum')
    plt.tight_layout()
    if filename:
        fig.savefig(filename, dpi=DPI)


def show_all():
    plt.show()