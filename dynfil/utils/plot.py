# This unused import is important and needs to be executed before the matplotlib import
# Otherwise the 3D projection is not available
# noinspection PyUnresolvedReferences
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt


def plot_trajectories(trajectories, labels, filename, show=False):
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    for traj, label in zip(trajectories, labels):
        ax.plot(traj[:,0], traj[:,1], traj[:,2], label=label)

    ax.legend()
    fig.savefig(filename)
    if show:
        plt.show()

