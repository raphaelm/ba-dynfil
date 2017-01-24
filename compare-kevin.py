import numpy as np
import rbdl
import os

from dynfil.utils import plot
from dynfil.utils.meshup import load_from_meshup

filename = "./data/models/iCubHeidelberg01_new_legs.urdf"
model = rbdl.loadModel(filename)

data_kevin = load_from_meshup('data/pgdata4_q_output_kevin.csv')
data_dynfil = load_from_meshup('out/inverse_from_pg.csv')
pgdata = np.genfromtxt('data/pgdata4.txt', delimiter=' ', dtype=None)
timesteps = pgdata[:, 0]

assert len(data_dynfil) == len(data_kevin) == len(pgdata)

com_kevin = np.zeros((len(data_kevin), 3))
com_dynfil = np.zeros((len(data_dynfil), 3))

for t in range(len(data_dynfil)):
    com_tmp = np.zeros(3)
    rbdl.CalcCenterOfMass(model, data_kevin[t, 1:], np.zeros(model.dof_count), com_tmp)
    com_kevin[t] = com_tmp

    com_tmp = np.zeros(3)
    rbdl.CalcCenterOfMass(model, data_dynfil[t, 1:], np.zeros(model.dof_count), com_tmp)
    com_dynfil[t] = com_tmp

plot.plot_trajectories_from_top(
    trajectories=[
        plot.PlotTrajectory(positions=com_kevin, rotations=None, label='Kevin', color='r'),
        plot.PlotTrajectory(positions=com_dynfil, rotations=None, label='Raphael', color='c'),
        plot.PlotTrajectory(positions=pgdata[:, 1:3], rotations=None, label='PG reference', color='k'),
    ],
    filename=os.path.join('out', 'com_compare_kevin.pdf'),
    title='CoM trajectories'
)

plot.plot_q_values(
    timesteps,
    (data_kevin[:, 1:], data_dynfil[:, 1:]),
    labels=('Kevin', 'Raphael'),
    filename=os.path.join('out', 'q_compare_kevin.pdf'),
    title='q trajectories'
)

plot.show_all()