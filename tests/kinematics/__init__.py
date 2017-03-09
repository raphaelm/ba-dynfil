import os

import numpy as np


def minimal_trajectory(model, fname):
    pgdata = np.genfromtxt(os.path.join(os.path.dirname(__file__), fname), delimiter=' ', dtype=None)
    timesteps = pgdata[:, 0]

    chest = model.get_body(model.chest_body_id)
    lsole = model.get_body(model.lfoot_body_id)
    rsole = model.get_body(model.rfoot_body_id)

    chest.set_trajectories(pgdata[:, 1:4], pgdata[:, 4:7])
    chest.traj_pos_dot = pgdata[:, 28:31]
    chest.traj_pos_ddot = pgdata[:, 37:40]

    lsole.set_trajectories(pgdata[:, 7:10], pgdata[:, 10:13])
    lsole.traj_pos_dot = pgdata[:, 22:25]
    lsole.traj_pos_ddot = pgdata[:, 31:34]

    rsole.set_trajectories(pgdata[:, 13:16], pgdata[:, 16:19])
    rsole.traj_pos_dot = pgdata[:, 25:28]
    rsole.traj_pos_ddot = pgdata[:, 34:37]

    return timesteps, chest, rsole, lsole
