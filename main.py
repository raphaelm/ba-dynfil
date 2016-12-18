import numpy as np
import rbdl

from dynfil import constants, zmp
from dynfil.bodies import BodyTrajectory
from dynfil.utils.parser import parse_trajectory
from dynfil.utils.plot import plot_trajectories

model = rbdl.loadModel('data/models/iCubHeidelberg01_no_weights.urdf')

# initial pose: half-sitting
q_ini = constants.POSE_HALF_SITTING

chest = BodyTrajectory(model, model.GetBodyId("chest"))
chest.set_trajectories(*parse_trajectory('data/traj/com_traj.csv'))

lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
lsole.set_trajectories(*parse_trajectory('data/traj/l_foot_traj.csv'))

rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))
rsole.set_trajectories(*parse_trajectory('data/traj/r_foot_traj.csv'))

traj_zmp = zmp.calculate_zmp_trajectory(model, q_ini, chest, lsole, rsole, sampling_interval=0.01)


"""
plot_trajectories(
    trajectories=[traj_com_pos, traj_lsole_pos, traj_rsole_pos],
    labels=['CoM', 'Left foot', 'Right foot'],
    filename='out/trajectories.png',
    rotations=[traj_com_ort, traj_lsole_pos, traj_rsole_pos],
    show=True
)
"""
