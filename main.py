import rbdl
import numpy as np

from dynfil import constants, zmp
from dynfil.bodies import BodyTrajectory
from dynfil.utils.plot import plot_trajectories_from_top, plot_trajectories

model = rbdl.loadModel('data/models/iCubHeidelberg01_no_weights.urdf')

# initial pose: half-sitting
q_ini = constants.POSE_HALF_SITTING

pgdata = np.genfromtxt('data/traj/alpha0015beta02/PatternGeneratorData.csv', delimiter=',', dtype=None)

chest = BodyTrajectory(model, model.GetBodyId("chest"))
chest.set_trajectories(pgdata[:,1:4], pgdata[:,4:7])

lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
lsole.set_trajectories(pgdata[:,7:10], pgdata[:,10:13])

rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))
rsole.set_trajectories(pgdata[:,13:16], pgdata[:,16:19])

#traj_zmp = zmp.calculate_zmp_trajectory(model, q_ini, chest, lsole, rsole, sampling_interval=0.01)


plot_trajectories(
    trajectories=[chest.traj_pos, lsole.traj_pos, rsole.traj_pos],
    labels=['CoM', 'Left foot', 'Right foot'],
    filename='out/trajectories.png',
    rotations=[chest.traj_ort, lsole.traj_ort, rsole.traj_ort],
    show=True
)
