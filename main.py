import rbdl

from dynfil import constants, zmp
from dynfil.bodies import BodyTrajectory
from dynfil.utils.plot import plot_trajectories

model = rbdl.loadModel('data/models/iCubHeidelberg01_no_weights.urdf')

# initial pose: half-sitting
q_ini = constants.POSE_HALF_SITTING

chest = BodyTrajectory(model, model.GetBodyId("chest"))
chest.load_trajectory('data/traj/com_traj.csv')

lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
lsole.load_trajectory('data/traj/l_foot_traj.csv')

rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))
rsole.load_trajectory('data/traj/r_foot_traj.csv')

traj_zmp = zmp.calculate_zmp_trajectory(model, q_ini, chest, lsole, rsole, sampling_interval=0.01)


plot_trajectories(
    trajectories=[chest.traj_pos, lsole.traj_pos, rsole.traj_pos],
    labels=['CoM', 'Left foot', 'Right foot'],
    filename='out/trajectories.png',
    rotations=[chest.traj_ort, lsole.traj_ort, rsole.traj_ort],
    show=True
)
