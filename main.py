import numpy as np
import rbdl

from dynfil import constants
from dynfil.utils.angles import matrix_from_euler_xyz
from dynfil.utils.parser import parse_trajectory
from dynfil.utils.plot import plot_trajectories

traj_com_pos, traj_com_ort = parse_trajectory('data/traj/com_traj.csv')
traj_lsole_pos, traj_lsole_ort = parse_trajectory('data/traj/l_foot_traj.csv')
traj_rsole_pos, traj_rsole_ort = parse_trajectory('data/traj/r_foot_traj.csv')

model = rbdl.loadModel('data/models/iCubHeidelberg01_no_weights.urdf')

# initial pose: half-sitting
q_ini = constants.POSE_HALF_SITTING

chest_body_id = model.GetBodyId("chest")
chest_body_point = np.array([0.0, 0.0, 0.0])
chest_tpos = np.array([0.0685, 0.0, 0.688])
chest_angles = np.array([np.pi/2., 0.0, np.pi/2.])
chest_tori = matrix_from_euler_xyz(chest_angles, "123")

lsole_body_id = model.GetBodyId("l_sole")
lsole_body_point = np.array([0.0, 0.0, 0.0])
lsole_tpos = np.array([0.0, 0.0739, 0.0])
lsole_angles = np.array([0.0, 0.0, 0.0])
lsole_tori = matrix_from_euler_xyz(lsole_angles, "123")

rsole_body_id = model.GetBodyId("r_sole")
rsole_body_point = np.array([0.0, 0.0, 0.0])
rsole_tpos = np.array([0.0, -0.0739, 0.0])
rsole_angles = np.array([0.0, 0.0, 0.0])
rsole_tori = matrix_from_euler_xyz(rsole_angles, "123")

plot_trajectories(
    trajectories=[traj_com_pos, traj_lsole_pos, traj_rsole_pos],
    labels=['CoM', 'Left foot', 'Right foot'],
    filename='out/trajectories.png',
    rotations=[traj_com_ort, traj_lsole_pos, traj_rsole_pos],
    show=True
)
