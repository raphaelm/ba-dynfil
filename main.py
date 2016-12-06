from dynfil.utils.parser import parse_trajectory
from dynfil.utils.plot import plot_trajectories

traj_com_pos, traj_com_ort = parse_trajectory('data/traj/com_traj.csv')
traj_foot_l_pos, traj_foot_l_ort = parse_trajectory('data/traj/l_foot_traj.csv')
traj_foot_r_pos, traj_foot_r_ort = parse_trajectory('data/traj/r_foot_traj.csv')


plot_trajectories(
    trajectories=[traj_com_pos, traj_foot_l_pos, traj_foot_r_pos],
    labels=['CoM', 'Left foot', 'Right foot'],
    filename='out/trajectories.png',
    rotations=[traj_com_ort, traj_foot_l_ort, traj_foot_r_ort],
    show=True
)
