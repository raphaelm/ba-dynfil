import numpy as np
import rbdl

from .utils.angles import matrix_from_euler_xyz, euler_from_matrix


class BodyTrajectory(object):
    """
    Represents one body trajectory in our model.
    """

    def __init__(self, model, body_id):
        self.model = model
        self.id = body_id
        self.body_point = np.array([0.0, 0.0, 0.0])
        self.traj_pos = np.array([])
        self.traj_ort = np.array([])
        self.traj_pos_dot = np.array([])
        self.traj_pos_ddot = np.array([])

    def set_trajectories(self, traj_pos, traj_angles, traj_pos_dot=None, traj_pos_ddot=None, offset_angles=np.zeros(3)):
        if len(traj_pos) != len(traj_angles):
            raise ValueError("Position and angle trajectories need to be of same length.")
        self.traj_pos = traj_pos
        traj_angles = offset_angles + np.deg2rad(traj_angles)
        self.traj_ort = np.array([matrix_from_euler_xyz(a, "123") for a in traj_angles])

        self.traj_pos_dot = traj_pos_dot or np.zeros_like(traj_pos)
        self.traj_pos_ddot = traj_pos_ddot or np.zeros_like(traj_pos)

    def to_constraint(self, time_index):
        return self.id, self.body_point, self.traj_pos[time_index], self.traj_ort[time_index]

    def __len__(self):
        return len(self.traj_pos)

    def copy(self):
        n = BodyTrajectory(self.model, self.id)
        n.body_point = np.copy(self.body_point)
        n.traj_pos = np.copy(self.traj_pos)
        n.traj_ort = np.copy(self.traj_ort)
        n.traj_pos_dot = np.copy(self.traj_pos_dot)
        n.traj_pos_ddot = np.copy(self.traj_pos_ddot)
        return n
