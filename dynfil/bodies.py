import numpy as np

from .utils.angles import matrix_from_euler_xyz


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

    def set_trajectories(self, traj_pos, traj_angles):
        if len(traj_pos) != len(traj_angles):
            raise ValueError("Position and angle trajectories need to be of same length.")
        self.traj_pos = traj_pos
        self.traj_ort = np.array([matrix_from_euler_xyz(a, "123") for a in traj_angles])

    def to_constraint(self, time_index):
        return self.id, self.body_point, self.traj_pos[time_index], self.traj_ort[time_index]

    def __len__(self):
        return len(self.traj_pos)
