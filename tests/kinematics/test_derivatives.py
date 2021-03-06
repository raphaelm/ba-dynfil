import numpy as np

from dynfil import constants
from dynfil.bodies import BodyTrajectory
from dynfil.kinematics.analytical import ik_one_leg, ik_constants


RTOL = 1e-7
ATOL = 1e-7
EPS = 1e-8


def ik_one_leg_fd(D, A, B, root_r, root_E, foot_r, foot_E, root_dot, foot_dot):
    q, __, __ = ik_one_leg(D, A, B, root_r, root_E, foot_r, foot_E, root_dot, foot_dot)
    if root_dot is not None and np.any(root_dot):
        dot_ndirs = root_dot.shape[-1]
    else:
        dot_ndirs = None

    qdot = np.zeros([6, len(q)])
    if dot_ndirs:
        qdot = np.zeros([6, dot_ndirs])
        root_h = np.zeros_like(root_r)
        foot_h = np.zeros_like(foot_r)
        for idir in range(dot_ndirs):
            root_h[:] = root_r + EPS * root_dot[:, idir]
            foot_h[:] = foot_r + EPS * foot_dot[:, idir]
            q_h, __, __ = ik_one_leg(D, A, B, root_h, root_E, foot_h, foot_E, root_dot, foot_dot)
            qdot[:, idir] = (q_h - q) / EPS

    qddot = np.zeros([6, len(q)])
    if dot_ndirs:
        qddot = np.zeros([6, dot_ndirs])
    return q, qdot, qddot


def minimal_trajectory(model):
    chest = BodyTrajectory(model.model, model.model.GetBodyId("pelvis"))
    rsole = BodyTrajectory(model.model, model.model.GetBodyId("ankle_right"))

    chest.set_trajectories(np.array([[0, 0, 0.6]]), np.array([[0, 0, 0]]))
    rsole.set_trajectories(np.array([[-0.2, -0.25, 0]]), np.array([[0, 0, 0]]))

    # Derivate in one arbitrary direction and by the unit matrix
    chest_dot = np.array([
        [[0.2, 1, 0, 0], [0.2, 0, 1, 0], [-0.1, 0, 0, 1]]
    ])
    rsole_dot = np.array([
        [[1, 0, 0, 0], [0.1, 0, 0, 0], [0.1, 0, 0, 0]]
    ])

    return chest, rsole, chest_dot, rsole_dot


def test_one_leg_consistency_first_order_derivative(model):
    chest, rsole, chest_dot, rsole_dot = minimal_trajectory(model)
    D, A, B, com_correction, __ = ik_constants(model, model.initial_pose_walking, chest)
    q_fd, qdot_fd, __ = ik_one_leg_fd(
        D, A, B,
        chest.traj_pos[0], chest.traj_ort[0],
        rsole.traj_pos[0], rsole.traj_ort[0],
        chest_dot[0], rsole_dot[0]
    )

    q, qdot, __ = ik_one_leg(
        D, A, B,
        chest.traj_pos[0], chest.traj_ort[0],
        rsole.traj_pos[0], rsole.traj_ort[0],
        chest_dot[0], rsole_dot[0],
        debug=True
    )

    print(qdot)
    print(qdot_fd)
    np.testing.assert_allclose(q_fd, q, rtol=RTOL, atol=ATOL)
    np.testing.assert_allclose(qdot_fd, qdot, rtol=RTOL, atol=ATOL)
