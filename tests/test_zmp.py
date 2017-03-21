import numpy as np
import rbdl

from dynfil import constants
from dynfil.bodies import BodyTrajectory
from dynfil.zmp import calculate_zmp_trajectory, calculate_real_zmp_trajectory
from tests.kinematics import minimal_trajectory


def test_zmp_below_com_in_static_pose(model):
    q = np.array([model.initial_pose_walking])
    qdot = np.zeros((1, model.dof_count))
    qddot = np.zeros((1, model.dof_count))
    chest = BodyTrajectory(model, model.model.GetBodyId("pelvis"))

    zmp = calculate_zmp_trajectory(model, q, qdot, qddot, chest)[0]
    com = np.zeros(3)
    rbdl.CalcCenterOfMass(model.model, q[0], np.zeros(model.dof_count), com)

    np.testing.assert_allclose(zmp[0:2], com[0:2], rtol=1e-05)


def test_real_zmp_below_com_in_static_pose(model):
    t, chest, lfoot, rfoot = minimal_trajectory(model, 'test_traj_simple.txt')
    q = np.array([model.initial_pose_walking])
    qdot = np.zeros((1, model.dof_count))
    qddot = np.zeros((1, model.dof_count))

    zmp = calculate_real_zmp_trajectory(model, q, qdot, qddot, chest, lfoot, rfoot)[0]
    com = np.zeros(3)
    rbdl.CalcCenterOfMass(model.model, q[0], np.zeros(model.dof_count), com)

    np.testing.assert_allclose(zmp[0:2], com[0:2], rtol=1e-05, atol=1e-02)
