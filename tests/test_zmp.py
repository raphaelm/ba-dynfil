import numpy as np
import rbdl

from dynfil import constants
from dynfil.bodies import BodyTrajectory
from dynfil.zmp import calculate_zmp_trajectory


def test_zmp_below_com_in_static_pose(model):
    q = np.array([constants.POSE_HALF_SITTING])
    qdot = np.zeros((1, model.dof_count))
    qddot = np.zeros((1, model.dof_count))
    chest = BodyTrajectory(model, model.GetBodyId("chest"))

    zmp = calculate_zmp_trajectory(model, q, qdot, qddot, chest)[0]
    com = np.zeros(3)
    rbdl.CalcCenterOfMass(model, q[0], np.zeros(model.dof_count), com)

    np.testing.assert_allclose(zmp[0:2], com[0:2], rtol=1e-03)
