import numpy as np
import rbdl

from dynfil import kinematics

RTOL = 1e-5
ATOL = 1e-5
EPS = 1e-8


def test_numerical_vs_analytical(any_model_with_trajectory):
    model, traj = any_model_with_trajectory
    timesteps, chest, rsole, lsole = traj
    q_calc_a, __, __ = kinematics.inverse_with_derivatives(
        model, model.initial_pose_walking, chest, lsole, rsole, timesteps,
        interpolate='none', method='analytical'
    )

    q_calc_n, __, __ = kinematics.inverse_with_derivatives(
        model, q_calc_a[0], chest, lsole, rsole, timesteps,
        interpolate='none', method='numerical'
    )

    print(np.vstack((q_calc_a[0], q_calc_n[0])))

    np.testing.assert_allclose(q_calc_a[0], q_calc_n[0], rtol=RTOL, atol=ATOL)


def test_end_effectors(any_model_with_trajectory, ik_method):
    model, traj = any_model_with_trajectory
    timesteps, chest, rsole, lsole = traj
    q_calc_a, __, __ = kinematics.inverse_with_derivatives(
        model, model.initial_pose_walking, chest, lsole, rsole, timesteps,
        interpolate='none', method=ik_method
    )

    lsole_pos = rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], lsole.id, np.zeros(3))
    np.testing.assert_allclose(lsole_pos, lsole.traj_pos[0], rtol=RTOL, atol=ATOL)
