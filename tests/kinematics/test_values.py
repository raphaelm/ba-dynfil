import numpy as np
import pytest
import rbdl

from dynfil import kinematics
from dynfil.models import SimpleModel

RTOL = 1e-5
ATOL = 1e-5
EPS = 1e-8


def test_numerical_vs_analytical(any_model_with_trajectory):
    model, traj = any_model_with_trajectory
    if not isinstance(model, SimpleModel):
        pytest.xfail('Cannot succeed due to modelling problems.')
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

    # TODO: This tolerance is very high.
    np.testing.assert_allclose(q_calc_a[2], q_calc_n[2], rtol=1, atol=.0001)
    np.testing.assert_allclose(q_calc_a[2, 3:], q_calc_n[2, 3:], rtol=.1, atol=.0001)


def test_end_effectors(any_model_with_trajectory, ik_method):
    model, traj = any_model_with_trajectory
    if not isinstance(model, SimpleModel) and ik_method == 'analytical':
        pytest.xfail('Cannot succeed due to modelling problems.')
    timesteps, chest, rsole, lsole = traj
    chest.traj_pos[0, 0:2] = 0  # Force y-symmetry to ease debugging
    q_calc_a, __, __ = kinematics.inverse_with_derivatives(
        model, model.initial_pose_walking, chest, lsole, rsole, timesteps,
        interpolate='none', method=ik_method
    )
    rbdl.UpdateKinematics(model.model, q_calc_a[0], np.zeros(model.qdot_size), np.zeros(model.qdot_size))

    print "leg1", q_calc_a[0][6:12]
    print "leg2", q_calc_a[0][12:18]

    if isinstance(model, SimpleModel):
        print "pelvis", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("pelvis"),
                                                       np.zeros(3))
        print "right_hip", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("hip_right"),
                                                          np.zeros(3))
        print "left_hip", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("hip_left"),
                                                         np.zeros(3))
        print "right_knee", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0],
                                                           model.model.GetBodyId("knee_right"), np.zeros(3))
        print "left_knee", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("knee_left"),
                                                          np.zeros(3))
        print "right_ankle", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0],
                                                            model.model.GetBodyId("ankle_right"), np.zeros(3))
        print "left_ankle", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0],
                                                           model.model.GetBodyId("ankle_left"), np.zeros(3))
        print "right_sole", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0],
                                                           model.model.GetBodyId("sole_right"), np.zeros(3))
        print "left_sole", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("sole_left"),
                                                          np.zeros(3))
    else:
        from mpl_toolkits.mplot3d import Axes3D
        import matplotlib.pyplot as plt

        lines = [
            [
                'root_link',
                'l_hip_1', 'l_hip_2', 'l_upper_leg', 'l_lower_leg', 'l_ankle_1', 'l_ankle_2', 'l_sole',
            ],
            [
                'root_link',
                'r_hip_1', 'r_hip_2', 'r_upper_leg', 'r_lower_leg', 'r_ankle_1', 'r_ankle_2', 'r_sole',
            ]
        ]
        fig = plt.figure(figsize=(11.69, 8.27))
        ax = fig.gca(projection='3d')
        ax.axis('equal')
        for line in lines:
            points = np.zeros((len(line), 3))

            for i, b in enumerate(line):
                p = rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId(b), np.zeros(3))
                print b, p
                points[i, :] = p

            ax.plot(points[:, 0], points[:, 1], points[:, 2], markersize=3, marker='o')
        #plt.show()

    rsole_pos = rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], rsole.id, np.zeros(3))
    np.testing.assert_allclose(rsole_pos, rsole.traj_pos[0], rtol=RTOL, atol=ATOL)

    lsole_pos = rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], lsole.id, np.zeros(3))
    np.testing.assert_allclose(lsole_pos, lsole.traj_pos[0], rtol=RTOL, atol=ATOL)
