import numpy as np
import rbdl

from dynfil import kinematics
from dynfil.models import SimpleModel

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

    # TODO: This tolerance is very high.
    np.testing.assert_allclose(q_calc_a[0], q_calc_n[0], rtol=.1, atol=.0001)


def test_end_effectors(any_model_with_trajectory, ik_method):
    model, traj = any_model_with_trajectory
    timesteps, chest, rsole, lsole = traj
    q_calc_a, __, __ = kinematics.inverse_with_derivatives(
        model, model.initial_pose_walking, chest, lsole, rsole, timesteps,
        interpolate='none', method=ik_method
    )
    rbdl.UpdateKinematics(model.model, q_calc_a[0], np.zeros(model.qdot_size), np.zeros(model.qdot_size))

    print "q", q_calc_a[0]
    if isinstance(model, SimpleModel):
        print "pelvis", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("pelvis"), np.zeros(3))
        print "right_hip", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("hip_right"), np.zeros(3))
        print "left_hip", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("hip_left"), np.zeros(3))
        print "right_knee", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("knee_right"), np.zeros(3))
        print "left_knee", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("knee_left"), np.zeros(3))
        print "right_ankle", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("ankle_right"), np.zeros(3))
        print "left_ankle", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("ankle_left"), np.zeros(3))
        print "right_sole", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("sole_right"), np.zeros(3))
        print "left_sole", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("sole_left"), np.zeros(3))
    else:
        print "root_link", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("root_link"), np.zeros(3))
        print "l_hip_1", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("l_hip_1"), np.zeros(3))
        print "r_hip_1", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("r_hip_1"), np.zeros(3))
        print "l_hip_2", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("l_hip_2"), np.zeros(3))
        print "r_hip_2", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("r_hip_2"), np.zeros(3))
        print "l_upper_leg", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("l_upper_leg"), np.zeros(3))
        print "r_upper_leg", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("r_upper_leg"), np.zeros(3))
        print "l_lower_leg", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("l_lower_leg"), np.zeros(3))
        print "r_lower_leg", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("r_lower_leg"), np.zeros(3))
        print "l_ankle_1", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("l_ankle_1"), np.zeros(3))
        print "r_ankle_1", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("r_ankle_1"), np.zeros(3))
        print "l_ankle_2", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("l_ankle_2"), np.zeros(3))
        print "r_ankle_2", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("r_ankle_2"), np.zeros(3))
        print "l_sole", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("l_sole"), np.zeros(3))
        print "r_sole", rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], model.model.GetBodyId("r_sole"), np.zeros(3))

    rsole_pos = rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], rsole.id, np.zeros(3))
    np.testing.assert_allclose(rsole_pos, rsole.traj_pos[0], rtol=RTOL, atol=ATOL)

    lsole_pos = rbdl.CalcBodyToBaseCoordinates(model.model, q_calc_a[0], lsole.id, np.zeros(3))
    np.testing.assert_allclose(lsole_pos, lsole.traj_pos[0], rtol=RTOL, atol=ATOL)
