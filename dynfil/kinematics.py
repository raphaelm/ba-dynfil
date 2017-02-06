import numpy as np
import rbdl
import warnings
from scipy.signal import savgol_filter

from dynfil.utils.angles import euler_from_matrix, rotx, roty

Q_TYPES = [
    ('pelvis_x', float),
    ('pelvis_y', float),
    ('pelvis_z', float),
    ('pelvis_rx', float),
    ('pelvis_ry', float),
    ('pelvis_rz', float),
    ('r_hip_rz', float),
    ('r_hip_rx', float),
    ('r_hip_ry', float),
    ('r_knee_ry', float),
    ('r_ankle_ry', float),
    ('r_ankle_rx', float),
    ('l_hip_rz', float),
    ('l_hip_rx', float),
    ('l_hip_ry', float),
    ('l_knee_y', float),
    ('l_ankle_y', float),
    ('l_ankle_x', float),
]


class IKConvergenceWarning(UserWarning):
    pass


def com_trajectory(model, chest, q):
    com = np.zeros((len(q), 3))
    for t in range(len(q)):
        com_tmp = np.zeros(3)
        rbdl.CalcCenterOfMass(model, q[t], np.zeros(model.dof_count), com_tmp)
        com[t] = com_tmp
    return com


def move_chest_body_to_com(model, q_ini, chest, lsole, rsole, trials=10):
    """
    Move the body_point of our chest body. This is an approach to compensate
    for the fact that the real Center of Mass is *not* equal to the free-flyer
    while we treat it as such in most of our calculations.

    In this method, we iteratively calculate the *actual* CoM and then move the
    free-flyer's position there.
    """
    for i in range(trials):
        cs = rbdl.InverseKinematicsConstraintSet()
        cs.damper = 1e-4
        cs.AddFullConstraint(*chest.to_constraint(0))
        cs.AddFullConstraint(*lsole.to_constraint(0))
        cs.AddFullConstraint(*rsole.to_constraint(0))
        q_res = rbdl.InverseKinematics(model, q_ini, cs)
        q_ini = q_res

        com_tmp = np.zeros(3)
        rbdl.CalcCenterOfMass(model, q_ini, np.zeros(model.dof_count), com_tmp)
        chest.body_point = rbdl.CalcBaseToBodyCoordinates(model, q_ini, chest.id, com_tmp)


def inverse_numerical(model, q_ini, chest, lsole, rsole):
    """
    Calculate the Inverse kinematics. Returns trajectories for q.
    """
    if len(chest) != len(lsole) or len(chest) != len(rsole):
        raise ValueError('Trajectories are not of same length.')

    com_tmp = np.zeros(3)
    rbdl.CalcCenterOfMass(model, q_ini, np.zeros(model.dof_count), com_tmp)
    chest.body_point = rbdl.CalcBaseToBodyCoordinates(model, q_ini, chest.id, com_tmp)

    q = np.zeros((len(chest), model.qdot_size))
    for t in range(len(chest)):  # Iterate over timesteps
        q_before = q[t - 1] if t > 0 else q_ini

        #if t == 0:
        #    move_chest_body_to_com(model, q_before, chest, lsole, rsole)

        com_tmp = np.zeros(3)
        rbdl.CalcCenterOfMass(model, q_before, np.zeros(model.dof_count), com_tmp)
        chest.body_point = rbdl.CalcBaseToBodyCoordinates(model, q_before, chest.id, com_tmp)
        cs = rbdl.InverseKinematicsConstraintSet()
        cs.damper = 1e-4
        cs.AddFullConstraint(*chest.to_constraint(t))
        cs.AddFullConstraint(*lsole.to_constraint(t))
        cs.AddFullConstraint(*rsole.to_constraint(t))

        # TODO: cs will have member "e" with the residuum. plot and compare to tolerance

        # TODO check for convergence?
        q[t] = rbdl.InverseKinematics(model, q_before, cs)

        if (cs.e > 1e-12).any():
            warnings.warn("IK error > tolerance: {}".format(cs.e), IKConvergenceWarning)

    return q


def inverse(model, q_ini, chest, lsole, rsole, method='numerical'):
    """
    Like inverse(), but returns a tuple of (q, qdot, qddot)
    """
    if method == 'numerical':
        return inverse_numerical(model, q_ini, chest, lsole, rsole)
    else:
        return inverse_analytical(model, q_ini, chest, lsole, rsole)


def inverse_with_derivatives(model, q_ini, chest, lsole, rsole, times, method='numerical', interpolate='none'):
    """
    Like inverse(), but returns a tuple of (q, qdot, qddot)
    """
    if method == 'numerical':
        q = inverse_numerical(model, q_ini, chest, lsole, rsole)
    else:
        q = inverse_analytical(model, q_ini, chest, lsole, rsole)

    qdot = np.zeros((len(q), model.qdot_size))
    qddot = np.zeros((len(q), model.qdot_size))

    poly_range = 4
    poly_degree = 5
    savgol_window_size = 101
    savgol_poly_order = 3
    savgol_mode = "interp"

    for t in range(len(q)):  # Iterate over timesteps
        if t < poly_range or interpolate != 'poly':
            # First values: Take naive finite differences
            if t > 0 and times[t] > times[t - 1]:
                qdot[t] = (q[t] - q[t - 1]) / (times[t] - times[t - 1])
            else:
                qdot[t] = np.zeros(model.qdot_size)

        else:
            # Use polynomial interpolation between values
            p = np.polyfit(
                times[t - poly_range:t + poly_range],
                q[t - poly_range:t + poly_range],
                poly_degree
            )
            h = 0.0001

            qdot[t] = (np.polyval(p, times[t]) - np.polyval(p, times[t] - h)) / h

    for t in range(len(q)):
        if t < poly_range or interpolate != 'poly':
            # First values: Take naive finite differences
            if t > 1 and times[t] > times[t - 1]:
                qddot[t] = (qdot[t] - qdot[t - 1]) / (times[t] - times[t - 1])
            else:
                qddot[t] = np.zeros(model.qdot_size)

        else:
            # Use polynomial interpolation between values
            p = np.polyfit(
                times[t - poly_range:t + poly_range],
                qdot[t - poly_range:t + poly_range],
                poly_degree
            )
            h = 0.0001

            qddot[t] = (np.polyval(p, times[t]) - np.polyval(p, times[t] - h)) / h

    if interpolate == 'savgol':
        for i in range(len(qddot[0])):
            filtered = savgol_filter(qddot[:, i], savgol_window_size, savgol_poly_order, mode=savgol_mode)
            qddot[:, i] = filtered

    return q, qdot, qddot


def inverse_analytical_one_leg(D, A, B, root, foot, t):
    root_r = root.traj_pos[t]
    foot_r = foot.traj_pos[t]
    root_E = root.traj_ort[t]
    foot_E = foot.traj_ort[t]
    # Kajitas book, p 53
    r = foot_E.T.dot(root_r + root_E.dot(D) - foot_r)
    C = np.linalg.norm(r)
    c5 = (C**2 - A**2 - B**2) / (2.0*A*B)
    if c5 >= 1:
        q5 = 0.0
    elif c5 <= -1:
        q5 = np.pi
    else:
        q5 = np.arccos(c5)  # knee pitch

    q6a = np.arcsin((A/C)*np.sin(np.pi - q5))  # ankle pitch sub

    q7 = np.arctan2(r[1], r[2])  # ankle roll -pi/2 < q(6) < pi/2
    if q7 > np.pi/2.:
        q7 = q7 - np.pi
    elif q7 < -np.pi/2.:
        q7 = q7 + np.pi

    # ankle pitch
    q6 = -np.arctan2(r[0], np.sign(r[2])*np.sqrt(r[1]**2 + r[2]**2)) - q6a

    R = root_E.T.dot(foot_E).dot(rotx(-q7)).dot(roty(-q6 - q5))

    # hip yaw
    q2 = np.arctan2(-R[0, 1], R[1,1])

    # hip roll
    cz = np.cos(q2)
    sz = np.sin(q2)
    q3 = np.arctan2(R[2, 1], -R[0, 1]*sz + R[1, 1]*cz)

    # hip pitch
    q4 = np.arctan2(-R[2, 0], R[2, 2])

    q = np.array([q2, q3, q4, q5, q6, q7])
    qdot = np.array([0, 0, 0, 0, 0, 0])
    qddot = np.array([0, 0, 0, 0, 0, 0])
    return q, qdot, qddot


def inverse_analytical(model, q_ini, chest, lsole, rsole):
    q = np.zeros((len(chest), model.q_size))
    qdot = np.zeros((len(chest), model.qdot_size))
    qddot = np.zeros((len(chest), model.qdot_size))

    # TODO: Center of mass corrections!
    rbdl.UpdateKinematics(model, q_ini, np.zeros(model.qdot_size), np.zeros(model.qdot_size))

    root_x = model.X_base[model.GetBodyId("pelvis")]
    hipr_x = model.X_base[model.GetBodyId("hip_right")]
    kneer_x = model.X_base[model.GetBodyId("knee_right")]
    footr_x = model.X_base[model.GetBodyId("ankle_right")]
    D = hipr_x.r - root_x.r
    A = np.linalg.norm(hipr_x.r - kneer_x.r)
    B = np.linalg.norm(kneer_x.r - footr_x.r)

    for t in range(len(q)):

        lq, lqdot, lqddot = inverse_analytical_one_leg(
            -D, A, B, chest, lsole, t
        )
        rq, rqdot, rqddot = inverse_analytical_one_leg(
            D, A, B, chest, rsole, t
        )

        q[t, 0:3] = chest.traj_pos[t]
        qdot[t, 0:3] = chest.traj_pos_dot[t]
        qddot[t, 0:3] = chest.traj_pos_ddot[t]
        # TDOO: qdot[0:3]
        # TODO: qddot[0:3]
        # TODO get Euler angles from matrix => q[3:6]

        q[t, 6:12] = rq
        qdot[t, 6:12] = rqdot
        qddot[t, 6:12] = rqddot

        q[t, 12:18] = lq
        qdot[t, 12:18] = lqdot
        qddot[t, 12:18] = lqddot

    return q