import numpy as np
import rbdl
from scipy.signal import savgol_filter

from dynfil.utils.angles import euler_from_matrix


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

        if t == 0:
            move_chest_body_to_com(model, q_before, chest, lsole, rsole)

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


def inverse_analytical(model, q_ini, chest, lsole, rsole):
    q = np.recarray((len(chest),), dtype=[
        ('root_link_x', float),
        ('root_link_y', float),
        ('root_link_z', float),
        ('root_link_rx', float),
        ('root_link_ry', float),
        ('root_link_rz', float),
        ('l_hip_1', float),
        ('l_hip_2', float),
        ('l_upper_leg', float),
        ('l_lower_leg', float),
        ('l_ankle_1', float),
        ('l_ankle_2', float),
        ('r_hip_1', float),
        ('r_hip_2', float),
        ('r_upper_leg', float),
        ('r_lower_leg', float),
        ('r_ankle_1', float),
        ('r_ankle_2', float),
        ('torso_1', float),
        ('torso_2', float),
        ('chest', float),
    ])
    #move_chest_body_to_com(model, q_ini, chest, lsole, rsole)

    for t in range(len(q)):
        # TODO: Center of mass corrections!
        q[t].root_link_x = chest.traj_pos[t][0]
        q[t].root_link_y = chest.traj_pos[t][1]
        q[t].root_link_z = chest.traj_pos[t][2]

        q[t].root_link_rx, q[t].root_link_ry, q[t].root_link_rz = euler_from_matrix(chest.traj_ort[t], "123")

        #r = np.transpose(rsole.traj_ort[t]).dot( chest.traj_pos[t] + chest.traj_ort[t].dot(np.transpose(np.array([0, D, 0]))) - rsole.traj_pos[t])

        q[t].l_hip_1 = 0

    return q