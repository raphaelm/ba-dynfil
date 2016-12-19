import numpy as np
import rbdl


def calculate_zmp_trajectory(model, q_ini, chest, lsole, rsole, times):
    """
    Calculate the ZMP trajectory. Corresponds to the CalculateZMP()
    algorithm in the thesis.
    """
    if len(chest) != len(lsole) or len(chest) != len(rsole):
        raise ValueError('Trajectories are not of same length.')
    if len(times) != len(chest):
        raise ValueError('Time is not of same length as trajectories.')

    prev_q = q_ini[:]
    prev_qdot = np.zeros(model.qdot_size)
    zmp = np.zeros((len(chest), 3))

    for t in range(len(chest)):  # Iterate over timesteps

        # Calculate q using rbdl
        cs = rbdl.InverseKinematicsConstraintSet()
        cs.lmbda = 1e-4
        cs.AddFullConstraint(*chest.to_constraint(t))
        cs.AddFullConstraint(*lsole.to_constraint(t))
        cs.AddFullConstraint(*rsole.to_constraint(t))
        q = rbdl.InverseKinematics(model, prev_q, cs)

        # Calculate qdot and qddot using finite differences
        if t > 0 and times[t] > times[t - 1]:
            qdot = (q - prev_q) / (times[t] - times[t - 1])
        else:
            qdot = np.zeros(model.qdot_size)

        if t > 1 and times[t] > times[t - 1]:
            qddot = (qdot - prev_qdot) / (times[t] - times[t - 1])
        else:
            qddot = np.zeros(model.qdot_size)

        # Calculate tau using rbdl
        tau = np.zeros(model.qdot_size)
        rbdl.InverseDynamics(model, q, qdot, qddot, tau)

        F_ext = tau[0:3]
        M_ext = tau[3:6]
        zmp[t] = 1 / (F_ext[2]) * np.array([-M_ext[1], M_ext[0], 0])

        prev_q = q
        prev_qdot = qdot

    return zmp
