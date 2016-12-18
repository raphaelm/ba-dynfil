import rbdl
import numpy as np


def calculate_zmp_trajectory(model, q_ini, chest, lsole, rsole, sampling_interval):
    """
    Calculate the ZMP trajectory. Corresponds to the CalculateZMP()
    algorithm in the thesis.
    """
    if len(chest) != len(lsole) or len(chest) != len(rsole):
        raise ValueError('Trajectories are not of same length.')

    prev_q = q_ini[:]
    prev_qdot = np.zeros(model.qdot_size)
    zmp = np.zeros(len(chest))

    for t in range(len(chest)):  # Iterate over timesteps

        # Calculate q using rbdl
        cs = rbdl.InverseKinematicsConstraintSet()
        cs.lmbda = 1e-4
        cs.AddFullConstraint(*chest.to_constraint(t))
        cs.AddFullConstraint(*lsole.to_constraint(t))
        cs.AddFullConstraint(*rsole.to_constraint(t))
        q = rbdl.InverseKinematics(model, prev_q, cs)

        # Calculate qdot and qddot using finite differences
        if t > 0:
            qdot = (q - prev_q) / sampling_interval
        else:
            qdot = np.zeros(model.qdot_size)

        if t > 1:
            qddot = (qdot - prev_qdot) / sampling_interval
        else:
            qddot = np.zeros(model.qdot_size)

        # Calculate tau using rbdl
        tau = np.zeros(model.qdot_size)
        rbdl.InverseDynamics(model, q, qdot, qddot, tau)

        # TODO: Calculate zmp from tau

        prev_q = q
        prev_qdot = qdot

    return zmp
