import numpy as np
import rbdl


def calculate_zmp_trajectory(model, q, times):
    """
    Calculate the ZMP trajectory. Corresponds to the CalculateZMP()
    algorithm in the thesis.
    """
    if len(times) != len(q):
        raise ValueError('Time is not of same length as trajectories.')

    prev_qdot = np.zeros(model.qdot_size)
    zmp = np.zeros((len(q), 3))

    for t in range(len(q)):  # Iterate over timesteps
        # Calculate qdot and qddot using finite differences
        if t > 0 and times[t] > times[t - 1]:
            qdot = (q[t] - q[t - 1]) / (times[t] - times[t - 1])
        else:
            qdot = np.zeros(model.qdot_size)

        if t > 1 and times[t] > times[t - 1]:
            qddot = (qdot - prev_qdot) / (times[t] - times[t - 1])
        else:
            qddot = np.zeros(model.qdot_size)

        # Calculate tau using rbdl
        tau = np.zeros(model.qdot_size)
        rbdl.InverseDynamics(model, q[t], qdot, qddot, tau)

        F_ext = tau[0:3]
        M_ext = tau[3:6]
        zmp[t] = 1 / (F_ext[2]) * np.array([-M_ext[1], M_ext[0], 0])

        prev_qdot = qdot

    return zmp
