import numpy as np
import rbdl


def calculate_zmp_trajectory(model, q, qdot, qddot, chest):
    """
    Calculate the ZMP trajectory. Corresponds to the CalculateZMP()
    algorithm in the thesis.
    """
    zmp = np.zeros((len(q), 3))

    for t in range(len(q)):  # Iterate over timesteps
        # Calculate tau using rbdl
        tau = np.zeros(model.qdot_size)
        # TODO check if kinematics are updated?
        rbdl.InverseDynamics(model, q[t], qdot[t], qddot[t], tau)

        # NOTE this only works when the first joint is a floating base joint
        F_ext = tau[0:3]
        M_ext = tau[3:6]

        # Calculate ZMP relative to CoM
        _zmp = 1 / (F_ext[2]) * np.array([-M_ext[1], M_ext[0], 0])

        # Calculate CoM position and transform to ZMP world coordinate
        chest_pos = rbdl.CalcBodyToBaseCoordinates (
            model, q[t], chest.id, chest.body_point
        )

        # TODO is this correct?
        _zmp += chest_pos
        _zmp[2] = 0
        zmp[t] = _zmp

    return zmp
