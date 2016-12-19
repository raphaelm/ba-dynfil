import numpy as np
import rbdl


def inverse(model, q_ini, chest, lsole, rsole):
    """
    Calculate the Inverse kinematics. Returns trajectories for q.
    """
    if len(chest) != len(lsole) or len(chest) != len(rsole):
        raise ValueError('Trajectories are not of same length.')

    q = np.zeros((len(chest), model.qdot_size))
    for t in range(len(chest)):  # Iterate over timesteps
        # Calculate q using rbdl
        cs = rbdl.InverseKinematicsConstraintSet()
        cs.lmbda = 1e-4
        cs.AddFullConstraint(*chest.to_constraint(t))
        cs.AddFullConstraint(*lsole.to_constraint(t))
        cs.AddFullConstraint(*rsole.to_constraint(t))
        q[t] = rbdl.InverseKinematics(model, q[t - 1] if t > 0 else q_ini, cs)

    return q
