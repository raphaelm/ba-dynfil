import numpy as np
import rbdl


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
        cs.lmbda = 1e-4
        cs.AddFullConstraint(*chest.to_constraint(0))
        cs.AddFullConstraint(*lsole.to_constraint(0))
        cs.AddFullConstraint(*rsole.to_constraint(0))
        q_res = rbdl.InverseKinematics(model, q_ini, cs)
        q_ini = q_res

        com_tmp = np.zeros(3)
        rbdl.CalcCenterOfMass(model, q_ini, np.zeros(model.dof_count), com_tmp)
        com_real = rbdl.CalcBaseToBodyCoordinates(model, q_ini, chest.id, com_tmp)
        chest.body_point = com_real


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

        move_chest_body_to_com(model, q_ini, chest, lsole, rsole)

        cs.AddFullConstraint(*chest.to_constraint(t))
        cs.AddFullConstraint(*lsole.to_constraint(t))
        cs.AddFullConstraint(*rsole.to_constraint(t))
        # TODO check for convergence?
        q[t] = rbdl.InverseKinematics(model, q[t - 1] if t > 0 else q_ini, cs)

    return q


def inverse_with_derivatives(model, q_ini, chest, lsole, rsole, times):
    """
    Like inverse(), but returns a tuple of (q, qdot, qddot)
    """
    q = inverse(model, q_ini, chest, lsole, rsole)

    qdot = np.zeros((len(q), model.qdot_size))
    qddot = np.zeros((len(q), model.qdot_size))

    for t in range(len(q)):  # Iterate over timesteps
        # Calculate qdot and qddot using finite differences
        if t > 0 and times[t] > times[t - 1]:
            qdot[t] = (q[t] - q[t - 1]) / (times[t] - times[t - 1])
        else:
            qdot[t] = np.zeros(model.qdot_size)

        if t > 1 and times[t] > times[t - 1]:
            qddot[t] = (qdot[t] - qdot[t - 1]) / (times[t] - times[t - 1])
        else:
            qddot[t] = np.zeros(model.qdot_size)

    return q, qdot, qddot