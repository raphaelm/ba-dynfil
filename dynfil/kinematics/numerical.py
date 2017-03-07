import numpy as np
import rbdl
import warnings


class IKConvergenceWarning(UserWarning):
    pass


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


def ik_trajectory(model, q_ini, chest, lsole, rsole):
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

        if (cs.e > 1e-12).any():
            warnings.warn("IK error > tolerance: {}".format(cs.e), IKConvergenceWarning)

    return q


