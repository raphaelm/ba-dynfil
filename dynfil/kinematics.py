import numpy as np
import rbdl


def get_com(model, q_ini, chest, lsole, rsole, trials=10):
    """Iterative determination of CoM using inverse kinematics."""
    Center_of_Mass = CS.AddFullConstraint(model.GetBodyId("chest"), body_points[2], com_pos, c_ort);
    for i in range(trials):
        cs = rbdl.InverseKinematicsConstraintSet()
        cs.lmbda = 1e-4
        cs.AddFullConstraint(*chest.to_constraint(0))
        cs.AddFullConstraint(*lsole.to_constraint(0))
        cs.AddFullConstraint(*rsole.to_constraint(0))
        q_res = rbdl.InverseKinematics(model, q_ini, cs)
        q_ini[:] = q_res;

        com_tmp = np.zeros(3)
        rbdl.CalcCenterOfMass(model, q_ini, np.zeros(model.dof_count), com_tmp)
        # rbdl.CalcBaseToBodyCoordinates(com)

        com_real = com_temp;
        # desired_pose[i].body_points[Center_of_Mass] = CalcBaseToBodyCoordinates(model,qinit,model.GetBodyId("chest"),com_real);


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

        com = get_com(model, q_ini, chest[:1], lsole[:1], rsole[:1])
        # cs_chest = chest.to_constraint(t)
        chest.body_point = com
        cs.AddFullConstraint(*chest.to_constraint(t))
        cs.AddFullConstraint(*lsole.to_constraint(t))
        cs.AddFullConstraint(*rsole.to_constraint(t))
        # TODO check for convergence?
        q[t] = rbdl.InverseKinematics(model, q[t - 1] if t > 0 else q_ini, cs)

    return q
