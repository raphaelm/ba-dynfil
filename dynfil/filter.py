import numpy as np

from . import zmp, kinematics


def zmp_jacobians(model, zmp_ini, chest, lsole, rsole, q_ini, times):
    """
    Calculate the Jacobian of r_ZMP(c) for every timestep in the trajectory.
    """
    h = 1e-10

    # Allocate an array of jacobians (one for each timestep)
    jacobians = np.zeros((len(times), 3, 3))

    # Calculate finite differences for all three dimensions
    for dim in range(3):
        h_vec = np.zeros_like(chest.traj_pos)
        h_vec[:, dim] = h

        chest_modified = chest.copy()
        chest_modified.traj_pos += h_vec

        q_calc = kinematics.inverse(model, q_ini, chest_modified, lsole, rsole)
        zmp_calc_second = zmp.calculate_zmp_trajectory(model, q_calc, chest_modified,
                                                       times=times)

        jacobians[:, :, dim] = (zmp_calc_second - zmp_ini) / h

    return jacobians


def dynfil_newton_numerical(chest, lsole, rsole, zmp_ref, q_ini, model, times, iterations=5):
    """
    Applies the dynamic filter using Newton-Raphson iterations and numerical derivatives.
    (See Algorithm 2.2 in thesis)
    """
    chest = chest.copy()

    for i in range(iterations):

        q_calc = kinematics.inverse(model, q_ini, chest, lsole, rsole)
        zmp_calc = zmp.calculate_zmp_trajectory(model, q_calc, chest, times=times)
        zmp_diff = zmp_calc - zmp_ref

        # Calculate jacobians
        jacobians = zmp_jacobians(model, zmp_calc, chest, lsole, rsole, q_ini, times)

        for t in range(len(chest)):  # Iterate over timesteps
            # One Newton iteration:
            chest.traj_pos[t] -= np.dot(np.linalg.inv(jacobians[t]), zmp_diff[t])

    return chest.traj_pos
