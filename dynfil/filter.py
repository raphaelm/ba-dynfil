import numpy as np

from . import zmp, kinematics


def zmp_jacobians(model, zmp_ini, chest, lsole, rsole, q_ini, times):
    """
    Calculate the Jacobian of r_ZMP(c) for every timestep in the trajectory.
    """
    h = 1e-10

    # Allocate an array of jacobians (one for each timestep)
    jacobians = np.zeros((len(times), 2, 2))

    # We only do this in 2D here
    zmp_ini = zmp_ini[:, 0:2]

    # Calculate finite differences for both dimensions
    for dim in range(2):
        h_vec = np.zeros_like(chest.traj_pos)
        h_vec[:, dim] = h

        chest_modified = chest.copy()
        chest_modified.traj_pos += h_vec

        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest_modified, lsole, rsole, times
        )
        zmp_calc_second = zmp.calculate_zmp_trajectory(
            model, q_calc, qdot_calc, qddot_calc, chest_modified
        )[:, 0:2]

        jacobians[:, :, dim] = (zmp_calc_second - zmp_ini) / h

    return jacobians


def dynfil_newton_numerical(chest, lsole, rsole, zmp_ref, q_ini, model, times, iterations=5):
    """
    Applies the dynamic filter using Newton-Raphson iterations and numerical derivatives.
    (See Algorithm 2.2 in thesis)
    """
    chest = chest.copy()

    for i in range(iterations):

        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, times
        )
        zmp_calc = zmp.calculate_zmp_trajectory(
            model, q_calc, qdot_calc, qddot_calc, chest
        )
        zmp_diff = zmp_calc - zmp_ref

        # Calculate jacobians
        jacobians = zmp_jacobians(model, zmp_calc, chest, lsole, rsole, q_ini, times)

        for t in range(len(chest)):  # Iterate over timesteps
            # One Newton iteration:
            diffxy = np.dot(np.linalg.inv(jacobians[t]), zmp_diff[t, 0:2])
            chest.traj_pos[t] -= np.array([diffxy[0], diffxy[1], 0])

    return chest


def dynfil_least_squares(chest, lsole, rsole, zmp_ref, q_ini, model, times, iterations=5):
    """
    Applies the dynamic filter using Gauss-Newton minimization
    """
    chest = chest.copy()

    for i in range(iterations):
        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, times
        )
        zmp_calc = zmp.calculate_zmp_trajectory(
            model, q_calc, qdot_calc, qddot_calc, chest
        )
        zmp_diff = zmp_calc - zmp_ref

        # Calculate jacobians
        jacobians = zmp_jacobians(model, zmp_calc, chest, lsole, rsole, q_ini, times)

        for t in range(len(chest)):  # Iterate over timesteps
            # One Gauss-Newton iteration: x -= (A^T A)^-1 A^T b
            # with A = jacobian, b = zmp_diff
            diffxy = np.dot(
                np.dot(
                    np.linalg.inv(
                        np.dot(
                            np.transpose(jacobians[t]),
                            jacobians[t]
                        )
                    ),
                    np.transpose(jacobians[t])
                ),
                zmp_diff[t, 0:2]
            )
            chest.traj_pos[t] -= np.array([diffxy[0], diffxy[1], 0])

    return chest