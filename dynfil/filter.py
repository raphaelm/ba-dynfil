import numpy as np

from dynfil.kinematics import interpolate_savgol
from .previewcontrol import online_preview_control
from . import zmp, kinematics


def zmp_jacobians(model, zmp_ini, chest, lsole, rsole, q_ini, times, ik_method):
    """
    Calculate the Jacobian of r_ZMP(c) for every timestep in the trajectory.
    """
    h = 1e-10

    # Allocate an array of Jacobians (one for each timestep)
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
            model, q_ini, chest_modified, lsole, rsole, times, method=ik_method
        )
        zmp_calc_second = zmp.calculate_zmp_trajectory(
            model, q_calc, qdot_calc, qddot_calc, chest_modified
        )[:, 0:2]

        jacobians[:, :, dim] = (zmp_calc_second - zmp_ini) / h

    return jacobians


def update_derivs(chest, times):
    for t in range(len(chest)):
        if t > 0:
            chest.traj_pos_dot[t] = (chest.traj_pos[t] - chest.traj_pos[t - 1]) / (times[t] - times[t - 1])
        if t > 1:
            chest.traj_pos_ddot[t] = (chest.traj_pos_dot[t] - chest.traj_pos_dot[t - 1]) / (times[t] - times[t - 1])


def dynfil_newton_numerical(chest, lsole, rsole, zmp_ref, q_ini, model, times,
                            ik_method='numerical'):
    """
    Applies the dynamic filter using Newton-Raphson iterations and numerical derivatives.
    (See Algorithm 2.2 in thesis)
    """
    q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
        model, q_ini, chest, lsole, rsole, times, method=ik_method
    )
    zmp_calc = zmp.calculate_zmp_trajectory(
        model, q_calc, qdot_calc, qddot_calc, chest
    )
    zmp_diff = zmp_calc - zmp_ref

    # Calculate jacobians
    jacobians = zmp_jacobians(model, zmp_calc, chest, lsole, rsole, q_ini, times, ik_method)

    for t in range(len(chest)):  # Iterate over timesteps
        diffxy = np.dot(np.linalg.inv(jacobians[t]), zmp_diff[t, 0:2])
        chest.traj_pos[t] -= np.array([diffxy[0], diffxy[1], 0])
    return chest


def dynfil_least_squares(chest, lsole, rsole, zmp_ref, q_ini, model, times, ik_method):
    """
    Applies the dynamic filter using Gauss-Newton minimization
    """
    def gauss_newton_r(chest_old, chest_new):
        r = np.zeros((len(chest_old), 2))
        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest_new, lsole, rsole, times, method=ik_method
        )
        zmp_calc_new = zmp.calculate_zmp_trajectory(
            model, q_calc, qdot_calc, qddot_calc, chest_new
        )
        zmp_diff = zmp_calc_new - zmp_ref
        com_diff = np.vstack((np.diff(chest_new.traj_pos, axis=0), np.array([[0, 0, 0]])))
        delta = 100
        r[:, 0:2] = zmp_diff[:, 0:2]
        #r[:, 2:4] = np.sqrt(delta) * com_diff[:, 0:2]
        return r

    def min_jacobians(min_ini, chest):
        h = 1e-10
        # Allocate an array of Jacobians (one for each timestep)
        jacobians = np.zeros((len(times), 2, 2))

        for dim in range(2):
            h_vec = np.zeros_like(chest.traj_pos)
            h_vec[:, dim] = h

            chest_modified = chest.copy()
            chest_modified.traj_pos += h_vec

            jacobians[:, :, dim] = (gauss_newton_r(chest, chest_modified) - min_ini) / h
        return jacobians

    min_ini = gauss_newton_r(chest, chest)

    # Calculate jacobians
    jacobians = min_jacobians(min_ini, chest)

    for t in range(len(chest)):  # Iterate over timesteps
        # One Gauss-Newton iteration: x -= (A^T A)^-1 A^T b
        # with A = jacobian, b = zmp_diff
        # TODO try np.pinv for Moore-Penrose pseudo inverse
        # TODO add Levenberg-Marquardt in case of rank insufficiencies
        diffxy = np.linalg.inv(
            jacobians[t].T.dot(jacobians[t])
        ).dot(jacobians[t].T).dot(min_ini[t])
        chest.traj_pos[t] -= np.array([diffxy[0], diffxy[1], 0])

    return chest


def dynfil_gradient_descent(chest, lsole, rsole, zmp_ref, q_ini, model, times, ik_method):
    """
    Applies the dynamic filter using steepest descent minimization
    """
    q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
        model, q_ini, chest, lsole, rsole, times
    )
    zmp_calc = zmp.calculate_zmp_trajectory(
        model, q_calc, qdot_calc, qddot_calc, chest
    )
    zmp_diff = zmp_calc - zmp_ref

    # Calculate jacobians
    jacobians = zmp_jacobians(model, zmp_calc, chest, lsole, rsole, q_ini, times, ik_method)
    gradients = np.zeros((len(times), 2))

    for t in range(len(chest)):  # Iterate over timesteps
        # Generate gradient from linear problem, see
        # https://en.wikipedia.org/wiki/Gradient_descent#Solution_of_a_linear_system
        gradients[t][:] = 2 * np.dot(
            np.transpose(jacobians[t]),
            np.dot(jacobians[t], chest.traj_pos[t, 0:2]) - zmp_diff[t, 0:2]
        )

    for t in range(2, len(chest)):  # Iterate over timesteps
        # FIXME PROBLEM here is non-convexity!
        # Gamma from Barzilai-Borwein method, see
        # https://en.wikipedia.org/wiki/Gradient_descent#Description
        gamma = (
            np.dot(np.transpose(zmp_diff[t, 0:2] - zmp_diff[t - 1, 0:2]), (gradients[t] - gradients[t - 1])) /
            np.linalg.norm(gradients[t] - gradients[t - 1])
        )
        diffxy = gamma * gradients[t]
        chest.traj_pos[t] -= np.array([diffxy[0], diffxy[1], 0])

    return chest


def dynfil_preview_control(chest, lsole, rsole, zmp_ref, q_ini, model, times,
                           ik_method):
    """
    Applies the dynamic filter using preview control. Notation from Kajita book (2004),
    page 143 ff. Helpful links:

    * http://www.mwm.im/lqr-controllers-with-python/
    * https://de.mathworks.com/help/control/ref/dlqr.html?requestedDomain=www.mathworks.com
    """
    z_c = chest.traj_pos[0, 2]
    t_step = np.mean(np.diff(times))

    # Calculate error
    q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
        model, q_ini, chest, lsole, rsole, times, method=ik_method
    )
    zmp_calc = zmp.calculate_zmp_trajectory(
        model, q_calc, qdot_calc, qddot_calc, chest
    )
    zmp_diff = zmp_calc - zmp_ref

    filter_traj = online_preview_control(zmp_diff, t_step, z_c, len(chest), window=0.8)
    chest.traj_pos[:, 0:2] -= filter_traj[0][:, 0:2]
    chest.traj_pos_dot -= filter_traj[1]
    chest.traj_pos_ddot -= filter_traj[2]

    return chest


filters = {
    'leastsquares': dynfil_least_squares,
    'newton': dynfil_newton_numerical,
    'steepestdescent': dynfil_gradient_descent,
    'pc': dynfil_preview_control,
}
