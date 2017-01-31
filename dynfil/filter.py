import numpy as np
import scipy.linalg

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


def dynfil_newton_numerical(chest, lsole, rsole, zmp_ref, q_ini, model, times, iterations=5,
                            ik_method='numerical', status_update=lambda i, n: None):
    """
    Applies the dynamic filter using Newton-Raphson iterations and numerical derivatives.
    (See Algorithm 2.2 in thesis)
    """
    chest = chest.copy()

    for i in range(iterations):
        status_update(i + 1, iterations)

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
            # One Newton iteration:
            # TODO check how inverse is computed?
            #      good choice could be QR decomposition
            # TODO check rank
            # TODO add Levenberg-Marquardt in case of rank insufficiencies
            diffxy = np.dot(np.linalg.inv(jacobians[t]), zmp_diff[t, 0:2])
            chest.traj_pos[t] -= np.array([diffxy[0], diffxy[1], 0])

    return chest


def dynfil_least_squares(chest, lsole, rsole, zmp_ref, q_ini, model, times,
                         ik_method, iterations=5,
                         status_update=lambda i, n: None):
    """
    Applies the dynamic filter using Gauss-Newton minimization
    """
    chest = chest.copy()

    for i in range(iterations):
        status_update(i + 1, iterations)

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
            # One Gauss-Newton iteration: x -= (A^T A)^-1 A^T b
            # with A = jacobian, b = zmp_diff
            # TODO try np.pinv for Moore-Penrose pseudo inverse
            # TODO check rank of matrix
            # TODO add Levenberg-Marquardt in case of rank insufficiencies
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


def dynfil_gradient_descent(chest, lsole, rsole, zmp_ref, q_ini, model, times,
                            ik_method, iterations=5,
                            status_update=lambda i, n: None):
    """
    Applies the dynamic filter using steepest descent minimization
    """
    chest = chest.copy()
    for i in range(iterations):
        status_update(i + 1, iterations)

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
                           ik_method, iterations=5, status_update=lambda i, n: None):
    """
    Applies the dynamic filter using preview control. Notation from Kajita book (2004),
    page 143 ff. Helpful links:

    * http://www.mwm.im/lqr-controllers-with-python/
    * https://de.mathworks.com/help/control/ref/dlqr.html?requestedDomain=www.mathworks.com
    """
    chest = chest.copy()

    # Weights taken from Kajita paper (2014), fig. 6
    Q = np.eye(3)
    R = 1e-6 * np.eye(1)

    # Definitions (p. 143-144 from Kajita's book)
    z_c = chest.traj_pos[0, 2]
    t_step = np.mean(np.diff(times))
    A = np.matrix([
        [1, t_step, t_step ** 2 / 2],
        [0, 1, t_step],
        [0, 0, 1]
    ])
    b = np.matrix([[t_step ** 3 / 6, t_step ** 2 / 2, t_step]]).T
    c = np.matrix([[1, 0, -z_c / 9.81]])

    # Size of preview window in steps, taken from jrl-walkgen AnalyticalMorisawaCompact.cpp:834
    N = int(0.8 / t_step)

    # Solve Ricatti equation, see links above and footnote 16 on p. 144 in Kajita's book
    P = np.matrix(scipy.linalg.solve_discrete_are(A, b, Q, R))

    # Compute LQR gain
    K = np.matrix(scipy.linalg.inv(b.T * P * b + R) * (b.T * P * A))

    for iteration in range(iterations):
        status_update(iteration + 1, iterations)

        # Calculate error
        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, times, method=ik_method
        )
        zmp_calc = zmp.calculate_zmp_trajectory(
            model, q_calc, qdot_calc, qddot_calc, chest
        )
        zmp_diff = zmp_calc - zmp_ref

        x = np.matrix([[0, 0, 0]]).T  # com, \dot com, \ddot com -- Initial values
        y = np.matrix([[0, 0, 0]]).T  # com, \dot com, \ddot com -- Initial values

        # Calculate weights
        f = np.zeros(N)
        for i in range(N):
            # eq (4.75) from p. 144 in Kajita's book
            f[i] = 1 / (R + b.T * P * b) * (b.T * (A - b * K).T ** i * c.T) * Q[0, 0]

        for t in range(len(chest)):
            # Limit preview window as we cannot access values after the end of the motion
            preview_size = min(N, len(chest) - t - 1)

            u = np.zeros(2)
            for i in range(2):
                # eq (4.74) from p. 144 in Kajita's book
                #u[i] = - K.dot(zmp_diff[t, i])[0,0] + f[:preview_size].dot(zmp_ref[t + 1:t + preview_size + 1, i].T)
                u[i] = - K.dot(x) + f[:preview_size].dot(zmp_diff[t + 1:t + preview_size + 1, i].T)

            x = A.dot(x) + b * u[0]
            y = A.dot(y) + b * u[1]

            chest.traj_pos[t][0] -= x[0, 0]
            chest.traj_pos[t][1] -= y[0, 0]

    return chest
