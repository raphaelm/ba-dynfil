import numpy as np
import scipy.linalg


def calculate_weights(A, b, c, N, Qe, R):
    # Stacked matrix from eq. (8) of Katayama paper, def. eq. (15)
    Atilde = np.vstack((
        np.hstack((np.eye(1), c.dot(A))),
        np.hstack((np.zeros((3, 1)), A))
    ))

    # Stacked vector from eq. (8) of Katayama paper, def. eq. (15)
    Btilde = np.vstack((
        c.dot(b),
        b
    ))

    # Qtilde matrix in form of eq. (15) of Katayama paper
    Qtilde = np.diag((Qe, 0, 0, 0))

    # Solve Algebraic Ricatty Equation, see eq. (18) of Katayama paper
    K = scipy.linalg.solve_discrete_are(Atilde, Btilde, Qtilde, R)

    # Compute weights (17a) and (17b) of Katayama paper in a single iteration
    Ghelpinv = np.linalg.inv(R + Btilde.T.dot(K.dot(Btilde)))[0, 0]
    Gix = Ghelpinv * Btilde.T.dot(K).dot(Atilde)
    Ge = Gix[0, 0]
    Gx = Gix[0, 1:4]

    # Initial condition from eq. (17c) of Katayama paper
    Gd = np.zeros(N)
    Gd[0] = - Ge

    # Definition from eq. (20) of Katayama paper
    Atilde_c = Atilde - Btilde.dot(Gix)

    # Initial condition from eq. (19) of Katayama paper
    Xprev = - Atilde_c.T.dot(K).dot(np.matrix([[1, 0, 0, 0]]).T)

    # Precompute weights for Gd
    for i in range(1, N):
        # Iteration from eq. (17d)
        Gd[i] = Ghelpinv * Btilde.T.dot(Xprev)[0, 0]
        # Iteration from eq. (19)
        Xprev = Atilde_c.T.dot(Xprev)

    return Ge, Gx, Gd


def online_preview_control(zmp_ref, t_step, z_c, traj_length):
    """
    Preview control algorithm, one iteration. Notation from Kajita book (2004), page 143 ff and
    Katayama paper, p. 679ff.

    Helpful links:

    * http://www.mwm.im/lqr-controllers-with-python/
    * https://de.mathworks.com/help/control/ref/dlqr.html?requestedDomain=www.mathworks.com
    * https://github.com/a-price/hubomz/blob/be60154a07381b52f1daecc56ffa40e1a598fe17/python/ZmpPreview.py
    """

    # Size of preview window in steps
    N = int(2.5 / t_step)

    # Weights taken from Kajita paper (2014), fig. 6
    Qe = 1
    R = 1e-6 * np.eye(1)

    # Definitions (p. 143-144 from Kajita's book, eq. (1)+(2) Katayama paper)
    A = np.matrix([
        [1, t_step, t_step ** 2 / 2],
        [0, 1, t_step],
        [0, 0, 1]
    ])
    b = np.matrix([[t_step ** 3 / 6, t_step ** 2 / 2, t_step]]).T
    c = np.matrix([[1, 0, -z_c / 9.81]])

    # Calculate weights
    Ge, Gx, Gd = calculate_weights(A, b, c, N, Qe, R)

    # Output trajectory
    com_traj = np.zeros((traj_length, 3))
    comdot_traj = np.zeros((traj_length, 3))
    comddot_traj = np.zeros((traj_length, 3))

    # State vectors
    x = np.matrix([[0, 0, 0]]).T  # com, \dot com, \ddot com -- Initial values
    y = np.matrix([[0, 0, 0]]).T  # com, \dot com, \ddot com -- Initial values

    # The preview control does not behave well towards the end of the time frame,
    # so we fill our zmp_ref trajectory with padding values at the end. This is under
    # the assumption that the robot does not move in the final state of the planned
    # trajectory.
    padding = np.multiply(np.ones((N, 3)), zmp_ref[-1][None, :])
    padded_zmpref = np.vstack((zmp_ref, padding))

    for t in range(traj_length):
        u = np.zeros(2)
        for i in range(2):
            val = (x, y)[i]
            err = c.dot(val) - zmp_ref[t, i]

            # Given by eq. (21) of Katayama paper
            u[i] = - Ge * err - Gx.dot(val) - Gd[:N].dot(padded_zmpref[t + 1:t + N + 1, i])

        # eq. (5) of Katayama paper
        x = A.dot(x) + b * u[0]
        y = A.dot(y) + b * u[1]

        # Update trajectory
        com_traj[t][0] = x[0, 0]
        com_traj[t][1] = y[0, 0]
        com_traj[t][2] = z_c
        comdot_traj[t][0] = x[1, 0]
        comdot_traj[t][1] = y[1, 0]
        comdot_traj[t][2] = 0
        comddot_traj[t][0] = x[2, 0]
        comddot_traj[t][1] = y[2, 0]
        comddot_traj[t][2] = 0

    return com_traj, comdot_traj, comddot_traj
