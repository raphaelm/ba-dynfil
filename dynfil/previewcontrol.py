import numpy as np
import scipy.linalg


def online_preview_control(zmp_ref, t_step, z_c, traj_length):
    """
    Preview control algorithm, one iteration. Notation from Kajita book (2004), page 143 ff.

    Helpful links:

    * http://www.mwm.im/lqr-controllers-with-python/
    * https://de.mathworks.com/help/control/ref/dlqr.html?requestedDomain=www.mathworks.com
    """

    # Weights taken from Kajita paper (2014), fig. 6
    Q = np.eye(3)
    R = 1e-6 * np.eye(1)

    # Definitions (p. 143-144 from Kajita's book)
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

    x = np.matrix([[0, 0, 0]]).T  # com, \dot com, \ddot com -- Initial values
    y = np.matrix([[0, 0, 0]]).T  # com, \dot com, \ddot com -- Initial values

    # Calculate weights
    f = np.zeros(N)
    for i in range(N):
        # eq (4.75) from p. 144 in Kajita's book
        f[i] = 1 / (R + b.T * P * b) * (b.T * (A - b * K).T ** i * c.T) * Q[0, 0]

    com_traj = np.zeros((traj_length, 3))
    for t in range(traj_length):
        # Limit preview window as we cannot access values after the end of the motion
        preview_size = min(N, traj_length - t - 1)

        u = np.zeros(2)
        for i in range(2):
            # eq (4.74) from p. 144 in Kajita's book
            #u[i] = - K.dot(zmp_diff[t, i])[0,0] + f[:preview_size].dot(zmp_ref[t + 1:t + preview_size + 1, i].T)
            u[i] = - K.dot(x) + f[:preview_size].dot(zmp_ref[t + 1:t + preview_size + 1, i].T)

        x = A.dot(x) + b * u[0]
        y = A.dot(y) + b * u[1]

        com_traj[t][0] = x[0, 0]
        com_traj[t][1] = y[0, 0]
        com_traj[t][2] = z_c

    return com_traj
