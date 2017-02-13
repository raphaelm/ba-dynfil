"""
Implements the analytical inverse kinematics for a biped as outlined in the book from
Kajita on page 53.

Note: This assumes that the *orientations* of pelvis and feet in the input are *constant*,
their derivatives will be ignored.
"""

import numpy as np
import rbdl

from dynfil.utils.angles import rotx, roty, rotx_dot, roty_dot


def fd_helper(dot_ndirs, func, *argpairs):
    """
    Takes a number of dimensions, a function to differentiate and any number of tuples as
    arguments. The first entry of each of these tuples is supposed to be an argument to the
    function, the second entry is supposed to be a vector with regard to which the
    derivation should be computed.
    """
    eps = 1e-8
    f = func(*[a[0] for a in argpairs])
    fdot = np.zeros([dot_ndirs] + list(np.shape(f)))
    for idir in range(dot_ndirs):
        args_h = []
        for a, adot in argpairs:
            args_h.append(
                a + eps * adot.T[idir]
            )
        f_h = func(*args_h)
        fdot[idir] = (f_h - f) / eps
    return fdot


def ik_one_leg(D, A, B, root_r, root_E, foot_r, foot_E, root_dot, foot_dot, debug=False):
    if root_dot is not None and np.any(root_dot):
        dot_ndirs = root_dot.shape[-1]
    else:
        dot_ndirs = None

    # r: vector hip to ankle
    r = foot_E.T.dot(root_r + root_E.dot(D) - foot_r)
    if dot_ndirs:
        r_dot = foot_E.T.dot(root_dot - foot_dot)

        if debug:
            r_func = lambda _root_r, _foot_r: foot_E.T.dot(_root_r + root_E.dot(D) - _foot_r)
            r_dot_fd = fd_helper(dot_ndirs, r_func, (root_r, root_dot), (foot_r, foot_dot)).T
            print "r_dot fd", r_dot_fd
            print "r_dot", r_dot
            np.testing.assert_allclose(r_dot, r_dot_fd, verbose=True, rtol=1e-7)

    # C: norm of r (distance hip to ankle
    # NOTE: C = sqrt(x^T * x)
    # NOTE: C_dot = 1 / (2*sqrt(x^T * x)) * (x_dot^T * x + x^T * x_dot)
    #             = (2 * x^T * x_dot) / (2*sqrt(x^T * x))
    #             = (x^T * x_dot) / sqrt(x^T * x)
    C = np.linalg.norm(r)
    if dot_ndirs:
        C_dot = r.dot(r_dot) / np.sqrt(r.dot(r))
        if debug:
            C_func = lambda _r: np.linalg.norm(_r)
            C_dot_fd = fd_helper(dot_ndirs, C_func, (r, r_dot))
            print "C_dot fd", C_dot_fd
            print "C_dot", C_dot
            np.testing.assert_allclose(C_dot, C_dot_fd, verbose=True, rtol=1e-7)

    # q5: Knee pitch
    # c5: arccos()-Argument for q5 calculation
    # NOTE: A and B are constant, therefore c5_dot is easy to calculate
    c5 = (C ** 2 - A ** 2 - B ** 2) / (2.0 * A * B)
    if dot_ndirs:
        c5_dot = (C * C_dot) / (A * B)
        if debug:
            c5_func = lambda _C: (_C ** 2 - A ** 2 - B ** 2) / (2.0 * A * B)
            c5_dot_fd = fd_helper(dot_ndirs, c5_func, (C, C_dot))
            print "c5_dot fd", c5_dot_fd
            print "c5_dot", c5_dot
            np.testing.assert_allclose(c5_dot, c5_dot_fd, verbose=True, rtol=1e-7)

    if c5 >= 1:
        q5 = 0.0
        if dot_ndirs:
            q5_dot = c5.fill(0.0)
    elif c5 <= -1:
        q5 = np.pi
        if dot_ndirs:
            q5_dot = c5.fill(0.0)
    else:
        if dot_ndirs:
            # from http://www.math.com/tables/derivatives/tableof.htm
            # d/dx arccos(x) = -1 / sqrt(1 - x**2)
            q5_dot = -c5_dot / np.sqrt(1 - c5**2)

        q5 = np.arccos(c5)  # knee pitch

    # q6a: ankle pitch argument
    if dot_ndirs:
        # NOTE A is constant
        # from http://www.math.com/tables/derivatives/tableof.htm
        # d/dx arcsin(x) = 1 / sqrt(1 - x**2)
        q6a_dot = (
            1 / np.sqrt(1 - ((A/C)*np.sin(np.pi - q5))**2) *
            (
                - (A*C_dot)/C**2 * np.sin(np.pi - q5)
                - (A/C) * np.cos(np.pi - q5) * q5_dot
            )
        )

        if debug:
            q6a_func = lambda _C, _q5: np.arcsin((A / _C) * np.sin(np.pi - _q5))
            q6a_dot_fd = fd_helper(dot_ndirs, q6a_func, (C, C_dot), (q5, q5_dot))
            print "q6a_dot fd", q6a_dot_fd
            print "q6a_dot", q6a_dot
            np.testing.assert_allclose(q6a_dot, q6a_dot_fd, verbose=True, rtol=1e-7)

    q6a = np.arcsin((A / C) * np.sin(np.pi - q5))

    # q7: ankle roll -pi/2 < q(6) < pi/2
    if dot_ndirs:
        # from https://en.wikipedia.org/wiki/Atan2#Derivative
        # d/d(x,y) arctan2(x, y) = (-y/(x**2 + y**2), x/(x**2 + y**2))
        q7_dot = (
            r[2]*r_dot[1, :]
            - r[1]*r_dot[2, :]
        )/(r[1]**2 + r[2]**2)

    q7 = np.arctan2(r[1], r[2])
    if q7 > np.pi / 2.:
        q7 = q7 - np.pi
        if dot_ndirs:
            q7_dot = q7_dot.fill(0.0)
    elif q7 < -np.pi / 2.:
        q7 = q7 + np.pi
        if dot_ndirs:
            q7_dot = q7_dot.fill(0.0)

    # q6: ankle pitch
    if dot_ndirs:
        # from https://en.wikipedia.org/wiki/Atan2#Derivative
        # d/d(x,y) arctan2(x, y) = (-y/(x**2 + y**2), x/(x**2 + y**2))
        q6_dot = (
            - (
                r_dot[0, :] * np.sign(r[2]) * np.sqrt(r[1] ** 2 + r[2] ** 2) -
                r[0] * np.sign(r[2]) * (2 * r[1] * r_dot[1, :] + 2 * r[2] * r_dot[2, :])/(2 * np.sqrt(r[1] ** 2 + r[2] ** 2))
            ) / (r[0]**2 + r[1] ** 2 + r[2] ** 2)
            - q6a_dot
        )

        if debug:
            q6_func = lambda _r, _q6a: -np.arctan2(_r[0], np.sign(_r[2]) * np.sqrt(_r[1] ** 2 + _r[2] ** 2)) - _q6a
            q6_dot_fd = fd_helper(dot_ndirs, q6_func, (r, r_dot), (q6a, q6a_dot))
            print "q6_dot fd", q6_dot_fd
            print "q6_dot", q6_dot
            np.testing.assert_allclose(q6_dot, q6_dot_fd, verbose=True, rtol=1e-7)

    q6 = -np.arctan2(r[0], np.sign(r[2]) * np.sqrt(r[1] ** 2 + r[2] ** 2)) - q6a

    # R = Body.R' * Foot.R * Rroll(-q7) * Rpitch(-q6-q5) # hipZ*hipX*hipY
    if dot_ndirs:
        R_dot = np.zeros((dot_ndirs, 3, 3))
        r_x_q7_dot = rotx_dot(-q7, -q7_dot)
        r_y_q6_q5_dot = roty_dot(-q6 - q5, - q6_dot - q5_dot)

        for i in range(dot_ndirs):
            R_dot[i, :, :] = root_E.T.dot(foot_E).dot(
                rotx(-q7).dot(r_y_q6_q5_dot[:, :, i])
                + r_x_q7_dot[:, :, i].dot(roty(-q6 - q5))
            )

        if debug:
            R_func = lambda _q7, _q6, _q5: root_E.T.dot(foot_E).dot(rotx(-_q7)).dot(roty(-_q6 - _q5))
            R_dot_fd = fd_helper(dot_ndirs, R_func, (q7, q7_dot), (q6, q6_dot), (q5, q5_dot))
            print "R_dot fd", R_dot_fd
            print "R_dot", R_dot
            np.testing.assert_allclose(R_dot, R_dot_fd, verbose=True, rtol=1e-5, atol=1e-5)

    R = root_E.T.dot(foot_E).dot(rotx(-q7)).dot(roty(-q6 - q5))

    # q2: hip yaw
    if dot_ndirs:
        q2_dot = (
            + R[1, 1]/(R[0, 1]**2 + R[1, 1]**2)*R_dot[:, 0, 1]
            - R[0, 1]/(R[0, 1]**2 + R[1, 1]**2)*R_dot[:, 1, 1]
        )

    q2 = np.arctan2(-R[0, 1], R[1, 1])

    # q3: hip roll
    cz = np.cos(q2)
    sz = np.sin(q2)
    if dot_ndirs:
        cz_dot = -np.sin(q2)*q2_dot
        sz_dot = np.cos(q2)*q2_dot
        q3_dot = (
            + R_dot[:, 2, 1] * (-R[0, 1] * sz + R[1, 1] * cz)
            - R[2, 1] * (-R_dot[:, 0, 1] * sz - R[0, 1] * sz_dot + R_dot[:, 1, 1] * cz + R[1, 1] * cz_dot)
        )/(R[2, 1]**2 + (-R[0, 1]*sz + R[1, 1]*cz)**2)

    q3 = np.arctan2(R[2, 1], -R[0, 1] * sz + R[1, 1] * cz)

    # q4: hip pitch
    if dot_ndirs:
        # from https://en.wikipedia.org/wiki/Atan2#Derivative
        # d/d(x,y) arctan2(x, y) = (-y/(x**2 + y**2), x/(x**2 + y**2))
        q4_dot = (
            - R[2, 2] * R_dot[:, 2, 0]
            + R[2, 0] * R_dot[:, 2, 2]
        )/(R[2, 0]**2 + R[2, 2]**2)

        if debug:
            q4_func = lambda _R: np.arctan2(-_R[2, 0], _R[2, 2])
            q4_dot_fd = fd_helper(dot_ndirs, q4_func, (R, R_dot.transpose([2, 1, 0])))
            print "q4_dot fd", q4_dot_fd
            print "q4_dot", q4_dot
            np.testing.assert_allclose(q4_dot, q4_dot_fd, verbose=True, rtol=1e-5, atol=1e-5)

    q4 = np.arctan2(-R[2, 0], R[2, 2])

    q = np.array([q2, q3, q4, q5, q6, q7])
    if dot_ndirs:
        qdot = np.array([q2_dot, q3_dot, q4_dot, q5_dot, q6_dot, q7_dot])
    else:
        qdot = None
    qddot = np.array([[0], [0], [0], [0], [0], [0]])
    return q, qdot, qddot


def ik_constants(model, q_ini):
    # TODO: Center of mass corrections!
    rbdl.UpdateKinematics(model, q_ini, np.zeros(model.qdot_size), np.zeros(model.qdot_size))

    root_x = model.X_base[model.GetBodyId("pelvis")]
    hipr_x = model.X_base[model.GetBodyId("hip_right")]
    kneer_x = model.X_base[model.GetBodyId("knee_right")]
    footr_x = model.X_base[model.GetBodyId("ankle_right")]
    D = hipr_x.r - root_x.r
    A = np.linalg.norm(hipr_x.r - kneer_x.r)
    B = np.linalg.norm(kneer_x.r - footr_x.r)
    return D, A, B


def ik_trajectory(model, q_ini, chest, lsole, rsole,
                  chest_dot=None, lsole_dot=None, rsole_dot=None):
    chest_dot = chest_dot or chest.traj_pos_dot
    lsole_dot = lsole_dot or lsole.traj_pos_dot
    rsole_dot = rsole_dot or rsole.traj_pos_dot

    # Check dimensions
    dot_ndirs = None
    if chest_dot is not None and np.any(chest_dot):
        dot_ndirs = rsole_dot.shape[-1]
        assert chest_dot.r.shape == (3, dot_ndirs)
        assert rsole_dot.r.shape == (3, dot_ndirs)
        assert lsole_dot.r.shape == (3, dot_ndirs)

    q = np.zeros((len(chest), model.q_size))
    if dot_ndirs:
        qdot = np.zeros((len(chest), model.dof_count, dot_ndirs or 1))
    else:
        qdot = None

    D, A, B = ik_constants(model, q_ini)

    for t in range(len(q)):
        lq, lqdot, lqddot = ik_one_leg(
            -D, A, B,
            chest.traj_pos[t], chest.traj_ort[t],
            lsole.traj_pos[t] + lsole.body_point, lsole.traj_ort[t],
            chest_dot[t], lsole_dot[t]
        )
        rq, rqdot, rqddot = ik_one_leg(
            D, A, B,
            chest.traj_pos[t], chest.traj_ort[t],
            rsole.traj_pos[t] + rsole.body_point, rsole.traj_ort[t],
            chest_dot[t], rsole_dot[t]
        )

        q[t, 0:3] = chest.traj_pos[t]
        q[t, 6:12] = rq
        q[t, 12:18] = lq

        if dot_ndirs:
            qdot[t, 0:3] = chest_dot[t, 1]  # TODO: Really?
            # TODO: qddot[t, 0:3] = chest.traj_pos_ddot[t]

            qdot[t, 6:12, :] = rqdot
            #qddot[t, 6:12] = rqddot

            qdot[t, 12:18, :] = lqdot
            #qddot[t, 12:18] = lqddot

    return q, qdot
