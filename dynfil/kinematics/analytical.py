"""
Implements the analytical inverse kinematics for a biped as outlined in the book from
Kajita on page 53.

Note: This assumes that the *orientations* of pelvis and feet in the input are *constant*,
their derivatives will be ignored.
"""

import numpy as np
import rbdl

from dynfil.utils.angles import rotx, roty


def ik_one_leg(D, A, B, root, foot, t):
    root_r = root.traj_pos[t]
    foot_r = foot.traj_pos[t]
    root_E = root.traj_ort[t]
    foot_E = foot.traj_ort[t]
    # Kajitas book, p 53
    r = foot_E.T.dot(root_r + root_E.dot(D) - foot_r)
    C = np.linalg.norm(r)
    c5 = (C ** 2 - A ** 2 - B ** 2) / (2.0 * A * B)
    if c5 >= 1:
        q5 = 0.0
    elif c5 <= -1:
        q5 = np.pi
    else:
        q5 = np.arccos(c5)  # knee pitch

    q6a = np.arcsin((A / C) * np.sin(np.pi - q5))  # ankle pitch sub

    q7 = np.arctan2(r[1], r[2])  # ankle roll -pi/2 < q(6) < pi/2
    if q7 > np.pi / 2.:
        q7 = q7 - np.pi
    elif q7 < -np.pi / 2.:
        q7 = q7 + np.pi

    # ankle pitch
    q6 = -np.arctan2(r[0], np.sign(r[2]) * np.sqrt(r[1] ** 2 + r[2] ** 2)) - q6a

    R = root_E.T.dot(foot_E).dot(rotx(-q7)).dot(roty(-q6 - q5))

    # hip yaw
    q2 = np.arctan2(-R[0, 1], R[1, 1])

    # hip roll
    cz = np.cos(q2)
    sz = np.sin(q2)
    q3 = np.arctan2(R[2, 1], -R[0, 1] * sz + R[1, 1] * cz)

    # hip pitch
    q4 = np.arctan2(-R[2, 0], R[2, 2])

    q = np.array([q2, q3, q4, q5, q6, q7])
    qdot = np.array([0, 0, 0, 0, 0, 0])
    qddot = np.array([0, 0, 0, 0, 0, 0])
    return q, qdot, qddot


def ik_trajectory(model, q_ini, chest, lsole, rsole):
    q = np.zeros((len(chest), model.q_size))
    qdot = np.zeros((len(chest), model.qdot_size))
    qddot = np.zeros((len(chest), model.qdot_size))

    # TODO: Center of mass corrections!
    rbdl.UpdateKinematics(model, q_ini, np.zeros(model.qdot_size), np.zeros(model.qdot_size))

    root_x = model.X_base[model.GetBodyId("pelvis")]
    hipr_x = model.X_base[model.GetBodyId("hip_right")]
    kneer_x = model.X_base[model.GetBodyId("knee_right")]
    footr_x = model.X_base[model.GetBodyId("ankle_right")]
    D = hipr_x.r - root_x.r
    A = np.linalg.norm(hipr_x.r - kneer_x.r)
    B = np.linalg.norm(kneer_x.r - footr_x.r)

    for t in range(len(q)):
        lq, lqdot, lqddot = ik_one_leg(
            -D, A, B, chest, lsole, t
        )
        rq, rqdot, rqddot = ik_one_leg(
            D, A, B, chest, rsole, t
        )

        q[t, 0:3] = chest.traj_pos[t]
        qdot[t, 0:3] = chest.traj_pos_dot[t]
        qddot[t, 0:3] = chest.traj_pos_ddot[t]
        # TDOO: qdot[0:3]
        # TODO: qddot[0:3]
        # TODO get Euler angles from matrix => q[3:6]

        q[t, 6:12] = rq
        qdot[t, 6:12] = rqdot
        qddot[t, 6:12] = rqddot

        q[t, 12:18] = lq
        qdot[t, 12:18] = lqdot
        qddot[t, 12:18] = lqddot

    return q
