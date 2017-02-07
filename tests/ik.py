#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Implementation of analytic inverse kinematics based on
'Introduction to Humanoid Robotics', Kajita, 2013
"""
import os
import sys
import copy
import pprint
import numpy as np

import rbdl

import dynfil.utils as utils
from dynfil.utils import (rotx, roty, rotz)
from dynfil.utils import (rotx_dot, roty_dot, rotz_dot)


# -----------------------------------------------------------------------------
HEADER = """
COLUMNS:
time,
pelvis:T:X,
pelvis:T:Y,
pelvis:T:Z,
pelvis:R:X:rad,
pelvis:R:Y:rad,
pelvis:R:Z:rad,
hip_right:R:-Z:rad,
hip_right:R:-X:rad,
hip_right:R:-Y:rad,
knee_right:R:Y:rad,
ankle_right:R:Y:rad,
ankle_right:R:X:rad,
hip_left:R:-Z:rad,
hip_left:R:-X:rad,
hip_left:R:-Y:rad,
knee_left:R:Y:rad,
ankle_left:R:Y:rad,
ankle_left:R:X:rad

DATA:
"""

# -----------------------------------------------------------------------------
def ik_one_leg_fd(
    model, side,
    root, foot,
    root_dot=None, foot_dot=None,
    root_ddot=None, foot_ddot=None,
    EPS=1e-8
):
    """Compute derivatives using finite differences."""
    # check dimensions
    dot_ndirs = None
    if root_dot and foot_dot:
        dot_ndirs = root_dot.r.shape[-1]
        assert root_dot.r.shape == (3, dot_ndirs)
        assert root_dot.E.shape == (3, 3, dot_ndirs)

        assert foot_dot.r.shape == (3, dot_ndirs)
        assert foot_dot.E.shape == (3, 3, dot_ndirs)

    ddot_ndirs = None
    if root_ddot and foot_ddot:
        ddot_ndirs = root_ddot.r.shape[-1]
        assert root_ddot.r.shape == (3, ddot_ndirs)
        assert root_ddot.E.shape == (3, 3, ddot_ndirs)

        assert foot_ddot.r.shape == (3, ddot_ndirs)
        assert foot_ddot.E.shape == (3, 3, ddot_ndirs)

    # nominal evaluation
    q, __, __ = ik_one_leg(
        model, side,
        root=root, foot=foot,
        root_dot=None, foot_dot=None,
        root_ddot=None, foot_ddot=None
    )

    # assign joint velocities
    qdot = None
    if dot_ndirs:
        qdot = np.zeros([6, dot_ndirs])
        root_h = copy.deepcopy(root)
        foot_h = copy.deepcopy(foot)
        for idir in range(dot_ndirs):
            root_h.r = root.r + EPS * root_dot.r[:, idir]
            root_h.E = root.E + EPS * root_dot.E[:, :, idir]

            foot_h.r = foot.r + EPS * foot_dot.r[:, idir]
            foot_h.E = foot.E + EPS * foot_dot.E[:, :, idir]

            q_h, __, __ = ik_one_leg(
                model, side,
                root=root_h, foot=foot_h,
                root_dot=None, foot_dot=None,
                root_ddot=None, foot_ddot=None
            )
            qdot[:, idir] = (q_h - q) / EPS


    qddot = None
    if ddot_ndirs:
        qddot = np.zeros([6, dot_ndirs])

    return q, qdot, qddot


# -----------------------------------------------------------------------------
def ik_one_leg(
    model, side,
    root, foot,
    root_dot=None, foot_dot=None,
    root_ddot=None, foot_ddot=None
):
    """
    Single leg inverse kinematics providing positions, velocities and
    accelerations.

    NOTE::
        Requires each leg to have 6 degrees of freedom with a particular
        configuration.

    Reference: 'Introduction to Humanoid Robotics', Kajita 2013
    """
    # check dimensions
    dot_ndirs = None
    if root_dot and foot_dot:
        dot_ndirs = root_dot.r.shape[-1]
        assert root_dot.r.shape == (3, dot_ndirs)
        assert root_dot.E.shape == (3, 3, dot_ndirs)

        assert foot_dot.r.shape == (3, dot_ndirs)
        assert foot_dot.E.shape == (3, 3, dot_ndirs)

    ddot_ndirs = None
    if root_ddot and foot_ddot:
        ddot_ndirs = root_ddot.r.shape[-1]
        assert root_ddot.r.shape == (3, ddot_ndirs)
        assert root_ddot.E.shape == (3, 3, ddot_ndirs)

        assert foot_ddot.r.shape == (3, ddot_ndirs)
        assert foot_ddot.E.shape == (3, 3, ddot_ndirs)

    # compute IK for each leg
    root_id = model.GetBodyId("pelvis")
    root_X = model.X_base[root_id]
    # print "root ID: ", pprint.pprint(root_id)
    # print "  X:\n", root_X

    hipr_id = model.GetBodyId("hip_" + side)
    hipr_X = model.X_base[hipr_id]
    # print "hip right ID: ", pprint.pprint(hipr_id)
    # print "  X:\n", hipr_X

    kneer_id = model.GetBodyId("knee_" + side)
    kneer_X = model.X_base[kneer_id]
    # print "knee right ID: ", pprint.pprint(kneer_id)
    # print "  X:\n", kneer_X

    footr_id = model.GetBodyId("ankle_" + side)
    footr_X = model.X_base[footr_id]
    # print "foot right ID: ", pprint.pprint(footr_id)
    # print "  X:\n", footr_X

    # NOTE D, A, B are constants
    D = hipr_X.r - root_X.r
    A = np.linalg.norm(hipr_X.r - kneer_X.r)
    B = np.linalg.norm(kneer_X.r - footr_X.r)

    # crotch from ankle
    # second order derivative evaluation
    if ddot_ndirs:
        pass

    # first order derivative evaluation
    if dot_ndirs:
        # NOTE D is constant
        r_dot = foot_dot.E.T.dot(root.r + root.E.dot(D) - foot.r).T \
            + foot.E.T.dot(root_dot.r + D.dot(root_dot.E.T).T - foot_dot.r)

        # print "r_dot: ", r_dot.shape
        # print r_dot

        # # compute FD
        EPS = 1e-8
        r_fd = np.zeros([3, dot_ndirs])
        r_h0 = foot.E.T.dot(root.r + root.E.dot(D) - foot.r)
        for i in range(dot_ndirs):
            root_h = copy.deepcopy(root)
            foot_h = copy.deepcopy(foot)

            root_h.r[:] = root.r[:] + EPS * root_dot.r[:, i]
            root_h.E[:, :] = root.E[:, :] + EPS * root_dot.E[:, :, i]

            foot_h.r[:] = foot.r[:] + EPS * foot_dot.r[:, i]
            foot_h.E[:, :] = foot.E[:, :] + EPS * foot_dot.E[:, :, i]

            r_h = foot_h.E.T.dot(root_h.r + root_h.E.dot(D) - foot_h.r)
            r_fd[:, i] = (r_h - r_h0) / EPS

        # print "r_fd: ", r_fd.shape
        # print r_fd
        # print "error: "
        # print r_dot - r_fd

    # nominal evaluation
    # r = Foot.R’ * (Body.p + Body.R * [0 D 0]’- Foot.p)
    r = foot.E.T.dot(root.r + root.E.dot(D) - foot.r)
    # print "r: ", r


    # first order derivative evaluation
    if dot_ndirs:
        # NOTE: C = sqrt(x^T * x)
        # NOTE: C_dot = 1 / (2*sqrt(x^T * x)) * (x_dot^T * x + x^T * x_dot)
        #             = (2 * x^T * x_dot) / (2*sqrt(x^T * x))
        #             = (x^T * x_dot) / sqrt(x^T * x)
        C_dot = r.dot(r_dot) / np.sqrt(r.dot(r))

        # print "C_dot: ", C_dot.shape
        # print C_dot

        # # compute FD
        # EPS = 1e-8
        # C_fd = np.zeros([1, dot_ndirs])
        # C_h0 = np.linalg.norm(r)
        # for i in range(dot_ndirs):
        #     r_h = r + EPS * r_dot[:, i]
        #     C_h = np.linalg.norm(r_h)
        #     C_fd[:, i] = (C_h - C_h0) / EPS
        # print "C_fd: ", C_fd.shape
        # print C_fd
        # print "error: "
        # print C_dot - C_fd

    # nominal evaluation
    # C = norm(r)
    C = np.linalg.norm(r)
    # print "C: ", C

    # compute knee angle q5
    # first order derivative evaluation
    if dot_ndirs:
        # NOTE only C is non-constant
        c5_dot = (C * C_dot) / (A*B)

        # print "c5_dot: ", c5_dot.shape
        # print c5_dot

        # # compute FD
        # EPS = 1e-8
        # c5_fd = np.zeros([dot_ndirs])
        # c5_h0 = (C**2 - A**2 - B**2) / (2.0*A*B)
        # for i in range(dot_ndirs):
        #     C_h = C + EPS * C_dot[i]
        #     c5_h = (C_h**2 - A**2 - B**2) / (2.0*A*B)
        #     c5_fd[i] = (c5_h - c5_h0) / EPS
        # print "c5_fd: ", c5_fd.shape
        # print c5_fd
        # print "error: "
        # print c5_dot - c5_fd

    # nominal evaluation
    c5 = (C**2 - A**2 - B**2) / (2.0*A*B)
    # print "c5: ", c5

    if c5 >= 1:
        # first order derivative evaluation
        if dot_ndirs:
            q5_dot = c5.fill(0.0)

        # nominal evaluation
        q5 = 0.0
    elif c5 <= -1:
        # first order derivative evaluation
        if dot_ndirs:
            q5_dot = c5.fill(0.0)

        # nominal evaluation
        q5 = np.pi
    else:
        # first order derivative evaluation
        if dot_ndirs:
            # from http://www.math.com/tables/derivatives/tableof.htm
            # d/dx arccos(x) = -1 / sqrt(1 - x**2)
            q5_dot = -c5_dot / np.sqrt(1 - c5**2)
            print "q5_dot: ", q5_dot.shape
            print q5_dot

            # compute FD
            # EPS = 1e-8
            # q5_fd = np.zeros([dot_ndirs])
            # q5_h0 = np.arccos(c5)  # knee pitch
            # for i in range(dot_ndirs):
            #     c5_h = c5 + EPS * c5_dot[i]
            #     q5_h = np.arccos(c5_h)  # knee pitch
            #     q5_fd[i] = (q5_h - q5_h0) / EPS
            # print "q5_fd: ", q5_fd.shape
            # print q5_fd
            # print "error: "
            # print q5_dot - q5_fd

        # nominal evaluation
        q5 = np.arccos(c5)  # knee pitch

    print "q5: ", q5

    # compute ankle pitch
    # first order derivative evaluation
    if dot_ndirs:
        # NOTE A is constant
        # from http://www.math.com/tables/derivatives/tableof.htm
        # d/dx arcsin(x) = 1 / sqrt(1 - x**2)
        q6a_dot = 1 / np.sqrt(1 - (A/C)*np.sin(np.pi - q5)) \
            * (
                (-A*C_dot)/C**2 * np.sin(np.pi - q5)
                - (A/C) * np.cos(np.pi - q5) * q5_dot
            )

    # nominal evaluation
    q6a = np.arcsin((A/C)*np.sin(np.pi - q5))  # ankle pitch sub

    sys.exit()
    # compute ankle roll
    # first order derivative evaluation
    if dot_ndirs:
        # from https://en.wikipedia.org/wiki/Atan2#Derivative
        # d/d(x,y) arctan2(x, y) = (-y/(x**2 + y**2), x/(x**2 + y**2))
        q7_dot = (
            -r[2]/(r[1]**2 + r[2]**2)*r_dot[1, :]
            + r[1]/(r[1]**2 + r[2]**2)*r_dot[2, :]
        )

    # nominal evaluation
    q7 = np.arctan2(r[1], r[2])  # ankle roll -pi/2 < q(6) < pi/2

    if q7 > np.pi/2.:
        # first order derivative evaluation
        if dot_ndirs:
            q7_dot = q7_dot.fill(0.0)
        # nominal evaluation
        q7 = q7 - np.pi
    elif q7 < -np.pi/2.:
        # first order derivative evaluation
        if dot_ndirs:
            q7_dot = q7_dot.fill(0.0)
        # nominal evaluation
        q7 = q7 + np.pi

    # compute ankle pitch
    # first order derivative evaluation
    if dot_ndirs:
        q6_dot = -np.arctan2(r[0], np.sign(r[2])*np.sqrt(r[1]**2 + r[2]**2)) - q6a
    # nominal evaluation
    q6 = -np.arctan2(r[0], np.sign(r[2])*np.sqrt(r[1]**2 + r[2]**2)) - q6a

    # R = Body.R’ * Foot.R * Rroll(-q7) * Rpitch(-q6-q5) # hipZ*hipX*hipY
    # TODO check roll, yaw, pitch
    # first order derivative evaluation
    if dot_ndirs:
        R_dot = root.E.T.dot(foot.E).dot(rotx(-q7)).dot(roty(-q6 - q5))
        R_dot = (
            root_dot.E.T.dot(foot.E).dot(rotx(-q7)).dot(roty(-q6 - q5)).T
            + root.E.T.dot(
                foot_dot.E.T.dot( rotx(-q7) ).dot( roty(-q6 - q5) ).T
                + foot.E.dot(
                    rotx_dot(-q7, -q7_dot).T.dot( roty(-q6 - q5) ).T
                    + rotx(-q7).dot( roty_dot(-q6 - q5, -q6_dot - q5_dot) )
                )
            )
        )

    # nominal evaluation
    R = root.E.T.dot(foot.E).dot(rotx(-q7)).dot(roty(-q6 - q5))

    # hip yaw
    # first order derivative evaluation
    if dot_ndirs:
        # from https://en.wikipedia.org/wiki/Atan2#Derivative
        # d/d(x,y) arctan2(x, y) = (-y/(x**2 + y**2), x/(x**2 + y**2))
        q2_dot = (
            R[1, 1]/(R[0, 1]**2 + R[1, 1]**2)*R_dot[0, 1, :]
            - R[0, 1]/(R[0, 1]**2 + R[1, 1]**2)*R_dot[1, 1, :]
        )

    # nominal evaluation
    q2 = np.arctan2(-R[0, 1], R[1,1])

    # hip roll
    # first order derivative evaluation
    if dot_ndirs:
        cz_dot = -np.sin(q2)*q2_dot
        sz_dot = np.cos(q2)*q2_dot

    # nominal evaluation
    cz = np.cos(q2)
    sz = np.sin(q2)

    # first order derivative evaluation
    if dot_ndirs:
        cz_dot = -np.sin(q2)*q2_dot
        sz_dot = np.cos(q2)*q2_dot
        # from https://en.wikipedia.org/wiki/Atan2#Derivative
        # d/d(x,y) arctan2(x, y) = (-y/(x**2 + y**2), x/(x**2 + y**2))
        q3_dot = (
            -(-R[0, 1]*sz + R[1, 1]*cz)/(R[2, 1]**2 + (-R[0, 1]*sz + R[1, 1]*cz)**2)
            + R[2, 1]/(R[2, 1]**2 + (-R[0, 1]*sz + R[1, 1]*cz)**2)
        )

    # nominal evaluation
    q3 = np.arctan2(R[2, 1], -R[0, 1]*sz + R[1, 1]*cz)

    # hip pitch
    # first order derivative evaluation
    if dot_ndirs:
        # from https://en.wikipedia.org/wiki/Atan2#Derivative
        # d/d(x,y) arctan2(x, y) = (-y/(x**2 + y**2), x/(x**2 + y**2))
        q4_dot = (
            R[2, 2]/(R[2, 0]**2 + R[2, 2]**2)*R_dot[2, 0, :]
            - R[2, 0]/(R[2, 0]**2 + R[2, 2]**2)*R_dot[2, 2, :]
        )
    # nominal evaluation
    q4 = np.arctan2(-R[2, 0], R[2, 2])

    # sort in joint angles
    q = np.array([q2, q3, q4, q5, q6, q7])

    qdot = None
    if dot_ndirs:
        qdot = np.zeros([6, dot_ndirs])
        qdot[0, :] = q2_dot
        qdot[1, :] = q3_dot
        qdot[2, :] = q4_dot
        qdot[3, :] = q5_dot
        qdot[4, :] = q6_dot
        qdot[5, :] = q7_dot

    qddot = None
    if ddot_ndirs:
        qddot = np.zeros([6, ddot_ndirs])
        # qddot[0, :] = q2_ddot
        # qddot[1, :] = q3_ddot
        # qddot[2, :] = q4_ddot
        # qddot[3, :] = q5_ddot
        # qddot[4, :] = q6_ddot
        # qddot[5, :] = q7_ddot

    return q, qdot, qddot


def ik_full(
        model,
        root, foot_right, foot_left,
        root_dot=None, foot_right_dot=None, foot_left_dot=None,
        root_ddot=None, foot_right_ddot=None, foot_left_ddot=None
):
    """
    Compute inverse kinematics for pelvis and both legs.
    """
    # check dimensions
    dot_ndirs = None
    if root_dot and foot_right_dot and foot_left_dot:
        dot_ndirs = root_dot.r.shape[-1]
        assert root_dot.r.shape == (3, dot_ndirs)
        assert root_dot.E.shape == (3, 3, dot_ndirs)

        assert foot_right_dot.r.shape == (3, dot_ndirs)
        assert foot_right_dot.E.shape == (3, 3, dot_ndirs)

        assert foot_left_dot.r.shape == (3, dot_ndirs)
        assert foot_left_dot.E.shape == (3, 3, dot_ndirs)


    ddot_ndirs = None
    if root_ddot and foot_right_ddot and foot_left_ddot:
        ddot_ndirs = root_ddot.r.shape[-1]
        assert root_ddot.r.shape == (3, ddot_ndirs)
        assert root_ddot.E.shape == (3, 3, ddot_ndirs)

        assert foot_right_ddot.r.shape == (3, ddot_ndirs)
        assert foot_right_ddot.E.shape == (3, 3, ddot_ndirs)

        assert foot_left_ddot.r.shape == (3, ddot_ndirs)
        assert foot_left_ddot.E.shape == (3, 3, ddot_ndirs)

    # compute IK for each leg
    lq, lqdot, lqddot = ik_one_leg(
        model, "left",
        root, foot_left,
        root_dot, foot_left_dot,
        root_ddot, foot_left_ddot
    )
    rq, rqdot, rqddot = ik_one_leg(
        model, "right",
        root, foot_right,
        root_dot, foot_right_dot,
        root_ddot, foot_right_ddot
    )

    # assign joint positions
    q = np.zeros([model.dof_count])
    q[0:3] = root.r
    q[3:6] = utils.euler_from_matrix(root.E, order="123")

    q[6:12] = rq
    q[12:18] = lq

    # assign joint velocities
    qdot = None
    if dot_ndirs:
        qdot = np.zeros([model.dof_count, dot_ndirs])
        qdot[0:3, :] = root_dot.r
        # TODO is this correct?
        qdot[3:6, :] = utils.euler_from_matrix(root_dot.E, order="123")
        qdot[6:12, :] = rqdot
        qdot[12:18, :] = lqdot

    # assign joint accelerations
    qddot = None
    if ddot_ndirs:
        qddot = np.zeros([model.dof_count, ddot_ndirs])
        qddot[0:3, :] = root_ddot.r
        # TODO is this correct?
        qddot[3:6, :] = utils.euler_from_matrix(root_ddot.E, order="123")
        qddot[6:12, :] = rqddot
        qddot[12:18, :] = lqddot

    # always return all 3 quantities
    # TODO better ideas?
    return q, qdot, qddot

# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # load model
    model_path = os.path.join("data", "models", "ik_test.lua")
    model = rbdl.loadModel(model_path)

    # prepare in and out states
    q_ini = np.zeros(model.dof_count)
    qdot_ini = np.zeros(model.dof_count)
    qddot_ini = np.zeros(model.dof_count)

    q_out = np.zeros(model.dof_count)
    qdot_out = np.zeros(model.dof_count)
    qddot_out = np.zeros(model.dof_count)

    # update kinematics
    rbdl.UpdateKinematics(model, q_ini, qdot_ini, qddot_ini)

    # prepare desired poses for CoM and feet
    class Pose(object):
        pass

    root_desired = Pose()
    root_desired.r = np.array([0.0, 0.0, 0.60])
    root_desired.E = utils.rotz(np.pi/8.)
    root_desired.r_dot = np.zeros([3, 4])

    root_dot_desired = Pose()
    root_dot_desired.r = np.zeros([3, 4])
    root_dot_desired.r[:, 0] = np.array([0.2, 0.0, -0.1])
    root_dot_desired.r[:, 1:] = np.eye(3)
    root_dot_desired.E = np.zeros([3, 3, 4])
    # root_dot_desired.E[:, :, 0] = utils.rotz_dot(np.pi/8., np.pi/16.)
    # root_dot_desired.E[:, :, 1:] = utils.rotz_dot(np.pi/8., np.pi/16.)

    footr_desired = Pose()
    footr_desired.r = np.array([-0.2, -0.25, 0.0])
    footr_desired.E = np.eye(3)

    footr_dot_desired = Pose()
    footr_dot_desired.r = np.zeros([3, 4])
    footr_dot_desired.r[:, 0] = np.array([0.2, 0.0, -0.1])
    footr_dot_desired.r[:, 1:] = np.eye(3)
    footr_dot_desired.E = np.zeros([3, 3, 4])

    footl_desired = Pose()
    footl_desired.r = np.array([0.3, 0.25, 0.0])
    footl_desired.E = utils.roty(np.pi/8.).dot(utils.rotx(np.pi/16.))

    footl_dot_desired = Pose()
    footl_dot_desired.r = np.zeros([3, 4])
    footl_dot_desired.r[:, 0] = np.array([0.2, 0.0, -0.1])
    footl_dot_desired.r[:, 1:] = np.eye(3)
    footl_dot_desired.E = np.zeros([3, 3, 4])

    # call to inverse kinematics
    q_out, qdot_out, qddot_out = ik_full(
        model,
        root_desired, footr_desired, footl_desired,
        root_dot_desired, footr_dot_desired, footl_dot_desired
    )

    print "qdot: \n", qdot_out
    print "qddot: \n", qddot_out

    # create MeshUp animation file
    meshup_data = np.zeros([2, 1 + model.dof_count])
    meshup_data[1, 0] = 0.0  # time
    meshup_data[0, 1:] = q_ini  # q

    meshup_data[1, 0] = 1.0  # time
    meshup_data[1, 1:] = q_out  # q

    meshup_path = os.path.join("out", "ik_test.csv")
    np.savetxt(meshup_path, meshup_data, fmt="%.18f", delimiter=", ", newline="\n", comments="", header=HEADER)
