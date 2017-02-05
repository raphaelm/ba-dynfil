#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Implementation of analytic inverse kinematics based on
'Introduction to Humanoid Robotics', Kajita, 2013
"""
import os
import sys
import pprint
import numpy as np

import rbdl

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
hip_right:R:Z:rad,
hip_right:R:Y:rad,
hip_right:R:X:rad,
knee_right:R:Y:rad,
ankle_right:R:Y:rad,
ankle_right:R:X:rad,
hip_left:R:Z:rad,
hip_left:R:Y:rad,
hip_left:R:X:rad,
knee_left:R:Y:rad,
ankle_left:R:Y:rad,
ankle_left:R:X:rad

DATA:
"""


# -----------------------------------------------------------------------------
def rotx (x):
    s = np.sin(x)
    c = np.cos(x)
    return np.array([
        [1., 0., 0.],
        [0., c, s],
        [0., -s, c]
    ])


def roty (y):
    s = np.sin(y)
    c = np.cos(y)
    return np.array([
        [c, 0., -s],
        [0., 1., 0.],
        [s, 0., c]
    ])


def rotz (z):
    s = np.sin(z)
    c = np.cos(z)
    return np.array([
        [c, s, 0.],
        [-s, c, 0.],
        [0., 0., 1.]
    ])


def rotxdot (x, xdot):
    s = np.sin(x)
    c = np.cos(x)
    return np.array([
        [0., 0., 0.],
        [0., -s * xdot, c * xdot],
        [0., -c * xdot,-s * xdot]
    ])


def rotydot (y, ydot):
    s = np.sin(y)
    c = np.cos(y)
    return np.array([
        [-s * ydot, 0., - c * ydot],
        [0., 0., 0.],
        [c * ydot, 0., - s * ydot]
    ])


def rotzdot (z, zdot):
    s = np.sin(z)
    c = np.cos(z)
    return np.array([
        [-s * zdot, c * zdot, 0.],
        [-c * zdot, -s * zdot, 0.],
        [0., 0., 0.]
    ])


# -----------------------------------------------------------------------------
def ik_leg(
    model,
    root, foot_right, foot_left
    # root_dot=None, foot_dot=None,
    # root_ddot=None, foot_ddot=None
):
    """
    Single leg inverse kinematics providing positions, velocities and
    accelerations.

    NOTE::
        Requires each leg to have 6 degrees of freedom with a particular
        configuration.

    Reference: 'Introduction to Humanoid Robotics', Kajita 2013

    Parameters
    ----------
    model : rbdl.Model()
        RBDL model of humanoid
    root : array-like (3,)
        desired root position
    root_dot : array-like (3,)
        desired root velocity
    root_ddot : array-like (3,)
        desired root acceleration
    foot : array-like (3,)
        desired foot position
    foot_dot : array-like (3,)
        desired foot velocity
    foot_ddot : array-like (3,)
        desired foot acceleration

    Returns
    -------
    q : array_like (model.dof_count)
        joint positions required for reaching desired position of root and foot
    q_dot : array_like (model.dof_count)
        joint velocities required for reaching desired velocities of root and foot
    q_ddot : array_like (model.dof_count)
        joint accelerations required for reaching desired accelerations of root and foot
    """
    print "root dsrd: ", root.r, "\n", root.E
    print ""

    print "foot right dsrd: ", foot_right.r, "\n", foot_right.E
    print ""

    print "foot left dsrd: ", foot_left.r, "\n", foot_left.E
    print ""

    q = np.zeros(model.dof_count)
    qdot = np.zeros(model.dof_count)
    qddot = np.zeros(model.dof_count)

    root_id = model.GetBodyId("pelvis")
    root_X = model.X_base[root_id]
    # print "root ID: ", pprint.pprint(root_id)
    # print "  X:\n", root_X

    hipr_id = model.GetBodyId("hip_right")
    hipr_X = model.X_base[hipr_id]
    # print "hip right ID: ", pprint.pprint(hipr_id)
    # print "  X:\n", hipr_X

    kneer_id = model.GetBodyId("knee_right")
    kneer_X = model.X_base[kneer_id]
    # print "knee right ID: ", pprint.pprint(kneer_id)
    # print "  X:\n", kneer_X

    footr_id = model.GetBodyId("ankle_right")
    footr_X = model.X_base[footr_id]
    # print "foot right ID: ", pprint.pprint(footr_id)
    # print "  X:\n", footr_X

    # assign obvious choices for root frame
    q[0:3] = root.r
    # TODO get Euler angles from matrix
    # q[3:6]

    # D = root_X.r - hipr_X.r
    D = hipr_X.r - root_X.r
    print "D: ", D

    A = np.linalg.norm(hipr_X.r - kneer_X.r)
    print "A: ", A

    B = np.linalg.norm(kneer_X.r - footr_X.r)
    print "B: ", B

    # crotch from ankle
    # r = Foot.R’ * (Body.p + Body.R * [0 D 0]’- Foot.p)
    r = foot_right.E.T.dot(root.r + root.E.dot(D) - foot_right.r)
    print "r", r

    # C = norm(r)
    C = np.linalg.norm(r)
    print "C: ", C

    # compute knee angle q5
    q5 = 0.0
    c5 = (C**2 - A**2 - B**2) / (2.0*A*B)
    print "c5: ", c5
    if c5 >= 1:
        print "c5 >= 1, i.e. q5 >= PI"
        q5 = 0.0
    elif c5 <= -1:
        print "c5 <= -1, i.e. q5 <= -PI"
        q5 = np.pi
    else:
        q5 = np.arccos(c5)  # knee pitch

    # compute ankle pitch
    q6a = np.arcsin((A/C)*np.sin(np.pi - q5))  # ankle pitch sub

    # compute ankle roll
    q7 = np.arctan2(r[1], r[2])  # ankle roll -pi/2 < q(6) < pi/2
    print "q7: ", q7
    if q7 > np.pi/2.:
        q7 = q7 - np.pi
    elif q7 < -np.pi/2.:
        q7 = q7 + np.pi

    # compute ankle pitch
    q6 = -np.arctan2(r[0], np.sign(r[2])*np.sqrt(r[1]**2 + r[2]**2)) - q6a
    print "q6: ", q6

    # R = Body.R’ * Foot.R * Rroll(-q7) * Rpitch(-q6-q5) # hipZ*hipX*hipY
    # TODO check roll, yaw, pitch
    R = root.E.T.dot(foot_right.E).dot(rotx(-q7)).dot(roty(-q6 - q5))
    # R = np.eye(3)
    print "R: \n", R
    print "R^T * R = \n", R.T.dot(R)

    # hip yaw
    q2 = np.arctan2(-R[0, 1], R[1,1])
    print "q2: ", q2

    # hip roll
    cz = np.cos(q2)
    sz = np.sin(q2)
    q3 = np.arctan2(R[2, 1], -R[0, 1]*sz + R[1, 1]*cz)
    print "q3: ", q3

    # hip pitch
    q4 = np.arctan2(-R[2, 0], R[2, 2])

    # sort in joint angles
    q[6:12] = [q2, q3, q4, q5, q6, q7]
    print "q: ", q

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
    root_desired.r = np.array([0.0, 0.0, 1.0])
    root_desired.E = np.eye(3)

    footr_desired = Pose()
    footr_desired.r = np.array([0.0, -0.25, 0.5])
    footr_desired.E = np.eye(3)

    footl_desired = Pose()
    footl_desired.r = np.array([0.0, 0.0, 0.5])
    footl_desired.E = np.eye(3)

    # call to inverse kinematics
    q_out, qdot_out, qddot_out = ik_leg(
        model,
        root_desired, footr_desired, footl_desired
    )

    # create MeshUp animation file
    meshup_data = np.zeros([2, 1 + model.dof_count])
    meshup_data[1, 0] = 0.0  # time
    meshup_data[0, 1:] = q_ini  # q

    meshup_data[1, 0] = 1.0  # time
    meshup_data[1, 1:] = q_out  # q

    meshup_path = os.path.join("out", "ik_test.csv")
    np.savetxt(meshup_path, meshup_data, fmt="%.18f", delimiter=", ", newline="\n", comments="", header=HEADER)
