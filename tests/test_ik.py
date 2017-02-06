# -*- coding: utf-8 -*-
"""Test analytic IK and derivatives against finite differences."""
import os
import numpy
import numpy as np
import pytest

from numpy.testing import (assert_allclose)

import ik
import rbdl
import dynfil.utils as utils
from dynfil.utils import (rotx, roty, rotz)


RTOL = 1e-8
ATOL = 1e-8

# prepare desired poses for CoM and feet
class Pose(object):
    pass

# load model
BASENAME = os.path.dirname(os.path.abspath(__file__))
BASENAME = os.path.dirname(BASENAME)
model_path = os.path.join(BASENAME, "data", "models", "ik_test.lua")
model_path = os.path.join(BASENAME, "data", "models", "ik_test.lua")
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

root_desired = Pose()
root_desired.r = np.array([0.0, 0.0, 0.60])
root_desired.E = utils.rotz(np.pi/8.)

root_dot_desired = Pose()
root_dot_desired.r = np.zeros([3, 4])
root_dot_desired.r[:, 0] = np.array([0.2, 0.0, -0.1])
root_dot_desired.r[:, 1:] = np.eye(3)
root_dot_desired.E = np.zeros([3, 3, 4])

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


def test_consistency_of_nominal_evaluation():
    actual, __, __ = ik.ik_one_leg(
        model, "right", root_desired, footr_desired
    )
    desired, __, __ = ik.ik_one_leg_fd(
        model, "right", root_desired, footr_desired
    )
    # print ""
    # print "actual: ", actual
    # print "desired: ", desired
    assert_allclose(actual, desired, rtol=RTOL, atol=ATOL)


def test_consistency_of_first_order_derivatives():
    __, actual, __ = ik.ik_one_leg(
        model, "right", root_desired, footr_desired,
        root_dot_desired, footr_dot_desired
    )
    __, desired, __ = ik.ik_one_leg_fd(
        model, "right", root_desired, footr_desired,
        root_dot_desired, footr_dot_desired
    )
    # print ""
    # print "actual: ", actual
    # print "desired: ", desired
    assert_allclose(actual, desired, rtol=RTOL, atol=ATOL)
