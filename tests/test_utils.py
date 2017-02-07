# -*- coding: utf-8 -*-
"""Test analytic IK and derivatives against finite differences."""
import os
import numpy
import numpy as np
import pytest

from numpy.testing import (assert_allclose)

import dynfil.utils as utils
from dynfil.utils import (rotx, roty, rotz)
from dynfil.utils import (rotx_dot, roty_dot, rotz_dot)
from dynfil.utils import (rotx_ddot, roty_ddot, rotz_ddot)


RTOL = 1e-7
ATOL = 1e-7

@pytest.mark.parametrize("funcs", [
    (rotx, rotx_dot),
    (roty, roty_dot),
    (rotz, rotz_dot),
])
def test_rot_dot_evaluation(funcs):
    # unpack functions
    rot = funcs[0]
    rot_dot = funcs[1]

    # prepare directions
    nom = np.pi/8.

    dot_ndirs = 6
    dot = np.random.uniform(low=-np.pi/2., high=np.pi/2., size=[dot_ndirs])

    # derivative evaluation using analytic derivative
    actual = rot_dot(nom, dot)

    # derivative evaluation using finite differences
    desired = np.zeros([3, 3, dot_ndirs])

    EPS = 1e-8
    rot_h0 = rot(nom)
    for idir in range(dot_ndirs):
        nom_h = nom + EPS * dot[idir]
        rot_h = rot(nom_h)
        desired[:, :, idir] = (rot_h - rot_h0) / EPS

    # compare derivatives
    # print ""
    # print "actual: ", actual.shape
    # for i in range(dot_ndirs):
    #     print actual[:, :, i]
    # print "desired: ", desired.shape
    # for i in range(dot_ndirs):
    #     print desired[:, :, i]
    # print "error: ", desired.shape
    # for i in range(dot_ndirs):
    #     print desired[:, :, i] - actual[:, :, i]
    assert_allclose(actual, desired, rtol=RTOL, atol=ATOL)
