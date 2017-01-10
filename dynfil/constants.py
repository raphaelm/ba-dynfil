import numpy as np


POSE_HALF_SITTING = np.array([
    0.063, 0, 0.605,  # x, y, z
    0, 0.25, np.pi,  # Euler x, y, z
    0.5, 0,  # l_hip x, -z
    0,  # l_up y
    -0.5,  # l_lo x
    -0.25, 0,  # l_ak -x, -z
    0.5, 0,  # r_hip x, -z
    0,  # r_up y
    -0.5,  # r_lo x
    -0.25, 0,  # r_ak -x, -z
    -0.25, 0,  # torso x, -z
    0  # chest -y
])
FOOT_LENGTH = 0.2172 - 0.04  # -2cm safety margin on each side
FOOT_WIDTH = 0.1380 - 0.04  # -2cm safety margin on each side
