import numpy as np

MESHUP_HEADER_HEICUB = """
COLUMNS:
time,
root_link:T:-X,
root_link:T:-Y,
root_link:T:Z,
root_link:R:-X:rad,
root_link:R:-Y:rad,
root_link:R:Z:rad,
l_hip_1:R:X:rad,
l_hip_2:R:-Z:rad,
l_upper_leg:R:Y:rad,
l_lower_leg:R:X:rad,
l_ankle_1:R:-X:rad,
l_ankle_2:R:-Z:rad,
r_hip_1:R:-X:rad,
r_hip_2:R:-Z:rad,
r_upper_leg:R:-Y:rad,
r_lower_leg:R:-X:rad,
r_ankle_1:R:X:rad,
r_ankle_2:R:-Z:rad,
torso_1:R:X:rad,
torso_2:R:-Z:rad,
chest:R:-Y:rad

DATA:
"""

MESHUP_HEADER_SIMPLE = """
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


def save_to_meshup(filename, timesteps, q, header=MESHUP_HEADER_SIMPLE):
    rows = 2
    ts = np.linspace(0.0, 1.0, rows, endpoint=True)
    meshup_data = np.zeros((q.shape[0], 1 + q.shape[1]))
    meshup_data[:, 0] = timesteps
    meshup_data[:, 1:] = q
    np.savetxt(filename, meshup_data, fmt="%.18f", delimiter=", ", newline="\n", comments="",
               header=header)


def load_from_meshup(filename):
    skip_header = 0
    with open(filename, 'r') as f:
        for i, l in enumerate(f):
            if 'DATA' in l:
                skip_header = i + 1
                break
    return np.genfromtxt(filename, delimiter=",", dtype=float, skip_header=skip_header)