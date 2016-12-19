import numpy as np

MESHUP_HEADER = """
COLUMNS:
time,
root_link:T:X,
root_link:T:Y,
root_link:T:Z,
root_link:R:X:rad,
root_link:R:Y:rad,
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
r_ankle_2:R:-Z:rad
torso_1:R:X:rad,
torso_2:R:-Z:rad,
chest:R:-Y:rad

DATA:
"""


def save_to_meshup(filename, timesteps, q):
    rows = 2
    ts = np.linspace(0.0, 1.0, rows, endpoint=True)
    meshup_data = np.zeros((q.shape[0], 1 + q.shape[1]))
    meshup_data[:, 0] = timesteps
    meshup_data[:, 1:] = q
    np.savetxt(filename, meshup_data, fmt="%.18f", delimiter=", ", newline="\n", comments="", header=MESHUP_HEADER)
