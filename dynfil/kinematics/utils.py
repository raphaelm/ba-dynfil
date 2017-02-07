import numpy as np
from scipy.signal import savgol_filter


def ik_derivs_fd(q, times):
    qdot = np.zeros((len(q), len(q[0])))
    for t in range(len(q)):  # Iterate over timesteps
        # First values: Take naive finite differences
        if t > 0 and times[t] > times[t - 1]:
            qdot[t] = (q[t] - q[t - 1]) / (times[t] - times[t - 1])
        else:
            qdot[t] = np.zeros(len(q[0]))

    return qdot


def interpolate_savgol(qddot):
    savgol_window_size = 101
    savgol_poly_order = 3
    savgol_mode = "interp"

    for i in range(len(qddot[0])):
        filtered = savgol_filter(qddot[:, i], savgol_window_size, savgol_poly_order, mode=savgol_mode)
        qddot[:, i] = filtered
