import numpy as np


def parse_trajectory(fname):
    """
    Parses a CSV file that contains six columns (three spatial coordinates and three
    rotational ones). Returns a tuple of two two-dimensional arrays. The first array
    contains the full spatial trajectory and the second array contains the rotational
    values.
    """
    data = np.loadtxt(fname, delimiter=',')
    return data[:, 0:3], data[:, 3:6]
