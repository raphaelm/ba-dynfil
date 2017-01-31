import numpy as np


def tridag(a, b, c, r):
    """Solve linear system with tridiagonal coefficient matrix.
    a is the lower band, b is the diagonal, c is the upper band, and r
    is the right hand side. The solution is returned in u.
    [b1 c1 0 ...           ] [u1]   [r1]
    [a1 b2 c2 0 ...        ] [ :]   [ :]
    [ 0 a2 b3 c3 0 ...     ] [  ] = [  ]
    [                      ] [  ]   [  ]
    [ ...  0 an-2 bn-1 cn-1] [ :]   [ :]
    [        ... 0 an-1 bn ] [un]   [rn]

    Algorithm from

        Press, W.H., Flannery, B.P., Teukolsky, S.A., Vetterling, W.T.:
        Numerical Recipes in C. Cambridge University Press (1988)
        http://www2.units.it/ipl/students_area/imm2/files/Numerical_Recipes.pdf

    Implementation from

        http://userpages.irap.omp.eu/~bdintrans/docs/tridag.py
    """

    n = len(b)
    tmp = np.zeros(n - 1)  # necessary temporary array
    if b[0] == 0:
        raise RuntimeError('System is effectively order N-1')

    beta = b[0]
    u = np.zeros_like(r)
    u[0] = r[0] / beta
    for i in range(1, n):  # Decompose and forward substitution
        tmp[i - 1] = c[i - 1] / beta
        beta = b[i] - a[i - 1] * tmp[i - 1]
        if beta == 0:
            raise RuntimeError('Method failure')
        u[i] = (r[i] - a[i - 1] * u[i - 1]) / beta

    for i in range(n - 1, 0, -1):  # Backward substitution
        u[i - 1] -= tmp[i - 1] * u[i]

    return u
