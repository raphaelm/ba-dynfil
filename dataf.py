import os
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
from scipy.interpolate import splev, splrep
import numpy.polynomial.polynomial as poly

def smooth_signal(x,window_len=11,window='hanning'):
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also:

    np.hanning, np.hamming, np.bartlett, np.blackman, np.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError, "smooth only accepts 1 dimension arrays."

    if x.size < window_len:
        raise ValueError, "Input vector needs to be bigger than window size."


    if window_len<3:
        return x


    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"


    s=np.r_[x[window_len-1:0:-1],x,x[-1:-window_len:-1]]
    #print(len(s))
    if window == 'flat': #moving average
        w=np.ones(window_len,'d')
    else:
        w=eval('np.'+window+'(window_len)')

    y = np.convolve(w/w.sum(), s, mode='valid')
    return y


def runningMeanFast(x, N, mode):
    return np.convolve(x, np.ones((N,))/N, mode=mode)


if __name__ == "__main__":
    # retrieve data
    data = np.genfromtxt('q_calc.txt', delimiter=", ", dtype=None)
    ts, qs = data[1:, 0].copy(), data[1:, 1:].copy()  # remove first entry
    ts, idx, cnt = np.unique(ts, return_index=True, return_counts=True)
    cnt = (cnt - 1).sum()
    if cnt > 0:
        print "ts: removed {} duplicate values".format(cnt)
    qs = qs[idx, :]

    data = np.genfromtxt('qdot_calc.txt', delimiter=", ", dtype=None)
    qds = data[1:, 1:].copy()  # remove first entry
    qds = qds[idx, :]

    data = np.genfromtxt('qddot_calc.txt', delimiter=", ", dtype=None)
    qdds = data[1:, 1:].copy()  # remove first entry
    qdds = qdds[idx, :]

    # drop container
    data = None

    # slice data
    choice = (2,)
    # choice = (2,)
    # choice = (2,)
    qs = qs[:, choice]
    qds = qds[:, choice]
    qdds = qdds[:, choice]

    # interpolate
    if False:
        interpd = 32
        coefs = poly.polyfit(ts, qs, interpd)
        qs_int = poly.polyval(ts, coefs).transpose()

    # apply moving average
    mavg = True
    if mavg:
        modes = ['full', 'same', 'valid']
        mode = "same"
        qs_mavg = runningMeanFast(qs.ravel(), 51, mode)
        qds_mavg = runningMeanFast(qds.ravel(), 51, mode)
        qdds_mavg = runningMeanFast(qdds.ravel(), 51, mode)

    # smoothing functions over window
    # NOTE does not work properly and changes array size
    smooth = False
    windows=['flat', 'hanning', 'hamming', 'bartlett', 'blackman']
    if smooth:
        qs_smooth = smooth_signal(qs.ravel(), 51, windows[1])
        qds_smooth = smooth_signal(qds.ravel(), 51, windows[1])
        qdds_smooth = smooth_signal(qdds.ravel(), 51, windows[1])

    else:  # if bspline
        pass

    # find bspline interpolation
    # NOTE does not smooth but interpolate
    bspline = False
    if bspline:
        tck = splrep(ts, qs)
        qs_bspline = splev(ts, tck)

        tck = splrep(ts, qds)
        qds_bspline = splev(ts, tck)

        tck = splrep(ts, qdds)
        qdds_bspline = splev(ts, tck)

    else:  # if bspline
        pass

    # filter
    savgol = True
    tts = np.linspace(ts.min(), ts.max(), ts.size*2, endpoint=True)
    if savgol:
        print
        window_size = 101
        poly_order = 3
        mode = "interp"

        itp = interp1d(ts, qs.ravel(), kind="linear")
        qs_savgol = savgol_filter(itp(tts), window_size, poly_order, mode=mode)
        qs_savgol = savgol_filter(qs.ravel(), window_size, poly_order, mode=mode)

        itp = interp1d(ts, qds.ravel(), kind="linear")
        qds_savgol = savgol_filter(itp(tts), window_size, poly_order, mode=mode)
        qds_savgol = savgol_filter(qds.ravel(), window_size, poly_order, mode=mode)

        itp = interp1d(ts, qdds.ravel(), kind="linear")
        qdds_savgol = savgol_filter(itp(tts), window_size, poly_order, mode=mode)
        qdds_savgol = savgol_filter(qdds.ravel(), window_size, poly_order, mode=mode)
    else:  # if savgol
        pass

    # plot
    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.plot(ts, qs, ls="", marker="x", ms=2, label=r"$qs$")
    ax.plot(ts, qds, ls="", marker="x", ms=2, label=r"$qds$")
    ax.plot(ts, qdds, ls="", marker="x", ms=2, label=r"$qdds$")

    if mavg:
        ax.plot(ts, qs_mavg, label=r"$qs_{mavg}$")
        ax.plot(ts, qds_mavg, label=r"$qds_{mavg}$")
        ax.plot(ts, qdds_mavg, label=r"$qdds_{mavg}$")

    if smooth:
        ax.plot(ts, qs_smooth[:-50], label=r"$qs_{smooth}$")
        ax.plot(ts, qds_smooth[:-50], label=r"$qds_{smooth}$")
        ax.plot(ts, qdds_smooth[:-50], label=r"$qdds_{smooth}$")

    if bspline:
        ax.plot(ts, qs_bspline, label=r"$qs_{bspline}$")
        ax.plot(ts, qds_bspline, label=r"$qds_{bspline}$")
        ax.plot(ts, qdds_bspline, label=r"$qdds_{bspline}$")

    if savgol:
        ax.plot(ts, qs_savgol, label=r"$qs_{savgol}$")
        ax.plot(ts, qds_savgol, label=r"$qds_{savgol}$")
        ax.plot(ts, qdds_savgol, label=r"$qdds_{savgol}$")

    plt.legend(loc="best")
    plt.show()
