import os

import numpy as np

from dynfil.previewcontrol import online_preview_control
from dynfil.utils import plot
from dynfil.utils.linalg import tridag

STEP_LENGTH = 0.2  # HeiCub: 0.2172 * 0.5  # 50% of foot length
STEP_SINGLE_SUPPORT_TIME = 0.7
STEP_DOUBLE_SUPPORT_TIME = 0.7
FOOT_DISTANCE = 0.25 * 2  # HeiCub: 0.075 * 2
N_STEPS = 6
RESOLUTION = 0.001
COM_HEIGHT = 0.60  # HeiCub: 0.444239
GRAVITY = 9.81
WAIT_TIME = 0.5


def poly_derivate(coef):
    dcoef = np.zeros(len(coef) - 1)
    for i in range(len(coef) - 1):
        dcoef[i] = (i + 1) * coef[i + 1]
    return dcoef


def interpol_foot_xy(time_length, final_pos):
    coef = np.zeros(6)
    coef[0] = 0.
    coef[1] = 0.
    coef[2] = 0.
    coef[3] = (10. * final_pos) / (time_length ** 3)
    coef[4] = (- 15. * final_pos) / (time_length ** 4)
    coef[5] = (6. * final_pos) / (time_length ** 5)
    p = np.polynomial.Polynomial(coef)

    dcoef = poly_derivate(coef)
    dp = np.polynomial.Polynomial(dcoef)

    ddcoef = poly_derivate(dcoef)
    ddp = np.polynomial.Polynomial(ddcoef)

    return (
        p(np.arange(0, time_length, RESOLUTION)),
        dp(np.arange(0, time_length, RESOLUTION)),
        ddp(np.arange(0, time_length, RESOLUTION))
    )


def interpol_foot_z(time_length, height):
    coef = np.zeros(5)
    coef[0] = 0.
    coef[1] = 0.
    coef[2] = (16 * height) / (time_length ** 2)
    coef[3] = (-32 * height) / (time_length ** 3)
    coef[4] = (16 * height) / (time_length ** 4)
    p = np.polynomial.Polynomial(coef)

    dcoef = poly_derivate(coef)
    dp = np.polynomial.Polynomial(dcoef)

    ddcoef = poly_derivate(dcoef)
    ddp = np.polynomial.Polynomial(ddcoef)

    return (
        p(np.arange(0, time_length, RESOLUTION)),
        dp(np.arange(0, time_length, RESOLUTION)),
        ddp(np.arange(0, time_length, RESOLUTION))
    )


timesteps = np.arange(0,
                      WAIT_TIME * 2 + (N_STEPS + 1) * (STEP_SINGLE_SUPPORT_TIME + STEP_DOUBLE_SUPPORT_TIME) + 1,
                      RESOLUTION)
lfoot = np.zeros((len(timesteps), 3))
rfoot = np.zeros((len(timesteps), 3))
zmp = np.zeros((len(timesteps), 3))
dlfoot = np.zeros((len(timesteps), 3))
drfoot = np.zeros((len(timesteps), 3))
dzmp = np.zeros((len(timesteps), 3))
ddlfoot = np.zeros((len(timesteps), 3))
ddrfoot = np.zeros((len(timesteps), 3))
ddzmp = np.zeros((len(timesteps), 3))
lfoot[:, 1] = np.ones(len(timesteps)) * FOOT_DISTANCE / 2.
rfoot[:, 1] = np.ones(len(timesteps)) * - FOOT_DISTANCE / 2.

# Walk in x direction
t = 0
current_lfoot = lfoot[0]
current_rfoot = rfoot[0]
single_support_timesteps = int(STEP_SINGLE_SUPPORT_TIME / RESOLUTION)
double_support_timesteps = int(STEP_DOUBLE_SUPPORT_TIME / RESOLUTION)
wait_steps = int(WAIT_TIME / RESOLUTION)

# Interpolation
i_z = interpol_foot_z(STEP_SINGLE_SUPPORT_TIME, 0.02)
i_xy = interpol_foot_xy(STEP_SINGLE_SUPPORT_TIME, STEP_LENGTH)
i_xy_half = interpol_foot_xy(STEP_SINGLE_SUPPORT_TIME, STEP_LENGTH / 2)

t += wait_steps

# Generate foot and ZMP trajectories
while t < wait_steps + len(timesteps) - 3 * (single_support_timesteps + double_support_timesteps):
    # Step with right foot
    r_i_xy = i_xy if t > wait_steps else i_xy_half
    rfoot[t:t + single_support_timesteps + 1, 2] = i_z[0]
    drfoot[t:t + single_support_timesteps + 1, 2] = i_z[1]
    ddrfoot[t:t + single_support_timesteps + 1, 2] = i_z[2]
    rfoot[t:t + single_support_timesteps + 1, 0] = current_rfoot[0] + r_i_xy[0]
    drfoot[t:t + single_support_timesteps + 1, 0] = r_i_xy[1]
    ddrfoot[t:t + single_support_timesteps + 1, 0] = r_i_xy[2]
    lfoot[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_lfoot[1]

    t += single_support_timesteps
    current_rfoot = rfoot[t - 1]

    # Shift ZMP
    lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + double_support_timesteps, 1] = np.ones(double_support_timesteps) * current_rfoot[1]
    t += double_support_timesteps

    # Step with left foot
    lfoot[t:t + single_support_timesteps + 1, 2] = i_z[0]
    dlfoot[t:t + single_support_timesteps + 1, 2] = i_z[1]
    ddlfoot[t:t + single_support_timesteps + 1, 2] = i_z[2]
    lfoot[t:t + single_support_timesteps + 1, 0] = current_lfoot[0] + i_xy[0]
    dlfoot[t:t + single_support_timesteps + 1, 0] = i_xy[1]
    ddlfoot[t:t + single_support_timesteps + 1, 0] = i_xy[2]
    rfoot[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_rfoot[1]

    t += single_support_timesteps
    current_lfoot = lfoot[t - 1]

    # Shift ZMP
    lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + double_support_timesteps, 1] = np.ones(double_support_timesteps) * current_lfoot[1]
    t += double_support_timesteps

# Move feed together, last step
rfoot[t:t + single_support_timesteps + 1, 2] = i_z[0]
drfoot[t:t + single_support_timesteps + 1, 2] = i_z[1]
ddrfoot[t:t + single_support_timesteps + 1, 2] = i_z[2]
rfoot[t:t + single_support_timesteps + 1, 0] = current_rfoot[0] + i_xy[0]
drfoot[t:t + single_support_timesteps + 1, 0] = i_xy[1]
ddrfoot[t:t + single_support_timesteps + 1, 0] = i_xy[2]
lfoot[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_lfoot[1]
t += single_support_timesteps
current_rfoot = rfoot[t - 1]

# Shift ZMP to middle
lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
zmp[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
zmp[t:t + double_support_timesteps, 1] = np.zeros(double_support_timesteps)
t += double_support_timesteps

# Wait
lfoot[t:t + wait_steps, 0] = np.ones(wait_steps) * current_lfoot[0]
rfoot[t:t + wait_steps, 0] = np.ones(wait_steps) * current_rfoot[0]
zmp[t:t + wait_steps, 0] = np.ones(wait_steps) * current_lfoot[0]
t += wait_steps

timesteps = timesteps[:t]
lfoot = lfoot[:t]
dlfoot = dlfoot[:t]
ddlfoot = ddlfoot[:t]
rfoot = rfoot[:t]
drfoot = drfoot[:t]
ddrfoot = ddrfoot[:t]
zmp = zmp[:t]
zmp[:, 2] = np.zeros(t)

"""
Offline PC
#  Calculate CoM Trajectories  (def (4.67) from p 140 in Kajita's book)
b = np.ones(t) * (2 * COM_HEIGHT / (GRAVITY * RESOLUTION ** 2) + 1)
a = c = np.ones(t - 1) * - COM_HEIGHT / (GRAVITY * RESOLUTION ** 2)
b[0] += a[0]
b[t-1] += c[-1]

# Solve eq. (4.69) from p 141 in Kajita's book
com = tridag(a, b, c, zmp)
com[:, 2] = np.ones(t) * COM_HEIGHT
np.ones((N, 2))
"""

com, com_dot, com_ddot = online_preview_control(zmp, RESOLUTION, COM_HEIGHT, len(zmp))

# Save to file
filedata = np.zeros((t, 41))
filedata[:, 0] = timesteps
filedata[:, 1:4] = com
filedata[:, 7:10] = lfoot
filedata[:, 13:16] = rfoot
filedata[:, 19:22] = zmp
filedata[:, 22:25] = dlfoot
filedata[:, 25:28] = drfoot
filedata[:, 28:31] = com_dot
filedata[:, 31:34] = ddlfoot
filedata[:, 34:37] = ddrfoot
filedata[:, 37:40] = com_ddot
np.savetxt('out/pg_data.txt', filedata, delimiter=' ')


# Plots
plot.plot_trajectories_1d_axis(
    timesteps[:t],
    trajectories=[
        plot.PlotTrajectory(positions=lfoot, rotations=None, label='left foot', color='r'),
        plot.PlotTrajectory(positions=rfoot, rotations=None, label='right foot', color='g'),
        plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
        plot.PlotTrajectory(positions=com, rotations=None, label='CoM', color='b'),
    ],
    filename=os.path.join('out', 'pg_trajectories_over_time.pdf'),
    title='Raw trajectories'
)

plot.plot_trajectories_1d_axis(
    timesteps[:t],
    trajectories=[
        plot.PlotTrajectory(positions=dlfoot, rotations=None, label='left foot dot', color='r'),
        plot.PlotTrajectory(positions=drfoot, rotations=None, label='right foot dot', color='g'),
        plot.PlotTrajectory(positions=com_dot, rotations=None, label='CoM dot', color='b'),
    ],
    filename=os.path.join('out', 'pg_derivs.pdf'),
    title='Derivatives'
)

plot.plot_trajectories_1d_axis(
    timesteps[:t],
    trajectories=[
        plot.PlotTrajectory(positions=ddlfoot, rotations=None, label='left foot ddot', color='r'),
        plot.PlotTrajectory(positions=ddrfoot, rotations=None, label='right foot ddot', color='g'),
        plot.PlotTrajectory(positions=com_ddot, rotations=None, label='CoM ddot', color='b'),
    ],
    filename=os.path.join('out', 'pg_derivs_ddot.pdf'),
    title='Second Derivatives'
)

plot.plot_trajectories(
    trajectories=[
        plot.PlotTrajectory(positions=lfoot, rotations=None, label='Left foot', color='r'),
        plot.PlotTrajectory(positions=rfoot, rotations=None, label='Right foot', color='g'),
        plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
        plot.PlotTrajectory(positions=com, rotations=None, label='CoM', color='b'),
    ],
    filename=os.path.join('out', 'pg_trajectories_3d.pdf'),
    title='Trajectories'
)

plot.show_all()