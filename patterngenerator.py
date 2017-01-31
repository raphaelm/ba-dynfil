import numpy as np
import rbdl
import os

from dynfil.utils import plot

STEP_LENGTH = 0.2172 * 0.7  # 70% of foot length
STEP_SINGLE_SUPPORT_TIME = 0.7
STEP_DOUBLE_SUPPORT_TIME = 0.7
FOOT_DISTANCE = 0.5
TIME_LENGTH = 6 * (STEP_SINGLE_SUPPORT_TIME + STEP_DOUBLE_SUPPORT_TIME) + 1
RESOLUTION = 0.001
COM_HEIGHT = 0.444239


def interpol_foot_xy(time_length, final_pos):
    coef = np.zeros(6)
    coef[0] = 0.
    coef[1] = 0.
    coef[2] = 0.
    coef[3] = (10. * final_pos)/(time_length**3)
    coef[4] = (- 15. * final_pos)/(time_length**4)
    coef[5] = (6. * final_pos)/(time_length**5)
    p = np.polynomial.Polynomial(coef)
    return p(np.arange(0, time_length, RESOLUTION))


def interpol_foot_z(time_length, height):
    coef = np.zeros(5)
    coef[0] = 0.
    coef[1] = 0.
    coef[2] = (16 * height)/(time_length**2)
    coef[3] = (-32 * height)/(time_length**3)
    coef[4] = (16 * height)/(time_length**4)
    p = np.polynomial.Polynomial(coef)
    return p(np.arange(0, time_length, RESOLUTION))


timesteps = np.arange(0, TIME_LENGTH, RESOLUTION)
lfoot = np.zeros((len(timesteps), 3))
rfoot = np.zeros((len(timesteps), 3))
zmp = np.zeros((len(timesteps), 3))
lfoot[:, 1] = np.ones(len(timesteps)) * FOOT_DISTANCE / 2.
rfoot[:, 1] = np.ones(len(timesteps)) * - FOOT_DISTANCE / 2.

# Walk in x direction
t = 0
current_lfoot = lfoot[0]
current_rfoot = rfoot[0]
single_support_timesteps = int(STEP_SINGLE_SUPPORT_TIME / RESOLUTION)
double_support_timesteps = int(STEP_DOUBLE_SUPPORT_TIME / RESOLUTION)

while t < len(timesteps) - 2 * (single_support_timesteps + double_support_timesteps):

    # Step with right foot
    rfoot[t:t + single_support_timesteps + 1, 2] = interpol_foot_z(STEP_SINGLE_SUPPORT_TIME, 0.02)
    rfoot[t:t + single_support_timesteps + 1, 0] = current_rfoot[0] + interpol_foot_xy(
        STEP_SINGLE_SUPPORT_TIME,
        STEP_LENGTH if t > 0 else STEP_LENGTH / 2  # First step is a half step
    )
    lfoot[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_lfoot[1]

    t += single_support_timesteps
    current_rfoot = rfoot[t - 1]

    # Shift ZMP
    lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_rfoot[1]
    t += double_support_timesteps

    # Step with left foot
    lfoot[t:t + single_support_timesteps + 1, 2] = interpol_foot_z(STEP_SINGLE_SUPPORT_TIME, 0.02)
    lfoot[t:t + single_support_timesteps + 1, 0] = current_lfoot[0] + interpol_foot_xy(
        STEP_SINGLE_SUPPORT_TIME,
        STEP_LENGTH
    )
    rfoot[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_rfoot[1]

    t += single_support_timesteps
    current_lfoot = lfoot[t - 1]

    # Shift ZMP
    lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_lfoot[1]
    t += double_support_timesteps


timesteps = timesteps[:t]
lfoot = lfoot[:t]
rfoot = rfoot[:t]
zmp = zmp[:t]
zmp[:, 2] = np.zeros(t)

plot.plot_trajectories_1d_axis(
    timesteps[:t],
    trajectories=[
        plot.PlotTrajectory(positions=lfoot, rotations=None, label='left foot', color='r'),
        plot.PlotTrajectory(positions=rfoot, rotations=None, label='right foot', color='g'),
        plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
    ],
    filename=os.path.join('out', 'pg_trajectories_over_time.pdf'),
    title='Raw trajectories'
)

plot.plot_trajectories(
    trajectories=[
        plot.PlotTrajectory(positions=lfoot, rotations=None, label='Left foot', color='r'),
        plot.PlotTrajectory(positions=rfoot, rotations=None, label='Right foot', color='g'),
        plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
    ],
    filename=os.path.join('out', 'pg_trajectories_3d.pdf'),
    title='Trajectories'
)

plot.show_all()