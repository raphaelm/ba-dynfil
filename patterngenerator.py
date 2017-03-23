import os

import click
import numpy as np

from dynfil.models import HeiCubModel, SimpleModelLX5, SimpleModelLX10, SimpleModelLX2
from dynfil.models import SimpleModel
from dynfil.previewcontrol import online_preview_control
from dynfil.utils import plot
from dynfil.utils.linalg import tridag

STEP_SINGLE_SUPPORT_TIME = 0.7
STEP_DOUBLE_SUPPORT_TIME = 0.7
N_STEPS = 6
RESOLUTION = 0.005
GRAVITY = 9.81
WAIT_TIME = 0.5
LINEAR_INTERPOLATION = True


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
        p(np.arange(0, time_length + RESOLUTION/2, RESOLUTION)),
        dp(np.arange(0, time_length + RESOLUTION/2, RESOLUTION)),
        ddp(np.arange(0, time_length + RESOLUTION/2, RESOLUTION))
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
        p(np.arange(0, time_length + RESOLUTION/2, RESOLUTION)),
        dp(np.arange(0, time_length + RESOLUTION/2, RESOLUTION)),
        ddp(np.arange(0, time_length + RESOLUTION/2, RESOLUTION))
    )


def zmp_shift(pos_from, pos_to, tlen):
    data = np.zeros(tlen)
    if LINEAR_INTERPOLATION:
        lspace = np.linspace(0, 1, tlen)
        data[:] = lspace * pos_to + (1 - lspace) * pos_from
    else:
        data[:] = np.ones(tlen) * pos_to
    return data


@click.command()
@click.option('--model', default='simple', help='Model', type=click.Choice(['simple', 'heicub', 'simplelx5',
                                                                            'simplelx10', 'simplelx2']))
@click.option('--out-dir', default='out/', help='Output directory')
def main(model, out_dir):
    if model == "heicub":
        model = HeiCubModel()
    elif model == "simple":
        model = SimpleModel()
    elif model == "simplelx2":
        model = SimpleModelLX2()
    elif model == "simplelx5":
        model = SimpleModelLX5()
    elif model == "simplelx10":
        model = SimpleModelLX10()

    if not os.path.exists(out_dir):
        os.mkdir(out_dir)

    STEP_LENGTH = model.step_length
    FOOT_DISTANCE = model.foot_distance
    COM_HEIGHT = model.com_height

    timesteps = np.arange(0,
                          WAIT_TIME * 2 + STEP_DOUBLE_SUPPORT_TIME + (N_STEPS + 1) * (STEP_SINGLE_SUPPORT_TIME + STEP_DOUBLE_SUPPORT_TIME) + 1,
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

    # Shift ZMP to left foot
    lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + double_support_timesteps, 1] = zmp_shift(0, current_lfoot[1], double_support_timesteps)
    t += double_support_timesteps

    # Generate foot and ZMP trajectories
    while t < wait_steps + len(timesteps) - 3 * (single_support_timesteps + double_support_timesteps):
        # Step with right foot
        r_i_xy = i_xy if t > wait_steps + double_support_timesteps else i_xy_half
        rfoot[t:t + single_support_timesteps + 1, 2] = i_z[0]
        drfoot[t:t + single_support_timesteps + 1, 2] = i_z[1]
        ddrfoot[t:t + single_support_timesteps + 1, 2] = i_z[2]
        rfoot[t:t + single_support_timesteps + 1, 0] = current_rfoot[0] + r_i_xy[0]
        drfoot[t:t + single_support_timesteps + 1, 0] = r_i_xy[1]
        ddrfoot[t:t + single_support_timesteps + 1, 0] = r_i_xy[2]
        lfoot[t:t + single_support_timesteps, 0] = np.ones(single_support_timesteps) * current_lfoot[0]
        zmp[t:t + single_support_timesteps, 0] = np.ones(single_support_timesteps) * current_lfoot[0]
        zmp[t:t + single_support_timesteps, 1] = np.ones(single_support_timesteps) * current_lfoot[1]

        t += single_support_timesteps
        current_rfoot = rfoot[t - 1]

        # Shift ZMP
        lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
        rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
        zmp[t:t + double_support_timesteps, 0] = zmp_shift(current_lfoot[0], current_rfoot[0], double_support_timesteps)
        zmp[t:t + double_support_timesteps, 1] = zmp_shift(current_lfoot[1], current_rfoot[1], double_support_timesteps)
        t += double_support_timesteps

        # Step with left foot
        lfoot[t:t + single_support_timesteps + 1, 2] = i_z[0]
        dlfoot[t:t + single_support_timesteps + 1, 2] = i_z[1]
        ddlfoot[t:t + single_support_timesteps + 1, 2] = i_z[2]
        lfoot[t:t + single_support_timesteps + 1, 0] = current_lfoot[0] + i_xy[0]
        dlfoot[t:t + single_support_timesteps + 1, 0] = i_xy[1]
        ddlfoot[t:t + single_support_timesteps + 1, 0] = i_xy[2]
        rfoot[t:t + single_support_timesteps, 0] = np.ones(single_support_timesteps) * current_rfoot[0]
        zmp[t:t + single_support_timesteps, 0] = np.ones(single_support_timesteps) * current_rfoot[0]
        zmp[t:t + single_support_timesteps, 1] = np.ones(single_support_timesteps) * current_rfoot[1]

        t += single_support_timesteps
        current_lfoot = lfoot[t - 1]

        # Shift ZMP
        lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
        rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
        zmp[t:t + double_support_timesteps, 0] = zmp_shift(current_rfoot[0], current_lfoot[0], double_support_timesteps)
        zmp[t:t + double_support_timesteps, 1] = zmp_shift(current_rfoot[1], current_lfoot[1], double_support_timesteps)
        t += double_support_timesteps

    # Move feed together, last step
    rfoot[t:t + single_support_timesteps + 1, 2] = i_z[0]
    drfoot[t:t + single_support_timesteps + 1, 2] = i_z[1]
    ddrfoot[t:t + single_support_timesteps + 1, 2] = i_z[2]
    rfoot[t:t + single_support_timesteps + 1, 0] = current_rfoot[0] + i_xy_half[0]
    drfoot[t:t + single_support_timesteps + 1, 0] = i_xy_half[1]
    ddrfoot[t:t + single_support_timesteps + 1, 0] = i_xy_half[2]
    lfoot[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + single_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + single_support_timesteps, 1] = np.ones(double_support_timesteps) * current_lfoot[1]
    t += single_support_timesteps
    current_rfoot = rfoot[t - 1]

    # Shift ZMP to middle
    lfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    rfoot[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_rfoot[0]
    zmp[t:t + double_support_timesteps, 0] = np.ones(double_support_timesteps) * current_lfoot[0]
    zmp[t:t + double_support_timesteps, 1] = zmp_shift(current_lfoot[1], 0, double_support_timesteps)
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
    np.savetxt(out_dir + '/pg_data.txt', filedata, delimiter=' ')


    # Plots
    plot.plot_trajectories_1d_axis(
        timesteps[:t],
        trajectories=[
            plot.PlotTrajectory(positions=zmp[:, 0:2], rotations=None, label=None, color='k'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_zmp_over_time.pgf'),
        ],
        #title='Planned ZMP trajectory'
    )

    plot.plot_trajectories_1d_axis_combined(
        timesteps[:t],
        trajectories=[
            plot.PlotTrajectory(positions=zmp[:, 0:2], rotations=None, label='ZMP', color='k', linestyle='dotted'),
            plot.PlotTrajectory(positions=com[:, 0:2], rotations=None, label='CoM', color='k'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_zmp_com_over_time.pgf'),
        ],
        #title='Planned ZMP trajectory'
    )

    plot.plot_trajectories_1d_axis(
        timesteps[:t],
        trajectories=[
            plot.PlotTrajectory(positions=lfoot, rotations=None, label='left foot', color='r'),
            plot.PlotTrajectory(positions=rfoot, rotations=None, label='right foot', color='g'),
            plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
            plot.PlotTrajectory(positions=com, rotations=None, label='CoM', color='b'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_trajectories_over_time.pgf')
        ],
        title='Raw trajectories'
    )

    plot.plot_trajectories_1d_axis(
        timesteps[:t],
        trajectories=[
            plot.PlotTrajectory(positions=dlfoot, rotations=None, label='left foot dot', color='r'),
            plot.PlotTrajectory(positions=drfoot, rotations=None, label='right foot dot', color='g'),
            plot.PlotTrajectory(positions=com_dot, rotations=None, label='CoM dot', color='b'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_derivs.pgf'),
        ],
        title='Derivatives'
    )

    plot.plot_trajectories_1d_axis(
        timesteps[:t],
        trajectories=[
            plot.PlotTrajectory(positions=ddlfoot, rotations=None, label='left foot ddot', color='r'),
            plot.PlotTrajectory(positions=ddrfoot, rotations=None, label='right foot ddot', color='g'),
            plot.PlotTrajectory(positions=com_ddot, rotations=None, label='CoM ddot', color='b'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_derivs_ddot.pgf')
        ],
        title='Second Derivatives'
    )

    plot.plot_trajectories(
        trajectories=[
            plot.PlotTrajectory(positions=lfoot, rotations=None, label='Left foot', color='r'),
            plot.PlotTrajectory(positions=rfoot, rotations=None, label='Right foot', color='g'),
            plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_trajectories_3d_input.pgf')
        ],
        title='Input trajectories'
    )

    plot.plot_trajectories(
        trajectories=[
            plot.PlotTrajectory(positions=lfoot, rotations=None, label='Left foot', color='r'),
            plot.PlotTrajectory(positions=rfoot, rotations=None, label='Right foot', color='g'),
            plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
            plot.PlotTrajectory(positions=com, rotations=None, label='CoM', color='b'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_trajectories_3d.pgf')
        ],
        title='Trajectories'
    )

    plot.plot_trajectories_from_top(
        trajectories=[
            plot.PlotTrajectory(positions=lfoot, rotations=None, label='Left foot', color='r'),
            plot.PlotTrajectory(positions=rfoot, rotations=None, label='Right foot', color='g'),
            plot.PlotTrajectory(positions=zmp, rotations=None, label='ZMP', color='k'),
            plot.FootTrajectory(positions=lfoot, rotations=None, color='r'),
            plot.FootTrajectory(positions=rfoot, rotations=None, color='g'),
        ],
        filenames=[
            os.path.join(out_dir, 'pg_trajectories_ground_input.pgf')
        ],
        #title='2D Trajectories on the ground'
    )

    #plot.show_all()


main()
