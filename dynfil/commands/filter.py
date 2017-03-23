import os

import click
import numpy as np

from dynfil import constants, zmp, kinematics, filter
from dynfil.commands import main
from dynfil.filter import update_derivs
from dynfil.kinematics import interpolate_savgol
from dynfil.utils import plot
from dynfil.utils.cli import status
from dynfil.utils.meshup import save_to_meshup


@main.main.command(name='filter')
@click.pass_context
@click.option('--filter-method', type=click.Choice(filter.filters.keys()),
              help='Filter method', default='newton')
@click.option('--ik-method', type=click.Choice(['numerical', 'analytical']),
              help='IK method', default='numerical')
@click.option('--iterations', type=click.IntRange(0, 100), default=5, help='Number of filter iterations')
@click.option('--interpolate', type=click.Choice(['none', 'savgol']), default='none',
              help='Apply interpolation to filter result')
def run_filter(ctx, filter_method, interpolate, iterations, ik_method):
    click.echo(click.style('Filter method:    {}'.format(filter_method), fg='blue'))
    click.echo(click.style('IK method:        {}'.format(ik_method), fg='blue'))
    click.echo(click.style('Iterations:       {}'.format(iterations), fg='blue'))
    click.echo(click.style('Interpolation:    {}'.format(interpolate), fg='blue'))
    click.echo()

    model = ctx.obj['model']
    chest = ctx.obj['chest']
    lsole = ctx.obj['lsole']
    rsole = ctx.obj['rsole']
    timesteps = ctx.obj['timesteps']
    zmp_ref = ctx.obj['zmp_ref']

    # initial pose: half-sitting
    q_ini = model.initial_pose_walking

    # First ZMP calculation
    with status('Calculate ZMP from forward run'):
        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate='savgol', method=ik_method
        )

        com_calc = kinematics.com_trajectory(model, chest, q_calc)
        zmp_calc = zmp.calculate_zmp_trajectory(model, q_calc, qdot_calc, qddot_calc, chest)

    with status('Export inverse kinematics results as files'):
        export_data = np.zeros((q_calc.shape[0], 1 + q_calc.shape[1]))
        export_data[:, 0] = timesteps
        export_data[:, 1:] = q_calc
        np.savetxt('out/q_calc.txt', export_data, fmt="%.18f", delimiter=", ", newline="\n", comments="")
        export_data[:, 1:] = qdot_calc
        np.savetxt('out/qdot_calc.txt', export_data, fmt="%.18f", delimiter=", ", newline="\n", comments="")
        export_data[:, 1:] = qddot_calc
        np.savetxt('out/qddot_calc.txt', export_data, fmt="%.18f", delimiter=", ", newline="\n", comments="")

    # Apply dynamic filter
    with status('Apply dynamic filter') as status_update:
        chest_filtered = chest.copy()
        for i in range(iterations):
            status_update(i + 1, iterations)
            chest_filtered = filter.filters[filter_method](
                chest=chest_filtered, lsole=lsole, rsole=rsole, zmp_ref=zmp_ref, q_ini=q_ini,
                model=model, times=timesteps, ik_method=ik_method
            )

        if interpolate == 'savgol':
            interpolate_savgol(chest_filtered.traj_pos)
        update_derivs(chest_filtered, timesteps)

    # Calculate ZMP from filtered result
    with status('Calculate ZMP from filtered data'):
        q_filtered, qdot_filtered, qddot_filtered = kinematics.inverse_with_derivatives(
            model, q_ini, chest_filtered, lsole, rsole, timesteps, interpolate='savgol', method=ik_method
        )
        zmp_filtered = zmp.calculate_zmp_trajectory(model, q_filtered, qdot_filtered, qddot_filtered, chest_filtered)

    # Save meshup files
    with status('Export MeshUp files'):
        save_to_meshup(os.path.join(ctx.obj['out_dir'], 'inverse_from_pg.csv'), timesteps, q_calc,
                       header=model.meshup_header)
        save_to_meshup(os.path.join(ctx.obj['out_dir'], 'inverse_after_filter.csv'), timesteps, q_filtered,
                       header=model.meshup_header)
        np.savetxt(os.path.join(ctx.obj['out_dir'], 'q.csv'),
                   q_calc, fmt="%.18f", delimiter=", ", newline="\n", comments="")
        np.savetxt(os.path.join(ctx.obj['out_dir'], 'qdot.csv'),
                   qdot_calc, fmt="%.18f", delimiter=", ", newline="\n", comments="")
        np.savetxt(os.path.join(ctx.obj['out_dir'], 'qddot.csv'),
                   qddot_calc, fmt="%.18f", delimiter=", ", newline="\n", comments="")

    # Generate plots
    with status('Generate plots'):
        plot.plot_q_values(
            timesteps,
            (q_calc, q_filtered),
            labels=('forward run', 'filtered'),
            filenames=[os.path.join(ctx.obj['out_dir'], 'q_all.pdf')],
            title='Filter results on q trajectories'
        )
        plot.plot_q_values(
            timesteps,
            (qdot_calc, qdot_filtered),
            labels=('forward run', 'filtered'),
            filenames=[os.path.join(ctx.obj['out_dir'], 'qdot_all.pdf')],
            title='Filter results on qdot'
        )
        plot.plot_q_values(
            timesteps,
            (qddot_calc, qddot_filtered),
            labels=('forward run', 'filtered'),
            filenames=[os.path.join(ctx.obj['out_dir'], 'qddot_all.pdf')],
            title='Filter results on qddot'
        )

        """
        plot.plot_trajectories(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=chest.traj_ort, label='PG: CoM', color='y'),
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot',
                                    color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot',
                                    color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
            ],
            filename=os.path.join(ctx.obj['out_dir'], 'trajectories.pdf'),
            title='3D Trajectories (reference and forward run)'
        )

        plot.plot_trajectories(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=chest.traj_ort, label='PG: CoM', color='y'),
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot',
                                    color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot',
                                    color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
                plot.PlotTrajectory(positions=chest_filtered.traj_pos, rotations=None, label='Dynfil: CoM', color='b'),
                plot.PlotTrajectory(positions=zmp_filtered, rotations=None, label='Dynfil: ZMP', color='k'),
            ],
            filename=os.path.join(ctx.obj['out_dir'], 'trajectories_with_filtered.pdf'),
            title='3D Trajectories (with filtered)'
        )

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot',
                                    color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot',
                                    color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
            ],
            filename=os.path.join(ctx.obj['out_dir'], 'trajectories_on_ground.pdf'),
            title='2D Trajectories on the ground (reference and forward run)'
        )

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=lsole.traj_ort, label='PG: CoM', color='r'),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
            ],
            filename=os.path.join(ctx.obj['out_dir'], 'trajectories_waist.pdf'),
            title='2D Trajectories on waist height ground (reference and forward run)'
        )
        """

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot',
                                    color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot',
                                    color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m',
                                    linestyle=(0, (5, 1))),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP unfiltered', color='c',
                                    linestyle=(0, (1, 1))),
                plot.PlotTrajectory(positions=zmp_filtered, rotations=None, label='ZMP filtered', color='k'),
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'trajectories_on_ground_with_filtered.pgf')],
            # title='2D Trajectories on the ground (with filtered)'
        )

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=lsole.traj_ort, label='CoM from PG', color='r',
                                    linestyle=(0, (5, 1))),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM unfiltered', color='c',
                                    linestyle=(0, (1, 1))),
                plot.PlotTrajectory(positions=chest_filtered.traj_pos, rotations=None, label='CoM filtered', color='k'),
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'trajectories_waist_with_filtered.pgf')],
            # title='2D Trajectories on waist height ground (with filtered)'
        )

        plot.plot_trajectories_1d_axis_combined(
            timesteps,
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos[:, 0:2], rotations=None, label='CoM from PG', color='r',
                                    linestyle=(0, (5, 1))),
                plot.PlotTrajectory(positions=com_calc[:, 0:2], rotations=None, label='CoM unfiltered', color='c',
                                    linestyle=(0, (1, 1))),
                plot.PlotTrajectory(positions=chest_filtered.traj_pos[:, 0:2], rotations=None, label='CoM filtered',
                                    color='k'),
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'com_filtered.pgf')],
            # title='ZMP components'
        )

        plot.plot_trajectories_1d_axis_combined(
            timesteps,
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos_dot[:, 0:2], rotations=None, label='dCoM from PG', color='r',
                                    linestyle=(0, (5, 1))),
                plot.PlotTrajectory(positions=chest_filtered.traj_pos_dot[:, 0:2], rotations=None, label='dCoM filtered',
                                    color='k'),
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'com_dot_filtered.pgf')],
            # title='ZMP components'
        )

        plot.plot_trajectories_1d_axis_combined(
            timesteps,
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos_ddot[:, 0:2], rotations=None, label='ddCoM from PG', color='r',
                                    linestyle=(0, (5, 1))),
                plot.PlotTrajectory(positions=chest_filtered.traj_pos_ddot[:, 0:2], rotations=None, label='ddCoM filtered',
                                    color='k'),
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'com_ddot_filtered.pgf')],
            # title='ZMP components'
        )

        plot.plot_trajectories_1d_axis_combined(
            timesteps,
            trajectories=[
                plot.PlotTrajectory(positions=zmp_calc[:, 0:2], rotations=None, label='ZMP unfiltered', color='c',
                                    linestyle=(0, (1, 1))),
                plot.PlotTrajectory(positions=zmp_ref[:, 0:2], rotations=None, label='ZMP reference', color='m',
                                    linestyle=(0, (5, 1))),
                plot.PlotTrajectory(positions=zmp_filtered[:, 0:2], rotations=None, label='ZMP filtered', color='k'),
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'zmp_filtered.pgf')],
            # title='ZMP components'
        )

        plot.plot_trajectories_1d_axis(
            timesteps,
            trajectories=[
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP calc', color='r'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP ref', color='g'),
                plot.PlotTrajectory(positions=zmp_calc - zmp_ref, rotations=None, label='ZMP diff', color='k'),
                plot.PlotTrajectory(positions=zmp_filtered, rotations=None, label='ZMP filtered', color='k'),
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'zmp_components.pgf')],
            title='ZMP components'
        )

        plot.plot_residuums(
            data=[
                plot.PlotResiduum(times=timesteps, values=zmp_calc - zmp_ref,
                                  # label=r'$\left\|\mathbf{r_{ZMP}} - \mathbf{r_{ZMP}}^{ref}\right\|$ without filter',
                                  color='b'),
                plot.PlotResiduum(times=timesteps, values=zmp_filtered - zmp_ref,
                                  # label=r'$\left\|\mathbf{r_{ZMP}} - \mathbf{r_{ZMP}}^{ref}\right\|$ with filter',
                                  color='g')
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'residuums.pgf')],
            # title='Filter result summary'
        )

        plot.plot_res_histo(
            data=[
                plot.PlotResiduum(times=timesteps, values=zmp_filtered - zmp_ref,
                                  # label=r'$\left\|\mathbf{r_{ZMP}} - \mathbf{r_{ZMP}}^{ref}\right\|$ with filter',
                                  color='g')
            ],
            filenames=[os.path.join(ctx.obj['out_dir'], 'residuum_histo.pgf')],
            # title='Filter result summary'
        )

    if ctx.obj['show']:
        plot.show_all()
