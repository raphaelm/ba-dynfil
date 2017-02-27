import os
from collections import namedtuple, OrderedDict

import click
import numpy as np

from dynfil import constants, zmp, kinematics, filter
from dynfil.commands import main
from dynfil.filter import update_derivs
from dynfil.kinematics import interpolate_savgol
from dynfil.utils import plot
from dynfil.utils.cli import status
from dynfil.utils.meshup import save_to_meshup

Configuration = namedtuple('Configuration', 'filter iterations interpolate')
CONFIGS = [
    Configuration('newton', 5, 'none'),
    Configuration('newton', 5, 'savgol'),
    Configuration('leastsquares', 5, 'none'),
    Configuration('leastsquares', 5, 'savgol'),
    Configuration('pc', 5, 'none'),
]


def thesis_plots(timesteps, lsole, rsole, chest, com_calc, zmp_ref, zmp_calc, zmp_filtered, chest_filtered, out_dir):
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
        filename=os.path.join(out_dir, 'trajectories_on_ground_with_filtered.pdf'),
    )

    plot.plot_trajectories_from_top(
        trajectories=[
            plot.PlotTrajectory(positions=chest.traj_pos, rotations=lsole.traj_ort, label='CoM from PG', color='r',
                                linestyle=(0, (5, 1))),
            plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM unfiltered', color='c',
                                linestyle=(0, (1, 1))),
            plot.PlotTrajectory(positions=chest_filtered.traj_pos, rotations=None, label='CoM filtered', color='k'),
        ],
        filename=os.path.join(out_dir, 'trajectories_waist_with_filtered.pdf'),
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
        filename=os.path.join(out_dir, 'zmp_filtered.pdf'),
    )

    plot.plot_trajectories_1d_axis(
        timesteps,
        trajectories=[
            plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP calc', color='r'),
            plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP ref', color='g'),
            plot.PlotTrajectory(positions=zmp_calc - zmp_ref, rotations=None, label='ZMP diff', color='k'),
            plot.PlotTrajectory(positions=zmp_filtered, rotations=None, label='ZMP filtered', color='k'),
        ],
        filename=os.path.join(out_dir, 'zmp_components.pdf'),
        title='ZMP components'
    )

    plot.plot_residuums(
        data=[
            plot.PlotResiduum(times=timesteps, values=zmp_calc - zmp_ref, color='b'),
            plot.PlotResiduum(times=timesteps, values=zmp_filtered - zmp_ref, color='g')
        ],
        filename=os.path.join(out_dir, 'residuums.pdf'),
    )

    plot.plot_res_histo(
        data=[
            plot.PlotResiduum(times=timesteps, values=zmp_filtered - zmp_ref, color='g')
        ],
        filename=os.path.join(out_dir, 'residuum_histo.pdf'),
    )


@main.main.command()
@click.pass_context
def evaluate(ctx):
    model = ctx.obj['model']
    chest = ctx.obj['chest']
    lsole = ctx.obj['lsole']
    rsole = ctx.obj['rsole']
    timesteps = ctx.obj['timesteps']
    zmp_ref = ctx.obj['zmp_ref']

    residuums = OrderedDict()

    # initial pose: half-sitting
    q_ini = constants.POSE_WALK_INITIAL_SIMPLE

    # First ZMP calculation
    with status('Calculate ZMP from forward run'):
        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate='savgol', method='analytical'
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

    errs = zmp_calc - zmp_ref
    normed_errors = (errs * errs).sum(axis=1) ** 0.5
    residuums['no filter'] = normed_errors

    for filter_method, iterations, interpolate in CONFIGS:
        confstr = '{}-{}-{}'.format(filter_method, iterations, interpolate)
        chest_next = chest.copy()

        for i in range(iterations):
            out_dir = os.path.join(ctx.obj['out_dir'], '{}-{}-{}'.format(filter_method, i + 1, interpolate))
            if not os.path.exists(out_dir):
                os.mkdir(out_dir)

            with status('{} - Apply dynamic filter iteration {}'.format(confstr, i + 1)):
                chest_next = filter.filters[filter_method](
                    chest=chest_next, lsole=lsole, rsole=rsole, zmp_ref=zmp_ref, q_ini=q_ini,
                    model=model, times=timesteps, ik_method='analytical'
                )

                chest_filtered = chest_next.copy()
                if interpolate == 'savgol':
                    interpolate_savgol(chest_filtered.traj_pos)
                update_derivs(chest_filtered, timesteps)

            # Calculate ZMP from filtered result
            with status('{} - Calculate ZMP from filtered data after iteration {}'.format(confstr, i + 1)):
                q_filtered, qdot_filtered, qddot_filtered = kinematics.inverse_with_derivatives(
                    model, q_ini, chest_filtered, lsole, rsole, timesteps, interpolate='savgol', method='analytical'
                )
                zmp_filtered = zmp.calculate_zmp_trajectory(model, q_filtered, qdot_filtered, qddot_filtered,
                                                            chest_filtered)

            # Save meshup files
            with status('{} - Export MeshUp files after iteration {}'.format(confstr, i + 1)):
                save_to_meshup(os.path.join(ctx.obj['out_dir'], 'inverse_from_pg.csv'), timesteps, q_calc)
                save_to_meshup(os.path.join(ctx.obj['out_dir'], 'inverse_after_filter.csv'), timesteps, q_filtered)

            # Generatae plots
            with status('{} - Generate plots after iteration {}'.format(confstr, i + 1)):
                thesis_plots(timesteps, lsole, rsole, chest, com_calc, zmp_ref, zmp_calc, zmp_filtered, chest_filtered,
                             out_dir)

            errs = zmp_filtered - zmp_ref
            normed_errors = (errs * errs).sum(axis=1) ** 0.5
            residuums['{}-{}-{}'.format(filter_method, i + 1, interpolate)] = normed_errors

    with status('Generate boxplot'):
        plot.plot_comparison(
            OrderedDict(reversed(list(residuums.items()))),
            filenames=[
                os.path.join(ctx.obj['out_dir'], 'comparison.pgf'),
                os.path.join(ctx.obj['out_dir'], 'comparison.pdf'),
            ]
        )
