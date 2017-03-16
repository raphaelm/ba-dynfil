import os
from collections import namedtuple, OrderedDict

import click
import numpy as np
import time

from dynfil import zmp, kinematics, filter
from dynfil.commands import main
from dynfil.filter import update_derivs
from dynfil.kinematics import interpolate_savgol
from dynfil.utils import plot
from dynfil.utils.cli import status
from dynfil.utils.meshup import save_to_meshup

Configuration = namedtuple('Configuration', 'filter iterations interpolations windows')
CONFIGS = [
    Configuration('newton', 5, ('none', 'savgol'), (0,)),
    Configuration('gaussnewton', 5, ('none', 'savgol'), (0,)),
    Configuration('pc', 5, ('none',), (0.5, 1, 1.5)),
]
SPEED_RUNS = 25


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
        filenames=[
            os.path.join(out_dir, 'trajectories_on_ground_with_filtered.pdf'),
            os.path.join(out_dir, 'trajectories_on_ground_with_filtered.pgf'),
        ]
    )

    plot.plot_trajectories_from_top(
        trajectories=[
            plot.PlotTrajectory(positions=chest.traj_pos, rotations=lsole.traj_ort, label='CoM from PG', color='r',
                                linestyle=(0, (5, 1))),
            plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM unfiltered', color='c',
                                linestyle=(0, (1, 1))),
            plot.PlotTrajectory(positions=chest_filtered.traj_pos, rotations=None, label='CoM filtered', color='k'),
        ],
        filenames=[
            os.path.join(out_dir, 'trajectories_waist_with_filtered.pgf'),
            os.path.join(out_dir, 'trajectories_waist_with_filtered.pdf'),
        ]
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
        filenames=[
            os.path.join(out_dir, 'zmp_filtered.pdf'),
            os.path.join(out_dir, 'zmp_filtered.pgf'),
        ]
    )

    plot.plot_trajectories_1d_axis(
        timesteps,
        trajectories=[
            plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP calc', color='r'),
            plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP ref', color='g'),
            plot.PlotTrajectory(positions=zmp_calc - zmp_ref, rotations=None, label='ZMP diff', color='k'),
            plot.PlotTrajectory(positions=zmp_filtered, rotations=None, label='ZMP filtered', color='k'),
        ],
        filenames=[
            os.path.join(out_dir, 'zmp_components.pdf'),
            os.path.join(out_dir, 'zmp_components.pgf'),
        ]
    )

    plot.plot_residuums(
        data=[
            plot.PlotResiduum(times=timesteps, values=zmp_calc - zmp_ref, color='b'),
            plot.PlotResiduum(times=timesteps, values=zmp_filtered - zmp_ref, color='g')
        ],
        filenames=[
            os.path.join(out_dir, 'residuums.pdf'),
            os.path.join(out_dir, 'residuums.pgf'),
        ]
    )

    plot.plot_res_histo(
        data=[
            plot.PlotResiduum(times=timesteps, values=zmp_filtered - zmp_ref, color='g')
        ],
        filenames=[
            os.path.join(out_dir, 'residuum_histo.pdf'),
            os.path.join(out_dir, 'residuum_histo.pgf'),
        ]
    )


def box_plots(resdata, jerkdata, cdddata, out_dir, fname):
    plot.plot_comparison(
        OrderedDict(reversed(resdata)),
        filenames=[
            os.path.join(out_dir, 'res_{}.pdf'.format(fname)),
            os.path.join(out_dir, 'res_{}.pgf'.format(fname)),
            os.path.join(out_dir, 'res_{}.tex'.format(fname)),
        ]
    )
    plot.plot_comparison(
        OrderedDict(reversed(jerkdata)),
        filenames=[
            os.path.join(out_dir, 'jerks_{}.pdf'.format(fname)),
            os.path.join(out_dir, 'jerks_{}.pgf'.format(fname)),
            os.path.join(out_dir, 'jerks_{}.tex'.format(fname)),
        ],
        x=r'jerk [$m s^{-3}$]'
    )
    plot.plot_comparison(
        OrderedDict(reversed(cdddata)),
        filenames=[
            os.path.join(out_dir, 'cdd_{}.pdf'.format(fname)),
            os.path.join(out_dir, 'cdd_{}.pgf'.format(fname)),
            os.path.join(out_dir, 'cdd_{}.tex'.format(fname)),
        ],
        x=r'$\ddot c$ [$m s^{-2}$]'
    )


def calculate_jerks(chest, times):
    jerks = np.zeros_like(chest.traj_pos_ddot)
    for t in range(len(chest)):
        if t > 2:
            jerks[t] = (chest.traj_pos_ddot[t] - chest.traj_pos_ddot[t - 1]) / (times[t] - times[t - 1])
    return jerks


def gen_confstr(filter_method, i, interpolate, previewwindow):
    if filter_method == 'pc':
        return '{}-{}-{}-{}s'.format(filter_method, i, interpolate, previewwindow)
    else:
        return '{}-{}-{}'.format(filter_method, i, interpolate)


@main.main.command()
@click.option('--ik-method', type=click.Choice(['numerical', 'analytical']),
              help='IK method', default='analytical')
@click.pass_context
def evaluate(ctx, ik_method):
    model = ctx.obj['model']
    chest = ctx.obj['chest']
    lsole = ctx.obj['lsole']
    rsole = ctx.obj['rsole']
    timesteps = ctx.obj['timesteps']
    zmp_ref = ctx.obj['zmp_ref']

    residuums = OrderedDict()
    jerks = OrderedDict()
    cddots = OrderedDict()

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

    errs = zmp_calc - zmp_ref
    normed_errors = (errs * errs).sum(axis=1) ** 0.5
    residuums['no filter'] = normed_errors
    j = calculate_jerks(chest, timesteps)
    jerks['no filter'] = (j * j).sum(axis=1) ** 0.5
    cddots['no filter'] = (chest.traj_pos_ddot * chest.traj_pos_ddot).sum(axis=1) ** 0.5

    for filter_method, iterations, interpolations, previewwindows in CONFIGS:
        for interpolate in interpolations:
            for previewwindow in previewwindows:
                chest_next = chest.copy()

                for i in range(iterations):
                    confstr = gen_confstr(filter_method, i + 1, interpolate, previewwindow)

                    out_dir = os.path.join(ctx.obj['out_dir'], confstr)
                    if not os.path.exists(out_dir):
                        os.mkdir(out_dir)

                    with status('{} - Apply dynamic filter'.format(confstr)):
                        chest_next = filter.filters[filter_method](
                            chest=chest_next, lsole=lsole, rsole=rsole, zmp_ref=zmp_ref, q_ini=q_ini,
                            model=model, times=timesteps, ik_method=ik_method, previewwindow=previewwindow
                        )

                        chest_filtered = chest_next.copy()
                        if interpolate == 'savgol':
                            interpolate_savgol(chest_filtered.traj_pos)
                        update_derivs(chest_filtered, timesteps)

                    # Calculate ZMP from filtered result
                    with status('{} - Calculate ZMP from filtered data after'.format(confstr)):
                        q_filtered, qdot_filtered, qddot_filtered = kinematics.inverse_with_derivatives(
                            model, q_ini, chest_filtered, lsole, rsole, timesteps, interpolate='savgol',
                            method=ik_method
                        )
                        zmp_filtered = zmp.calculate_zmp_trajectory(model, q_filtered, qdot_filtered, qddot_filtered,
                                                                    chest_filtered)

                    # Save meshup files
                    with status('{} - Export MeshUp files'.format(confstr)):
                        save_to_meshup(os.path.join(ctx.obj['out_dir'], 'inverse_from_pg.csv'), timesteps, q_calc,
                                       header=model.meshup_header)
                        save_to_meshup(os.path.join(ctx.obj['out_dir'], 'inverse_after_filter.csv'), timesteps,
                                       q_filtered, header=model.meshup_header)

                    # Generatae plots
                    with status('{} - Generate plots'.format(confstr)):
                        thesis_plots(timesteps, lsole, rsole, chest, com_calc, zmp_ref, zmp_calc, zmp_filtered,
                                     chest_filtered, out_dir)

                    with status('{} - Generate data for box plots'.format(confstr)):
                        errs = zmp_filtered - zmp_ref
                        normed_errors = (errs * errs).sum(axis=1) ** 0.5
                        residuums[(filter_method, i + 1, interpolate, previewwindow)] = normed_errors
                        j = calculate_jerks(chest_filtered, timesteps)
                        jerks[(filter_method, i + 1, interpolate, previewwindow)] = (j * j).sum(axis=1) ** 0.5
                        cdd = (chest_filtered.traj_pos_ddot * chest_filtered.traj_pos_ddot).sum(axis=1) ** 0.5
                        cddots[(filter_method, i + 1, interpolate, previewwindow)] = cdd

    with status('Generate boxplots'):
        default = [('no filter', residuums['no filter'])]
        jerkdefault = [('no filter', jerks['no filter'])]
        cdddefault = [('no filter', cddots['no filter'])]
        box_plots(
            default + [(gen_confstr(*a), b) for a, b in residuums.items() if isinstance(a, tuple)],
            jerkdefault + [(gen_confstr(*a), b) for a, b in jerks.items() if isinstance(a, tuple)],
            cdddefault + [(gen_confstr(*a), b) for a, b in cddots.items() if isinstance(a, tuple)],
            ctx.obj['out_dir'],
            'comparison'
        )

        for filter_method, iterations, interpolations, pwindows in CONFIGS:
            for interpolate in interpolations:
                resdata = [(gen_confstr(*a), b) for a, b in residuums.items()
                           if a[0] == filter_method and a[2] == interpolate]
                jerkdata = [(gen_confstr(*a), b) for a, b in jerks.items()
                            if a[0] == filter_method and a[2] == interpolate]
                cdddata = [(gen_confstr(*a), b) for a, b in cddots.items()
                           if a[0] == filter_method and a[2] == interpolate]
                box_plots(
                    default + resdata,
                    jerkdefault + jerkdata,
                    cdddefault + cdddata,
                    ctx.obj['out_dir'],
                    '{}_{}'.format(filter_method, interpolate)
                )

        for filter_method, iterations, interpolations, pwindows in CONFIGS:
            for i in range(iterations):
                resdata = [(gen_confstr(*a), b) for a, b in residuums.items()
                           if a[0] == filter_method and a[1] == i + 1]
                jerkdata = [(gen_confstr(*a), b) for a, b in jerks.items()
                            if a[0] == filter_method and a[1] == i + 1]
                cdddata = [(gen_confstr(*a), b) for a, b in cddots.items()
                           if a[0] == filter_method and a[1] == i + 1]
                box_plots(
                    default + resdata,
                    jerkdefault + jerkdata,
                    cdddefault + cdddata,
                    ctx.obj['out_dir'],
                    '{}_{}'.format(filter_method, i + 1)
                )

        for filter_method, iterations, interpolations, pwindows in CONFIGS:
            for interpolate in interpolations:
                if len(pwindows) == 1:
                    continue
                for pwindow in pwindows:
                    for i in range(iterations):
                        resdata = [(gen_confstr(*a), b) for a, b in residuums.items()
                                   if a[0] == filter_method and a[2] == interpolate and a[3] == pwindow]
                        jerkdata = [(gen_confstr(*a), b) for a, b in jerks.items()
                                    if a[0] == filter_method and a[2] == interpolate and a[3] == pwindow]
                        cdddata = [(gen_confstr(*a), b) for a, b in cddots.items()
                                   if a[0] == filter_method and a[2] == interpolate and a[3] == pwindow]
                        box_plots(
                            default + resdata,
                            jerkdefault + jerkdata,
                            cdddefault + cdddata,
                            ctx.obj['out_dir'],
                            '{}_{}_{}'.format(filter_method, interpolate, pwindow)
                        )


@main.main.command()
@click.option('--ik-method', type=click.Choice(['numerical', 'analytical']),
              help='IK method', default='analytical')
@click.pass_context
def evaluate_speed(ctx, ik_method):
    model = ctx.obj['model']
    chest = ctx.obj['chest']
    lsole = ctx.obj['lsole']
    rsole = ctx.obj['rsole']
    timesteps = ctx.obj['timesteps']
    zmp_ref = ctx.obj['zmp_ref']

    speed = OrderedDict()

    q_ini = model.initial_pose_walking

    # First ZMP calculation
    with status('Calculate ZMP from forward run'):
        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate='savgol', method=ik_method
        )

        com_calc = kinematics.com_trajectory(model, chest, q_calc)
        zmp_calc = zmp.calculate_zmp_trajectory(model, q_calc, qdot_calc, qddot_calc, chest)

    for filter_method, iterations, interpolations, previewwindows in CONFIGS:
        for interpolate in interpolations:
            for previewwindow in previewwindows:

                speeds = []
                confstr = gen_confstr(filter_method, 1, interpolate, previewwindow)
                for i in range(SPEED_RUNS):
                    with status('{} - Apply dynamic filter - Run {}'.format(confstr, i)):
                        chest_next = chest.copy()
                        t0 = time.time()
                        chest_next = filter.filters[filter_method](
                            chest=chest_next, lsole=lsole, rsole=rsole, zmp_ref=zmp_ref, q_ini=q_ini,
                            model=model, times=timesteps, ik_method=ik_method, previewwindow=previewwindow
                        )
                        if interpolate == 'savgol':
                            interpolate_savgol(chest_next.traj_pos)
                        update_derivs(chest_next, timesteps)
                        speeds.append(time.time() - t0)

                speed[confstr] = np.array(speeds)

    with status('Generate boxplots'):
        plot.plot_speed(
            OrderedDict(reversed(speed.items())),
            filenames=[
                os.path.join(ctx.obj['out_dir'], 'speed_comparison.pdf'),
                os.path.join(ctx.obj['out_dir'], 'speed_comparison.pgf'),
            ]
        )
