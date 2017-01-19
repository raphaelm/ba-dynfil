import click
import numpy as np
import os
import rbdl
import sys
import warnings

from dynfil import constants, zmp, kinematics, filter
from dynfil.bodies import BodyTrajectory
from dynfil.utils import plot
from dynfil.utils.meshup import save_to_meshup
from dynfil.utils.cli import status

warnings.simplefilter("ignore", category=np.RankWarning)


@click.command()
@click.option('--model', default='data/models/iCubHeidelberg01_no_weights.urdf', help='Model file')
@click.option('--trajectory', default='data/traj/alpha0015beta02/PatternGeneratorData.csv', help='Trajectory file')
@click.option('--csv-delim', default=',', help='CSV delimiter of trajectory file')
@click.option('--out-dir', default='out/', help='Output directory')
@click.option('--show', is_flag=True, help='Open plot windows')
@click.option('--interpolate/--no-interpolate', default=True, help='Apply interpolation')
@click.option('--filter-method', type=click.Choice(['newton', 'leastsquares', 'steepestdescent']),
              help='Filter method', default='newton')
@click.option('--iterations', type=click.IntRange(1, 100), default=5, help='Number of filter iterations')
def main(model, trajectory, out_dir, show, filter_method, iterations, csv_delim, interpolate):
    click.echo(click.style('Model:            {}'.format(model), fg='blue'))
    click.echo(click.style('Trajectory:       {}'.format(trajectory), fg='blue'))
    click.echo(click.style('Filter method:    {}'.format(filter_method), fg='blue'))
    click.echo(click.style('Iterations:       {}'.format(iterations), fg='blue'))
    click.echo(click.style('Output directory: {}'.format(out_dir), fg='blue'))
    click.echo(click.style('Interpolation:    {}'.format(interpolate), fg='blue'))
    click.echo()

    model = rbdl.loadModel(model)

    # initial pose: half-sitting
    q_ini = constants.POSE_HALF_SITTING

    # Load data from file
    with status('Loading data'):
        pgdata = np.genfromtxt(trajectory, delimiter=csv_delim, dtype=None)
        timesteps = pgdata[:, 0]

        offset_angles = np.array([np.pi/2., 0.0, np.pi/2.])
        chest = BodyTrajectory(model, model.GetBodyId("chest"))
        chest.set_trajectories(pgdata[:, 1:4], pgdata[:, 4:7], offset_angles)

        lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
        lsole.set_trajectories(pgdata[:, 7:10], pgdata[:, 10:13])

        rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))
        rsole.set_trajectories(pgdata[:, 13:16], pgdata[:, 16:19])

        zmp_ref = pgdata[:, 19:22]

    # Choose IK method
    ik = kinematics.inverse_with_derivatives

    # First ZMP calculation
    with status('Calculate ZMP from forward run'):
        q_calc_raw, qdot_calc_raw, qddot_calc_raw = ik(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate=False
        )
        q_calc_int, qdot_calc_int, qddot_calc_int = ik(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate=True
        )
        if interpolate:
            q_calc, qdot_calc, qddot_calc = q_calc_int, qdot_calc_int, qddot_calc_int
        else:
            q_calc, qdot_calc, qddot_calc = q_calc_raw, qdot_calc_raw, qddot_calc_raw

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
        filters = {
            'least_squares': filter.dynfil_least_squares,
            'newton': filter.dynfil_newton_numerical,
            'steepestdescent': filter.dynfil_gradient_descent,
        }
        chest_filtered = filters[filter_method](
            chest=chest, lsole=lsole, rsole=rsole, zmp_ref=zmp_ref, q_ini=q_ini,
            model=model, times=timesteps, iterations=iterations, ik=ik, status_update=status_update
        )

    # Calculate ZMP from filtered result
    with status('Calculate ZMP from filtered data'):
        q_filtered, qdot_filtered, qddot_filtered = ik(
            model, q_ini, chest_filtered, lsole, rsole, timesteps
        )
        zmp_filtered = zmp.calculate_zmp_trajectory(model, q_filtered, qdot_filtered, qddot_filtered, chest_filtered)

    # Save meshup files
    with status('Export MeshUp files'):
        save_to_meshup(os.path.join(out_dir, 'inverse_from_pg.csv'), timesteps, q_calc)
        save_to_meshup(os.path.join(out_dir, 'inverse_after_filter.csv'), timesteps, q_filtered)

    # Generate plots
    with status('Generate plots'):
        plot.plot_q_interpolation(timesteps, qdot_calc_raw, qdot_calc_int, name='qdot',
                                  limit=5, filename=os.path.join(out_dir, 'test_interpol.pdf'),
                                  title='Interpolation results')
        plot.plot_q_interpolation(timesteps, qddot_calc_raw, qddot_calc_int, name='qddot',
                                  limit=5, filename=os.path.join(out_dir, 'test_interpol.pdf'),
                                  title='Interpolation results')
        plot.plot_q_values(
            timesteps,
            (q_calc, q_filtered),
            (qdot_calc, qddot_filtered),
            (qddot_calc, qddot_filtered),
            limit=5,
            filename=os.path.join(out_dir, 'q_calc.pdf'),
            title='Filter results on trajectories'
        )

        plot.plot_trajectories(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=chest.traj_ort, label='PG: CoM', color='y'),
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot', color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot', color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
            ],
            filename=os.path.join(out_dir, 'trajectories.pdf'),
            title='3D Trajectories (reference and forward run)'
        )

        plot.plot_trajectories(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=chest.traj_ort, label='PG: CoM', color='y'),
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot', color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot', color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
                plot.PlotTrajectory(positions=chest_filtered.traj_pos, rotations=None, label='Dynfil: CoM', color='b'),
                plot.PlotTrajectory(positions=zmp_filtered, rotations=None, label='Dynfil: ZMP', color='k'),
            ],
            filename=os.path.join(out_dir, 'trajectories_with_filtered.pdf'),
            title='3D Trajectories (with filtered)'
        )

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot', color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot', color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
            ],
            filename=os.path.join(out_dir, 'trajectories_on_ground.pdf'),
            title='2D Trajectories on the ground (reference and forward run)'
        )

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=lsole.traj_ort, label='PG: CoM', color='r'),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
            ],
            filename=os.path.join(out_dir, 'trajectories_waist.pdf'),
            title='2D Trajectories on waist height ground (reference and forward run)'
        )

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot', color='r'),
                plot.PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot', color='g'),
                plot.FootTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, color='r'),
                plot.FootTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, color='g'),
                plot.PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
                plot.PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
                plot.PlotTrajectory(positions=zmp_filtered, rotations=None, label='Dynfil: ZMP', color='k'),
            ],
            filename=os.path.join(out_dir, 'trajectories_on_ground_with_filtered.pdf'),
            title='2D Trajectories on the ground (with filteredrun)'
        )

        plot.plot_trajectories_from_top(
            trajectories=[
                plot.PlotTrajectory(positions=chest.traj_pos, rotations=lsole.traj_ort, label='PG: CoM', color='r'),
                plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
                plot.PlotTrajectory(positions=chest_filtered.traj_pos, rotations=None, label='Dynfil: CoM', color='k'),
            ],
            filename=os.path.join(out_dir, 'trajectories_waist_with_filtered.pdf'),
            title='2D Trajectories on waist height ground (with filtered)'
        )

        plot.plot_residuums(
            data=[
                plot.PlotResiduum(times=timesteps, values=zmp_calc - zmp_ref,
                                  label=r'$\left\|\mathbf{r_{ZMP}} - \mathbf{r_{ZMP}}^{ref}\right\|$ without filter',
                                  color='b'),
                plot.PlotResiduum(times=timesteps, values=zmp_filtered - zmp_ref,
                                  label=r'$\left\|\mathbf{r_{ZMP}} - \mathbf{r_{ZMP}}^{ref}\right\|$ with filter',
                                  color='g')
            ],
            filename=os.path.join(out_dir, 'residuums.pdf'),
            title='Filter result summary'
        )

    if show:
        plot.show_all()

if __name__ == '__main__':
    main()
