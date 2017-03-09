import click
import numpy as np
import rbdl
import os
import warnings

from dynfil.bodies import BodyTrajectory
from dynfil.kinematics.numerical import IKConvergenceWarning
from dynfil.models import HeiCubModel
from dynfil.models import SimpleModel
from dynfil.utils.cli import status


@click.group()
@click.option('--model', default='simple', help='Model', type=click.Choice(['simple', 'heicub']))
@click.option('--trajectory', required=True, help='Trajectory file')
@click.option('--csv-delim', default=' ', help='CSV delimiter of trajectory file')
@click.option('--out-dir', default='out/', help='Output directory')
@click.option('--show', is_flag=True, help='Open plot windows')
@click.option('-w/--show-warnings', is_flag=True, help='Show warnings')
@click.pass_context
def main(ctx, model, trajectory, out_dir, show, csv_delim, w):
    click.echo(click.style('Model:            {}'.format(model), fg='blue'))
    click.echo(click.style('Trajectory:       {}'.format(trajectory), fg='blue'))
    click.echo(click.style('Output directory: {}'.format(out_dir), fg='blue'))
    click.echo()

    if not os.path.exists(out_dir):
        os.mkdir(out_dir)

    if model == "heicub":
        model = HeiCubModel()
    elif model == "simple":
        model = SimpleModel()

    if not w:
        warnings.simplefilter("ignore", IKConvergenceWarning)
    else:
        warnings.simplefilter("once", IKConvergenceWarning)

    # Load data from file
    with status('Loading data'):
        pgdata = np.genfromtxt(trajectory, delimiter=csv_delim, dtype=None)
        timesteps = pgdata[:, 0]

        chest = model.get_body(model.chest_body_id)
        lsole = model.get_body(model.lfoot_body_id)
        rsole = model.get_body(model.rfoot_body_id)

        chest.set_trajectories(pgdata[:, 1:4], pgdata[:, 4:7], offset_angles=model.chest_offset_angles)
        chest.traj_pos_dot = pgdata[:, 28:31]
        chest.traj_pos_ddot = pgdata[:, 37:40]

        lsole.set_trajectories(pgdata[:, 7:10], pgdata[:, 10:13])
        lsole.traj_pos_dot = pgdata[:, 22:25]
        lsole.traj_pos_ddot = pgdata[:, 31:34]

        rsole.set_trajectories(pgdata[:, 13:16], pgdata[:, 16:19])
        rsole.traj_pos_dot = pgdata[:, 25:28]
        rsole.traj_pos_ddot = pgdata[:, 34:37]

        zmp_ref = pgdata[:, 19:22]

    ctx.obj['timesteps'] = timesteps
    ctx.obj['chest'] = chest
    ctx.obj['lsole'] = lsole
    ctx.obj['rsole'] = rsole
    ctx.obj['zmp_ref'] = zmp_ref
    ctx.obj['model'] = model
    ctx.obj['show'] = show
    ctx.obj['out_dir'] = out_dir
    click.echo()
