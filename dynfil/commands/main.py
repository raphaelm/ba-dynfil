import click
import numpy as np
import rbdl
import warnings

from dynfil.bodies import BodyTrajectory
from dynfil.kinematics import IKConvergenceWarning
from dynfil.utils.cli import status


@click.group()
@click.option('--model', default='data/models/iCubHeidelberg01_new_legs.urdf', help='Model file')
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

    model = rbdl.loadModel(model)

    if not w:
        warnings.simplefilter("ignore", IKConvergenceWarning)
    else:
        warnings.simplefilter("once", IKConvergenceWarning)

    # Load data from file
    with status('Loading data'):
        pgdata = np.genfromtxt(trajectory, delimiter=csv_delim, dtype=None)
        timesteps = pgdata[:, 0]

        chest = BodyTrajectory(model, model.GetBodyId("chest"))

        lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
        rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))

        offset_angles = np.array([np.pi/2., 0.0, np.pi/2.])
        chest.set_trajectories(pgdata[:, 1:4], pgdata[:, 4:7], offset_angles)
        offset_angles = np.array([0., 0.0, 0])
        lsole.set_trajectories(pgdata[:, 7:10], pgdata[:, 10:13], offset_angles)
        rsole.set_trajectories(pgdata[:, 13:16], pgdata[:, 16:19], offset_angles)

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
