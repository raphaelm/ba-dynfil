import click
import numpy as np
import rbdl

from dynfil.bodies import BodyTrajectory
from dynfil.utils.cli import status


@click.group()
@click.option('--model', default='data/models/iCubHeidelberg01_no_weights.urdf', help='Model file')
@click.option('--trajectory', required=True, help='Trajectory file')
@click.option('--csv-delim', default=',', help='CSV delimiter of trajectory file')
@click.option('--out-dir', default='out/', help='Output directory')
@click.option('--show', is_flag=True, help='Open plot windows')
@click.pass_context
def main(ctx, model, trajectory, out_dir, show, csv_delim):
    click.echo(click.style('Model:            {}'.format(model), fg='blue'))
    click.echo(click.style('Trajectory:       {}'.format(trajectory), fg='blue'))
    click.echo(click.style('Output directory: {}'.format(out_dir), fg='blue'))
    click.echo()

    model = rbdl.loadModel(model)

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

    ctx.obj['timesteps'] = timesteps
    ctx.obj['chest'] = chest
    ctx.obj['lsole'] = lsole
    ctx.obj['rsole'] = rsole
    ctx.obj['zmp_ref'] = zmp_ref
    ctx.obj['model'] = model
    ctx.obj['show'] = show
    ctx.obj['out_dir'] = out_dir
    click.echo()
