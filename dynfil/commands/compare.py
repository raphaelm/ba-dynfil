import os

import click

from dynfil import constants, kinematics
from dynfil.commands import main
from dynfil.utils import plot
from dynfil.utils.cli import status


@main.main.command()
@click.pass_context
@click.option('--ik-method', type=click.Choice(['numerical', 'analytical']),
              help='IK method', default='numerical')
def compare_interpolation(ctx, ik_method):
    click.echo(click.style('IK method:        {}'.format(ik_method), fg='blue'))
    click.echo()

    model = ctx.obj['model']
    chest = ctx.obj['chest']
    lsole = ctx.obj['lsole']
    rsole = ctx.obj['rsole']
    timesteps = ctx.obj['timesteps']

    # initial pose: half-sitting
    q_ini = constants.POSE_HALF_SITTING_HEICUB

    with status('Inverse Kinematics without interpolation'):
        q_calc_raw, qdot_calc_raw, qddot_calc_raw = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate='none', method=ik_method
        )

    with status('Inverse Kinematics with poly interpolation'):
        q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate='poly', method=ik_method
        )

    with status('Inverse Kinematics with savgol interpolation'):
        q_calc_sg, qdot_calc_sg, qddot_calc_sg = kinematics.inverse_with_derivatives(
            model, q_ini, chest, lsole, rsole, timesteps, interpolate='savgol', method=ik_method
        )

    # Generate plots
    with status('Generate plots'):
        plot.plot_q_interpolation(
            timesteps, q_calc_raw, q_calc, name='qdot',
            limit=5, filename=os.path.join(ctx.obj['out_dir'], 'compare_interpol_poly_q.pdf'),
            title='Interpolation results q (polynoms)'
        )
        plot.plot_q_interpolation(
            timesteps, qdot_calc_raw, qdot_calc, name='qdot',
            limit=5, filename=os.path.join(ctx.obj['out_dir'], 'compare_interpol_poly_qdot.pdf'),
            title='Interpolation results qdot (polynoms)'
        )
        plot.plot_q_interpolation(
            timesteps, qddot_calc_raw, qddot_calc, name='qddot',
            limit=5, filename=os.path.join(ctx.obj['out_dir'], 'compare_interpol_poly_qddot.pdf'),
            title='Interpolation results qddot (polynoms)'
        )
        plot.plot_q_interpolation(
            timesteps, qddot_calc_raw, qddot_calc_sg, name='qddot',
            limit=5, filename=os.path.join(ctx.obj['out_dir'], 'compare_interpol_savgol_qddot.pdf'),
            title='Interpolation results qddot (savgol)'
        )

    if ctx.obj['show']:
        plot.show_all()


@main.main.command()
@click.pass_context
def compare_ik(ctx):
    model = ctx.obj['model']
    chest = ctx.obj['chest']
    lsole = ctx.obj['lsole']
    rsole = ctx.obj['rsole']
    timesteps = ctx.obj['timesteps']

    # initial pose: half-sitting
    q_ini = constants.POSE_HALF_SITTING_HEICUB

    with status('Run numerical IK'):
        q_calc = kinematics.inverse(
            model, q_ini, chest, lsole, rsole, method='numerical'
        )

    with status('Run analytical IK'):
        q_calc_a = kinematics.inverse(
            model, q_ini, chest, lsole, rsole, method='analytical'
        )

    # Generate plots
    with status('Generate plots'):
        plot.plot_q_values(
            timesteps, (q_calc, q_calc_a),
            labels=('numerical', 'analytical'),
            filename=os.path.join(ctx.obj['out_dir'], 'compare_kinematics.pdf'),
            title='Interpolation results q'
        )

    if ctx.obj['show']:
        plot.show_all()
