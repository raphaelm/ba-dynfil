import numpy as np
import rbdl
import sys
import warnings

from dynfil import constants, zmp, kinematics, filter
from dynfil.bodies import BodyTrajectory
from dynfil.utils import plot
from dynfil.utils.meshup import save_to_meshup

warnings.simplefilter("once", category=np.RankWarning)

model = rbdl.loadModel('data/models/iCubHeidelberg01_no_weights.urdf')

# initial pose: half-sitting
q_ini = constants.POSE_HALF_SITTING

# Load data from file
pgdata = np.genfromtxt('data/traj/alpha0015beta02/PatternGeneratorData.csv', delimiter=',', dtype=None)
timesteps = pgdata[:, 0]

offset_angles = np.array([np.pi/2., 0.0, np.pi/2.])
chest = BodyTrajectory(model, model.GetBodyId("chest"))
chest.set_trajectories(pgdata[:, 1:4], pgdata[:, 4:7], offset_angles)

lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
lsole.set_trajectories(pgdata[:, 7:10], pgdata[:, 10:13])

rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))
rsole.set_trajectories(pgdata[:, 13:16], pgdata[:, 16:19])

zmp_ref = pgdata[:, 19:22]

# First ZMP calculation
q_calc_raw, qdot_calc_raw, qddot_calc_raw = kinematics.inverse_with_derivatives(
    model, q_ini, chest, lsole, rsole, timesteps, interpolate=False
)
q_calc, qdot_calc, qddot_calc = kinematics.inverse_with_derivatives(
    model, q_ini, chest, lsole, rsole, timesteps, interpolate=True
)
com_calc = kinematics.com_trajectory(model, chest, q_calc)
zmp_calc = zmp.calculate_zmp_trajectory(model, q_calc, qdot_calc, qddot_calc, chest)

# Apply dynamic filter
chest_filtered = filter.dynfil_least_squares(
    chest=chest, lsole=lsole, rsole=rsole, zmp_ref=zmp_ref, q_ini=q_ini,
    model=model, times=timesteps, iterations=1
)

# Calculate ZMP from filtered result
q_filtered, qdot_filtered, qddot_filtered = kinematics.inverse_with_derivatives(
    model, q_ini, chest_filtered, lsole, rsole, timesteps
)
zmp_filtered = zmp.calculate_zmp_trajectory(model, q_filtered, qdot_filtered, qddot_filtered, chest_filtered)

# Save meshup files
save_to_meshup('out/inverse_from_pg.csv', timesteps, q_calc)
save_to_meshup('out/inverse_after_filter.csv', timesteps, q_filtered)

# Generate plots
show_plots = '--show' in sys.argv
plot.plot_q_interpolation(timesteps, qdot_calc_raw, qdot_calc, name='qdot',
                          limit=5, filename='out/test_interpol.png', show=show_plots)
plot.plot_q_interpolation(timesteps, qddot_calc_raw, qddot_calc, name='qddot',
                          limit=5, filename='out/test_interpol.png', show=show_plots)
plot.plot_q_values(
    timesteps,
    (q_calc, q_filtered),
    (qdot_calc, qddot_filtered),
    (qddot_calc, qddot_filtered),
    limit=5,
    filename='out/q_calc.png',
    show=show_plots,
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
    filename='out/trajectories.png',
    show=show_plots
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
    filename='out/trajectories_on_ground.png',
    show=show_plots
)

plot.plot_trajectories_from_top(
    trajectories=[
        plot.PlotTrajectory(positions=chest.traj_pos, rotations=lsole.traj_ort, label='PG: CoM', color='r'),
        plot.PlotTrajectory(positions=com_calc, rotations=None, label='CoM from forward run', color='c'),
        plot.PlotTrajectory(positions=chest_filtered.traj_pos, rotations=None, label='Dynfil: CoM', color='k'),
    ],
    filename='out/trajectories_waist.png',
    show=show_plots
)
