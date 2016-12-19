import numpy as np
import rbdl

from dynfil import constants, zmp, kinematics
from dynfil.bodies import BodyTrajectory
from dynfil.utils.plot import plot_trajectories, PlotTrajectory

model = rbdl.loadModel('data/models/iCubHeidelberg01_no_weights.urdf')

# initial pose: half-sitting
q_ini = constants.POSE_HALF_SITTING

pgdata = np.genfromtxt('data/traj/alpha0015beta02/PatternGeneratorData.csv', delimiter=',', dtype=None)

chest = BodyTrajectory(model, model.GetBodyId("chest"))
chest.set_trajectories(pgdata[:, 1:4], pgdata[:, 4:7])

lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
lsole.set_trajectories(pgdata[:, 7:10], pgdata[:, 10:13])

rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))
rsole.set_trajectories(pgdata[:, 13:16], pgdata[:, 16:19])

zmp_ref = pgdata[:, 19:22]

q_calc = kinematics.inverse(model, q_ini, chest, lsole, rsole)
zmp_calc = zmp.calculate_zmp_trajectory(model, q_calc, times=pgdata[:, 0])

plot_trajectories(
    trajectories=[
        PlotTrajectory(positions=chest.traj_pos, rotations=chest.traj_ort, label='PG: CoM', color='y'),
        PlotTrajectory(positions=lsole.traj_pos, rotations=lsole.traj_ort, label='PG: left foot', color='r'),
        PlotTrajectory(positions=rsole.traj_pos, rotations=rsole.traj_ort, label='PG: right foot', color='g'),
        PlotTrajectory(positions=zmp_ref, rotations=None, label='ZMP reference', color='m'),
        PlotTrajectory(positions=zmp_calc, rotations=None, label='ZMP from forward run', color='c'),
    ],
    filename='out/trajectories.png',
    show=True
)

