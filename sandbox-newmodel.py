import numpy as np
import rbdl
import os

from dynfil.bodies import BodyTrajectory
from dynfil.constants import POSE_HALF_SITTING
from dynfil.utils.meshup import save_to_meshup

filename = "./data/models/iCubHeidelberg01_new_legs.urdf"
model = rbdl.loadModel(filename)

pgdata = np.genfromtxt('data/pgdata4.txt', delimiter=' ', dtype=None)
timesteps = np.array([0])

offset_angles = np.array([np.pi/2., 0.0, -np.pi/2.])
chest = BodyTrajectory(model, model.GetBodyId("chest"))
chest.set_trajectories(pgdata[3:4, 1:4], pgdata[3:4, 4:7], offset_angles)

lsole = BodyTrajectory(model, model.GetBodyId("l_sole"))
rsole = BodyTrajectory(model, model.GetBodyId("r_sole"))

offset_angles = np.array([0, 0., np.pi])
rsole.set_trajectories(pgdata[3:4, 7:10], pgdata[3:4, 10:13], offset_angles)
lsole.set_trajectories(pgdata[3:4, 13:16], pgdata[3:4, 16:19], offset_angles)

cs = rbdl.InverseKinematicsConstraintSet()
cs.lmbda = 1e-4

cs.AddFullConstraint(*chest.to_constraint(0))
cs.AddFullConstraint(*lsole.to_constraint(0))
cs.AddFullConstraint(*rsole.to_constraint(0))

q = rbdl.InverseKinematics(model, POSE_HALF_SITTING, cs)

save_to_meshup(os.path.join('out', 'sandbox.csv'), timesteps, np.array([q]))
