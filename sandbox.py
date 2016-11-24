import numpy as np

import rbdl
import pyrbdl_model

# load trajectory CoM, foot_l, foot_r
load_d = {
    "delimiter": ", ",
}
trajectory = np.loadtxt(".", **load_d)

# load model
# python wrapper
# path to lua file
# RBDL and RBDL Wrapper

# hard-coded model path for example
model_path = "leo_dl.lua"
model = pyrbdl_model.RBDLModel()
model.loadModelFromFile (model_path, True)
model.loadPointsFromFile (model_path, True)

qs = []

for i in range(trajectory.shape[0]):
    CoM = trajectory[i, 1:4]
    foot_l = trajectory[i, 1:4]
    foot_r = trajectory[i, 1:4]


    q_i = rbdl.IK(model, [CoM, foot_l, foot_r])
    qs.append(q_i)


# as  MeshUp compatible csv
np.savetxt()
