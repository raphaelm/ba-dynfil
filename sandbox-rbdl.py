import sys
import rbdl
import numpy as np


# ------------------------------------------------------------------------------
# header for MeshUp animation file
meshup_header = """
COLUMNS:
  time,
  root_link:T:X,
  root_link:T:Y,
  root_link:T:Z,
  root_link:R:X:rad,
  root_link:R:Y:rad,
  root_link:R:Z:rad,
  l_hip_1:R:X:rad,
  l_hip_2:R:-Z:rad,
  l_upper_leg:R:Y:rad,
  l_lower_leg:R:X:rad,
  l_ankle_1:R:-X:rad,
  l_ankle_2:R:-Z:rad,
  r_hip_1:R:-X:rad,
  r_hip_2:R:-Z:rad,
  r_upper_leg:R:-Y:rad,
  r_lower_leg:R:-X:rad,
  r_ankle_1:R:X:rad,
  r_ankle_2:R:-Z:rad
  torso_1:R:X:rad,
  torso_2:R:-Z:rad,
  chest:R:-Y:rad
  DATA:
"""

animation_d = {
    "fmt": "%.18f",
    "delimiter": ", ",
    "newline": "\n",
    "comments": "",
    "header": meshup_header,
}


# ------------------------------------------------------------------------------
def rotx(rad):
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array([
        [1., 0., 0.],
        [0., c, s],
        [0., -s, c]
    ])


def roty(rad):
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array([
        [c, 0., -s],
        [0., 1., 0.],
        [s, 0., c]
    ])

def rotz(rad):
    c = np.cos(rad)
    s = np.sin(rad)
    return np.array([
        [c, s, 0.],
        [-s, c, 0.],
        [0., 0., 1.]
    ])


def get_matrix_from_EulerXYZ (angles, order):
    res = np.zeros([3, 3])
    if order == "123":
        R1 = rotx(angles[0])
        R2 = roty(angles[1])
        R3 = rotz(angles[2])
        res[...] = R1.dot(R2).dot(R3)
    elif order == "321":
        R1 = rotx(angles[2])
        R2 = roty(angles[1])
        R3 = rotz(angles[0])
        res[...] = R3.dot(R2).dot(R1)
    else:
        raise NotImplementedError()

    return res


def get_Euler_from_matrix(R, order):
    angles = np.zeros([3,])
    if order == "123":
        angles[0] = np.arctan2(R[1, 2], R[2, 2])
        angles[1] = -np.arcsin(R[0,2])
        angles[2] = np.arctan2(R[0,1], R[0,0])
    elif order == "321":
        angles[0] = np.arctan2(-R[1, 0], R[0, 0])
        angles[1] = np.arcsin(R[2, 0])
        angles[2] = np.arctan2(-R[2, 1],R[2, 2])
    else:
        raise NotImplementedError()

    return None


# ------------------------------------------------------------------------------
if __name__ == "__main__":
    # load lua model
    # filename = "./python/leo_dl.lua"
    # filename = "./python/iCubHeidelberg01.lua"
    filename = "./iCubHeidelberg01_no_weights.urdf";

    # NOTE loadModel loads lua and urdf models
    print "loading model: ", filename
    model = rbdl.loadModel(filename)
    print "model successfully loaded!"

    # initial pose: half-sitting
    q_ini = np.array([
        0.063, 0, 0.605, # x, y, z
        0, 0.25, np.pi, # Euler x, y, z
        0.5, 0, # l_hip x, -z
        0, # l_up y
        -0.5, # l_lo x
        -0.25, 0, # l_ak -x, -z
        0.5, 0, # r_hip x, -z
        0, # r_up y
        -0.5, # r_lo x
        -0.25, 0, # r_ak -x, -z
        -0.25, 0, # torso x, -z
        0 # chest -y
    ])

    # define some constants
    chest_body_id = model.GetBodyId("chest")
    chest_body_point = np.array([0.0, 0.0, 0.0])
    chest_tpos = np.array([0.0685, 0.0, 0.688])
    chest_angles = np.array([np.pi/2., 0.0, np.pi/2.])
    chest_tori = get_matrix_from_EulerXYZ(chest_angles, "123")

    lsole_body_id = model.GetBodyId("l_sole")
    lsole_body_point = np.array([0.0, 0.0, 0.0])
    lsole_tpos = np.array([0.0, 0.0739, 0.0])
    lsole_angles = np.array([0.0, 0.0, 0.0])
    lsole_tori = get_matrix_from_EulerXYZ(lsole_angles, "123")

    rsole_body_id = model.GetBodyId("r_sole")
    rsole_body_point = np.array([0.0, 0.0, 0.0])
    rsole_tpos = np.array([0.0, -0.0739, 0.0])
    rsole_angles = np.array([0.0, 0.0, 0.0])
    rsole_tori = get_matrix_from_EulerXYZ(rsole_angles, "123")

    # show points before optimization
    chest_pos = rbdl.CalcBodyToBaseCoordinates (
        model, q_ini, chest_body_id, chest_body_point
    )
    chest_ori = rbdl.CalcBodyWorldOrientation (model, q_ini, chest_body_id)
    print "chest_id:   ", chest_body_id
    print "chest_pos:  ", chest_pos
    print "chest_tpos: ", chest_tpos
    print "error:      ", np.sqrt((chest_pos - chest_tpos)**2)
    print "chest_ori: \n", chest_ori
    print "chest_tori:\n", chest_tori
    print "error:     \n", np.sqrt((chest_ori - chest_tori)**2)
    print ""

    lsole_pos = rbdl.CalcBodyToBaseCoordinates (
        model, q_ini, lsole_body_id, lsole_body_point
    )
    lsole_ori = rbdl.CalcBodyWorldOrientation (model, q_ini, lsole_body_id)
    print "lsole_id:   ", lsole_body_id
    print "lsole_pos:  ", lsole_pos
    print "lsole_tpos: ", lsole_tpos
    print "error:      ", np.sqrt((lsole_pos - lsole_tpos)**2)
    print "lsole_ori: \n", lsole_ori
    print "lsole_tori:\n", lsole_tori
    print "error:     \n", np.sqrt((lsole_ori - lsole_tori)**2)
    print ""

    rsole_pos = rbdl.CalcBodyToBaseCoordinates (
        model, q_ini, rsole_body_id, rsole_body_point
    )
    rsole_ori = rbdl.CalcBodyWorldOrientation (model, q_ini, rsole_body_id)
    print "rsole_id:   ", rsole_body_id
    print "rsole_pos:  ", rsole_pos
    print "rsole_tpos: ", rsole_tpos
    print "error:      ", np.sqrt((rsole_pos - rsole_tpos)**2)
    print "rsole_ori: \n", rsole_ori
    print "rsole_tori:\n", rsole_tori
    print "error:     \n", np.sqrt((rsole_ori - rsole_tori)**2)
    print ""

    # define IK constraint set
    cs = rbdl.InverseKinematicsConstraintSet()
    cs.lmbda = 1e-4

    # add point position constraint
    #   min_q || p(q) - p_d ||_2
    # com = cs.AddPointConstraint(chest_body_id, chest_body_point, chest_tpos)
    # ls = cs.AddPointConstraint(lsole_body_id, lsole_body_point, lsole_tpos)
    # rs = cs.AddPointConstraint(rsole_body_id, rsole_body_point, rsole_tpos)

    # com = cs.AddOrientationConstraint(chest_body_id, chest_tori)
    # ls = cs.AddOrientationConstraint(lsole_body_id, lsole_tori)
    # rs = cs.AddOrientationConstraint(rsole_body_id, rsole_tori)

    com = cs.AddFullConstraint(
        chest_body_id, chest_body_point, chest_tpos, chest_tori
    )
    ls = cs.AddFullConstraint(
        lsole_body_id, lsole_body_point, lsole_tpos, lsole_tori
    )
    rs = cs.AddFullConstraint(
        rsole_body_id, rsole_body_point, rsole_tpos, rsole_tori
    )

    # calculate inverse kinematics
    q_res = rbdl.InverseKinematics(model, q_ini, cs)

    # test if points reached target positions
    chest_pos = rbdl.CalcBodyToBaseCoordinates (
        model, q_ini, chest_body_id, chest_body_point
    )
    chest_ori = rbdl.CalcBodyWorldOrientation (model, q_res, chest_body_id)
    print "chest_id:   ", chest_body_id
    print "chest_pos:  ", chest_pos
    print "chest_tpos: ", chest_tpos
    print "error:      ", np.sqrt((chest_pos - chest_tpos)**2)
    print "chest_ori: \n", chest_ori
    print "chest_tori:\n", chest_tori
    print "error:     \n", np.sqrt((chest_ori - chest_tori)**2)
    print ""

    lsole_pos = rbdl.CalcBodyToBaseCoordinates (
        model, q_res, lsole_body_id, lsole_body_point
    )
    lsole_ori = rbdl.CalcBodyWorldOrientation (model, q_res, lsole_body_id)
    print "lsole_id:   ", lsole_body_id
    print "lsole_pos:  ", lsole_pos
    print "lsole_tpos: ", lsole_tpos
    print "error:      ", np.sqrt((lsole_pos - lsole_tpos)**2)
    print "lsole_ori: \n", lsole_ori
    print "lsole_tori:\n", lsole_tori
    print "error:     \n", np.sqrt((lsole_ori - lsole_tori)**2)
    print ""

    rsole_pos = rbdl.CalcBodyToBaseCoordinates (
        model, q_res, rsole_body_id, rsole_body_point
    )
    rsole_ori = rbdl.CalcBodyWorldOrientation (model, q_res, rsole_body_id)
    print "rsole_id:   ", rsole_body_id
    print "rsole_pos:  ", rsole_pos
    print "rsole_tpos: ", rsole_tpos
    print "error:      ", np.sqrt((rsole_pos - rsole_tpos)**2)
    print "rsole_ori: \n", rsole_ori
    print "rsole_tori:\n", rsole_tori
    print "error:     \n", np.sqrt((rsole_ori - rsole_tori)**2)
    print ""

    # save MeshUp animation file
    meshup_filepath = "animation.csv"
    rows = 2
    ts = np.linspace(0.0, 1.0, rows, endpoint=True)
    meshup_data = np.zeros([2, 1+q_res.size])
    meshup_data[:, 0] = ts
    meshup_data[0, 1:] = q_ini
    meshup_data[1, 1:] = q_res
    np.savetxt(meshup_filepath, meshup_data, **animation_d)


# ------------------------------------------------------------------------------
