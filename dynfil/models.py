import numpy as np
import rbdl

from dynfil.bodies import BodyTrajectory
from dynfil.utils import meshup


@property
def _abstract_property(self):
    raise NotImplementedError()


class RobotModel(object):
    chest_offset_angles = np.array([0., 0., 0.])
    model_file = _abstract_property
    chest_body_id = _abstract_property
    lfoot_body_id = _abstract_property
    rfoot_body_id = _abstract_property
    lankle_body_id = _abstract_property
    rankle_body_id = _abstract_property
    lknee_body_id = _abstract_property
    rknee_body_id = _abstract_property
    lhip_body_id = _abstract_property
    rhip_body_id = _abstract_property
    initial_pose_walking = _abstract_property
    meshup_header = _abstract_property
    step_length = _abstract_property
    foot_distance = _abstract_property
    com_height = _abstract_property

    def __init__(self):
        self.model = rbdl.loadModel(self.model_file)

    def get_body(self, body_id):
        return BodyTrajectory(self.model, body_id)

    def leg_vector_from_simple(self, lq, rq):
        return np.hstack((rq.T, lq.T)).T

    @property
    def qdot_size(self):
        return self.model.qdot_size

    @property
    def dof_count(self):
        return self.model.dof_count


class HeiCubModel(RobotModel):
    model_file = "data/models/iCubHeidelberg01_new_legs.urdf"
    chest_body_id = "root_link"
    lfoot_body_id = "l_sole"
    rfoot_body_id = "r_sole"
    lankle_body_id = "l_ankle_1"
    rankle_body_id = "r_ankle_1"
    lknee_body_id = "l_lower_leg"
    rknee_body_id = "r_lower_leg"
    lhip_body_id = "l_hip_1"
    rhip_body_id = "r_hip_1"
    initial_pose_walking = np.array([
        -0.001949885904078843, 0.009946050324140285, 0.584596046738965480,
        -0.008293313280757705, 0.231216009268220279, 0.002275791030702485,
        0.664116025045018321, -0.003012971387495609,
        -0.001047937397426056,
        -0.745127746017596793,
        -0.312224186746417720, 0.010067205602002581,
        0.655670717078791543, 0.025015005687746280,
        -0.008873325580543002,
        -0.728701070878931323,
        -0.304328633067010523, -0.034221059397175228,
        -0.231233803533270527, -0.007771770032605045,
        0.002215295506378073
    ])
    meshup_header = meshup.MESHUP_HEADER_HEICUB
    step_length = 0.2162 * 0.5
    foot_distance = 0.075 * 2
    com_height = 0.40

    def leg_vector_from_simple(self, lq, rq):
        return np.array([
            lq[2],  # l_hip_1 = q4
            - lq[1],  # l_hip_2 = q3
            lq[0],  # l_upper_leg = q0
            - lq[3],  # l_lower_leg = q5
            lq[4],  # l_ankle_1 = q6
            lq[5],  # l_ankle_2 = q7
            rq[2],  # l_hip_1 = q4
            rq[1],  # l_hip_2 = q3
            rq[0],  # l_upper_leg = q0
            - rq[3],  # l_lower_leg = q5
            rq[4],  # l_ankle_1 = q6
            - rq[5],  # l_ankle_2 = q7
        ])


class SimpleModel(RobotModel):
    model_file = "data/models/simple_model.lua"
    chest_body_id = "pelvis"
    lfoot_body_id = "sole_left"
    rfoot_body_id = "sole_right"
    lankle_body_id = "ankle_left"
    rankle_body_id = "ankle_right"
    lknee_body_id = "knee_left"
    rknee_body_id = "knee_right"
    lhip_body_id = "hip_left"
    rhip_body_id = "hip_right"
    initial_pose_walking = np.array([
        0.000000000000000000, 0.000000000000000000, 0.680000000000000049,
        0.000000000000000000, 0.000000000000000000, 0.000000000000000000,
        -0.000000000000000000, 0.000000000000000000, 0.206880378272365800,
        0.484302315745919143, -0.277421937473553371, 0.000000000000000000,
        -0.000000000000000000, 0.000000000000000000, 0.206880378272365800,
        0.484302315745919143, -0.277421937473553371, 0.000000000000000000
    ])
    meshup_header = meshup.MESHUP_HEADER_SIMPLE
    step_length = 0.2
    foot_distance = 0.25 * 2
    com_height = 0.60


class SimpleModelLX5(SimpleModel):
    model_file = "data/models/simple_model_legsx5.lua"
    com_height = 0.65


class SimpleModelLX10(SimpleModel):
    model_file = "data/models/simple_model_legsx10.lua"
    com_height = 0.70
