import numpy as np
import rbdl


def calculate_zmp_trajectory(model, q, qdot, qddot, chest):
    """
    Calculate the ZMP trajectory. Corresponds to the CalculateZMP()
    algorithm in the thesis.
    """
    zmp = np.zeros((len(q), 3))

    for t in range(len(q)):  # Iterate over timesteps
        # Calculate tau using rbdl
        tau = np.zeros(model.model.qdot_size)
        # TODO check if kinematics are updated?
        # rbdl.UpdateKinematics is not available in wrapper
        rbdl.InverseDynamics(model.model, q[t], qdot[t], qddot[t], tau)

        # NOTE this only works when the first joint is a floating base joint
        F_ext = tau[0:3]
        M_ext = tau[3:6]

        # Calculate ZMP relative to CoM
        # TODO do we have to map it to the CoM?
        _zmp = 1 / (F_ext[2]) * np.array([-M_ext[1], M_ext[0], 0])

        # Calculate CoM position and transform to ZMP world coordinate
        # TODO is this correct?
        zmp_pos = rbdl.CalcBodyToBaseCoordinates(
            model.model, q[t], chest.id, chest.body_point
        ) + _zmp

        zmp_pos[2] = 0
        zmp[t] = zmp_pos

    return zmp


def calculate_real_zmp_trajectory(model, q, qdot, qddot, chest, lsole, rsole):
    """
    Calculate the ZMP trajectory. Corresponds to the CalculateZMP()
    algorithm in the thesis.
    """
    zmp = np.zeros((len(q), 3))

    bid_left = model.model.GetBodyId(model.lfoot_body_id)
    bid_right = model.model.GetBodyId(model.lfoot_body_id)
    cs_left = rbdl.ConstraintSet()
    cs_right = rbdl.ConstraintSet()
    cs_both = rbdl.ConstraintSet()
    points = (np.array([-.025, -.01, 0]), np.array([0.025, -0.01, 0]), np.array([0, 0.01, 0]))

    for point in points:
        for direction in (np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])):
            cs_left.AddContactConstraint(bid_left, point, direction)
            cs_both.AddContactConstraint(bid_left, point, direction)

    for point in points:
        for direction in (np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])):
            cs_right.AddContactConstraint(bid_right, point, direction)
            cs_both.AddContactConstraint(bid_right, point, direction)

    cs_left.Bind(model.model)
    cs_right.Bind(model.model)
    cs_both.Bind(model.model)


    for t in range(len(q)):  # Iterate over timesteps
        # Calculate tau using rbdl
        tau = np.zeros(model.model.qdot_size)
        # TODO check if kinematics are updated?
        # rbdl.UpdateKinematics is not available in wrapper
        rbdl.InverseDynamics(model.model, q[t], qdot[t], qddot[t], tau)

        if lsole.traj_pos[t, 2] < 1e-8 and rsole.traj_pos[t, 2] < 1e-8:
            cs = cs_both
            cstype = 'both'
        elif lsole.traj_pos[t, 2] < 1e-8:
            cs = cs_left
            cstype = 'left'
        else:
            cs = cs_right
            cstype = 'right'

        com_tmp = np.zeros(3)
        mass = rbdl.CalcCenterOfMass(model.model, q[t], np.zeros(model.dof_count), com_tmp)

        rbdl.ForwardDynamicsForwardDynamicsConstraintsDirect(model.model, q[t], qdot[t], tau, cs, qddot[t])
        lmbda = cs.force

        net_force = np.zeros(3)
        net_moment = np.zeros(3)

        force_gravity = mass * model.model.gravity
        net_force += force_gravity
        net_moment += np.cross(com_tmp, force_gravity)

        if cstype in ('left', 'both'):
            forces = np.split(lmbda[:9], len(points))
            for i, point in enumerate(points):
                point_pos = rbdl.CalcBodyToBaseCoordinates(model.model, q[t], bid_left, point)
                net_force += forces[i]
                net_moment += np.cross(point_pos, forces[i])

        if cstype in ('right', 'both'):
            if cstype == 'both':
                forces = np.split(lmbda[9:], len(points))
            else:
                forces = np.split(lmbda, len(points))
            for i, point in enumerate(points):
                point_pos = rbdl.CalcBodyToBaseCoordinates(model.model, q[t], bid_right, point)
                net_force += forces[i]
                net_moment += np.cross(point_pos, forces[i])

        com = rbdl.CalcBodyToBaseCoordinates(
            model.model, q[t], chest.id, chest.body_point
        )

        net_moment = np.cross(com, force_gravity)

        _zmp = 1 / (net_force[2]) * np.array([-net_moment[1], net_moment[0], 0])
        zmp[t] = _zmp

    return zmp
