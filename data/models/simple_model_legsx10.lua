--[[
    Implementation of a 3 DoF SCARA arm to test inverse kinematics algorithms.
    Author: Manuel Kudruss <manuel.kudruss@iwr.uni-heidelberg.de>
--]]

-- define some convenience functions

function get_point_by_name (container, name)
    for key, value in ipairs(container) do
        if value["name"] == name then
            value["coordinates"] = value["point"]
            return value
        end
    end
    print('container does not contain point with name: ', name)
end

function get_sphere_inertia (mass, radius)
    local val
    val = 2.0/5.0 * mass * radius * radius
    return {
        {val, 0.0, 0.0},
        {0.0, val, 0.0},
        {0.0, 0.0, val}
    }
end

function get_box_inertia (mass, width, height, depth)
    local valx, valy, valz
    valx = 1.0/12.0 * mass * (height*height + depth*depth)
    valy = 1.0/12.0 * mass * (width*width + height*height)
    valz = 1.0/12.0 * mass * (width*width + depth*depth)
    return {
        {valx,  0.0,  0.0},
        { 0.0, valy,  0.0},
        { 0.0,  0.0, valz}
    }
end

function get_segment(mass, com, Ic)
    local segment = {}
    segment.mass    = mass
    segment.com     = com
    segment.inertia = Ic
    return segment
end

-- define some constants

constants = {
    hip_w = 0.5,

    pelvis_m = 20,
    pelvis_l = 0.2,
    pelvis_w = 0.9*0.5,
    pelvis_h = 0.2,

    thigh_m = 4,
    thigh_l = 0.2,
    thigh_w = 0.2,
    thigh_h = 0.4,

    shank_m = 3,
    shank_l = 0.2,
    shank_w = 0.2,
    shank_h = 0.3,

    foot_m = 1.0,
    foot_l = 0.3,
    foot_w = 0.2,
    foot_h = 0.1,
}

-- define bodies
bodies = {
    pelvis = get_segment(
        constants.pelvis_m,
        {0, 0, 0},
        get_box_inertia(constants.pelvis_m, constants.pelvis_l, constants.pelvis_w, constants.pelvis_h)
    ),
    hip_right = get_segment(
        constants.thigh_m,
        {constants.thigh_l/2, 0, 0},
        get_box_inertia(constants.thigh_m, constants.thigh_l, constants.thigh_w, constants.thigh_h)
    ),
    hip_left = get_segment(
        constants.thigh_m,
        {constants.thigh_l/2, 0, 0},
        get_box_inertia(constants.thigh_m, constants.thigh_l, constants.thigh_w, constants.thigh_h)
    ),
    knee_right = get_segment(
        constants.shank_m,
        {constants.shank_l/2, 0, 0},
        get_box_inertia(constants.shank_m, constants.shank_l, constants.shank_w, constants.shank_h)
    ),
    knee_left = get_segment(
        constants.shank_m,
        {constants.shank_l/2, 0, 0},
        get_box_inertia(constants.shank_m, constants.shank_l, constants.shank_w, constants.shank_h)
    ),
    ankle_right = get_segment(
        constants.foot_m,
        {constants.foot_l/2, 0, 0},
        get_box_inertia(constants.foot_m, constants.foot_l, constants.foot_w, constants.foot_h)
    ),
    ankle_left = get_segment(
        constants.foot_m,
        {constants.foot_l/2, 0, 0},
        get_box_inertia(constants.foot_m, constants.foot_l, constants.foot_w, constants.foot_h)
    ),
}

-- define degrees of freedom of model
joints = {
    fixed = {},
    free_flyer = {
        { 0., 0., 0., 1., 0., 0.},
        { 0., 0., 0., 0., 1., 0.},
        { 0., 0., 0., 0., 0., 1.},
        { 1., 0., 0., 0., 0., 0.},
        { 0., 1., 0., 0., 0., 0.},
        { 0., 0., 1., 0., 0., 0.}
    },
    hip_right = {
        { 0., 0., -1., 0., 0., 0.},
        { -1., 0., 0., 0., 0., 0.},
        { 0., -1., 0., 0., 0., 0.},
    },
    knee_right = {
        { 0., 1., 0., 0., 0., 0.}
    },
    ankle_right = {
        { 0., 1., 0., 0., 0., 0.},
        { 1., 0., 0., 0., 0., 0.},
    },
    hip_left = {
        { 0., 0., -1., 0., 0., 0.},
        { -1., 0., 0., 0., 0., 0.},
        { 0., -1., 0., 0., 0., 0.},
    },
    knee_left = {
        { 0., 1., 0., 0., 0., 0.}
    },
    ankle_left = {
        { 0., 1., 0., 0., 0., 0.},
        { 1., 0., 0., 0., 0., 0.},
    },
}

-- define meshes for visualization

colors = {
    color_body = {0.0, 0.0, 1.0},
    color_left = {1.0, 0.0, 0.0},
    color_left2 = {1.0, 0.2, 0.2},
    color_right = {0.0, 1.0, 0.0},
    color_right2 = {0.2, 1.0, 0.2},
    -- color_foot = {0.0, 0.0, 1.0}
}

meshes = {
    -- PELVIS
    pelvis = {
        name        = "pelvis",
        dimensions  = {
            constants.pelvis_l, constants.pelvis_w, constants.pelvis_h
        },
        color       = colors.color_body,
        mesh_center = { 0.0, 0.0, 0.0 },
        src         = "meshes/unit_cube.obj",
    },
    -- THIGH LEFT
    thigh_left = {
        name        = "thigh_left",
        dimensions  = {
            constants.thigh_l, constants.thigh_w, constants.thigh_h
        },
        color       = colors.color_left,
        mesh_center = { 0.0, 0.0, -0.5*constants.thigh_h },
        src         = "meshes/unit_cube.obj",
    },
    -- SHANK LEFT
    shank_left = {
        name        = "shank_left",
        dimensions  = {
            constants.shank_l, constants.shank_w, constants.shank_h
        },
        color       = colors.color_left2,
        mesh_center = { 0.0, 0.0, -0.5*constants.shank_h },
        src         = "meshes/unit_cube.obj",
    },
    -- FOOT LEFT
    foot_left = {
        name        = "foot_left",
        dimensions  = {
            constants.foot_l, constants.foot_w, constants.foot_h
        },
        color       = colors.color_left,
        mesh_center = { 0.5*constants.foot_l - 0.5 * constants.shank_w, 0.0, -0.5*constants.foot_h },
        src         = "meshes/unit_cube.obj",
    },
    -- THIGH RIGHT
    thigh_right = {
        name        = "thigh_right",
        dimensions  = {
            constants.thigh_l, constants.thigh_w, constants.thigh_h
        },
        color       = colors.color_right,
        mesh_center = { 0.0, 0.0, -0.5*constants.thigh_h },
        src         = "meshes/unit_cube.obj",
    },
    -- SHANK RIGHT
    shank_right = {
        name        = "shank_right",
        dimensions  = {
            constants.shank_l, constants.shank_w, constants.shank_h
        },
        color       = colors.color_right2,
        mesh_center = { 0.0, 0.0, -0.5*constants.shank_h },
        src         = "meshes/unit_cube.obj",
    },
    -- FOOT RIGHT
    foot_right = {
        name        = "foot_right",
        dimensions  = {
            constants.foot_l, constants.foot_w, constants.foot_h
        },
        color       = colors.color_right,
        mesh_center = { 0.5*constants.foot_l - 0.5 * constants.shank_w, 0.0, -0.5*constants.foot_h },
        src         = "meshes/unit_cube.obj",
    }
}

--[[ Contact Point Definition

     Point Information Table
     point = {
        coordinates -- type: (3-d vector), point coordinates relative to frame
        color       -- type: (3-d vector, default: {1., 1., 1.})
        draw_line   -- (boolean, default: false), draw line from frame origin to point?
        line_width  -- (number, default: 1.) line width in pixels
     }
--]]

contact_points = {
}

--[[ Contact Set Definition

     Constraint Set Information Table
     set = { -- (array of (point, normal tuples))
         {
            point   -- type: (string), name of corresponding point in contact_points
            normal  -- type: (3-d vector), normal of contact point on surface
         },
     }
--]]

contact_sets = {
}

--[[ Model Assembly

    Frame Information Table
    frame = {
        name        -- (required, type: string): Name of body. Name must be unique.
        parent      -- (required, type: string): "ROOT" for base coordinate system,
                    -- otherwise must be exact name of parent frame.
        joint_frame -- (optional, type: table): Specifies the origin of the joint
                    -- in the frame of the parent. It uses the values (if existing):
            r -- (3-d vector, default: (0., 0., 0.)),
            E -- (3x3 matrix, default: identity matrix) for which r is the translation
              -- and E the rotation of the joint frame.
        visuals     -- (optional, type: array of Mesh Information Tables):
                    -- Specification of all meshes that are attached to the current frame.
        points      -- (optional, type: array of Point Information Tables):
                    -- Additional points that are specified relative to the frame
                    -- and move with it.
    }
--]]
model = {
    -- including definitions into model
    -- this can be used to distinguish plotting model ./model.lua
    -- and ./SRC/muscod_model.lua, that loads this plotting model where
    -- changes can be made
    constants = constants,
    bodies = bodies,
    joints = joints,
    colors = colors,
    meshes = meshes,

    -- configuration of the environment
    configuration = {
        axis_front = { 1, 0, 0},
        axis_up    = { 0, 0, 1},
        axis_right = { 0, -1, 0},
    },

    -- gravity can be transformed to simulate slope walking
    gravity = {0., 0., -9.81},

    -- configuration of contact points and sets
    -- Note: needed for simulation with ODESIM and optimal control with MUSCOD
    points = contact_points,
    constraint_sets = contact_sets,

    -- assembly of the multi-body system
    frames = {
        -- PELVIS: pelvis body of the robot with floating-base attached
        {
            name    = "pelvis",
            parent  = "ROOT",
            joint   = joints.free_flyer,
            body = bodies.pelvis,
            joint_frame = {
                r = { 0.0, 0.0, 0.0},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.pelvis,
            },
        },
        -- HIP RIGHT: upper part of the leg attached to pelvis via hip
        {
            name    = "hip_right",
            parent  = "pelvis",
            joint   = joints.hip_right,
            body = bodies.hip_right,
            joint_frame = {
                r = { 0.0, -0.5*constants.hip_w, 0.0},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.thigh_right,
            },
        },
        -- KNEE RIGHT: lower part of the leg attached to thigh via knee
        {
            name    = "knee_right",
            parent  = "hip_right",
            joint   = joints.knee_right,
            body = bodies.knee_right;
            joint_frame = {
                r = { 0.0, 0.0, -constants.thigh_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.shank_right,
            },
        },
        -- ANKLE RIGHT: right foot attached to shank via ankle
        {
            name    = "ankle_right",
            parent  = "knee_right",
            joint   = joints.ankle_right,
            body = bodies.ankle_right,
            joint_frame = {
                r = { 0.0, 0.0, -constants.shank_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.foot_right
            },
        },
        -- HIP LEFT: upper part of the leg attached to pelvis via hip
        {
            name    = "hip_left",
            parent  = "pelvis",
            joint   = joints.hip_left,
            body = bodies.hip_left,
            joint_frame = {
                r = { 0.0, 0.5*constants.hip_w, 0.0},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.thigh_left,
            },
        },
        -- KNEE LEFT: lower part of the leg attached to thigh via knee
        {
            name    = "knee_left",
            parent  = "hip_left",
            joint   = joints.knee_left,
            body = bodies.knee_left,
            joint_frame = {
                r = { 0.0, 0.0, -constants.thigh_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.shank_left,
            },
        },
        -- ANKLE LEFT: left foot attached to shank via ankle
        {
            name    = "ankle_left",
            parent  = "knee_left",
            joint   = joints.ankle_left,
            body = bodies.ankle_left,
            joint_frame = {
                r = { 0.0, 0.0, -constants.shank_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.foot_left
            },
        },
        {
            name    = "sole_left",
            parent  = "ankle_left",
            joint   = joints.fixed,
            joint_frame = {
                r = { 0.5 * constants.foot_l - 0.5 * constants.shank_w, 0.0, -constants.foot_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                },
            },
        },
        {
            name    = "sole_right",
            parent  = "ankle_right",
            joint   = joints.fixed,
            joint_frame = {
                r = { 0.5 * constants.foot_l - 0.5 * constants.shank_w, 0.0, -constants.foot_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                },
            },
        },
    },
}

return model
