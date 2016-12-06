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
body_r = 0.30 -- radius of the cylinder covering the upper body
body_h = 0.70 -- height of the cylinder covering the upper body

up_arm_l = 1.0
up_arm_w = 0.2
up_arm_h = 0.1

eb_joint_r = 0.20 -- radius of the cylinder covering the joint
eb_joint_h = 0.30 -- height of the cylinder covering the joint

lo_arm_l = 0.7
lo_arm_w = 0.1
lo_arm_h = 0.1

wr_joint_r = 0.10 -- radius of the cylinder covering the joint
wr_joint_h = 0.20 -- height of the cylinder covering the joint

hand_r = 0.05
hand_h = 1.0

constants = {
    body_r = body_r,
    body_m = body_m,
    up_arm_l = up_arm_l,
    up_arm_w = up_arm_w,
    up_arm_h = up_arm_h,
    eb_joint_r = eb_joint_r,
    eb_joint_h = eb_joint_h,
    lo_arm_l = lo_arm_l,
    lo_arm_w = lo_arm_w,
    lo_arm_h = lo_arm_h,
    wr_joint_r = wr_joint_r,
    wr_joint_h = wr_joint_h,
    hand_r = hand_r,
    hand_h = hand_h,
}

-- define bodies
bodies = {
    -- body = get_segment(body_m,  {0.0, 0.0,  0.0}, get_sphere_inertia (body_m,  body_r)),
    -- upper_arm = get_segment(body_m,  {0.0, 0.0,  0.0}, get_sphere_inertia (body_m,  body_r)),
    -- lower_arm = get_segment(body_m,  {0.0, 0.0,  0.0}, get_sphere_inertia (body_m,  body_r)),
    -- end_effector = get_segment(body_m,  {0.0, 0.0,  0.0}, get_sphere_inertia (body_m,  body_r)),
}

-- define degrees of freedom of model
joints = {
    fixed = {},
    rot_z = {
        { 0., 0., 1., 0., 0., 0.}
    },
    trans_z = {
        { 0., 0., 0., 0., 0., 1.}
    },
}

-- define meshes for visualization
color_body = {0.0, 0.0, 1.0}
color_uppr = {1.0, 0.0, 0.0}
color_lowr = {0.0, 1.0, 0.0}
color_hand = {0.0, 0.0, 1.0}

colors = {
    color_body = color_body,
    color_uppr = color_uppr,
    color_lowr = color_lowr,
    color_hand = color_hand,
}

meshes = {
    -- BODY
    body = {
        name        = "body",
        dimensions  = { body_r, body_r, body_h},
        color       = color_body,
        mesh_center = { 0.0, 0.0, -0.5*body_h},
        src         = "cylinder.obj"
    },
    body_cap = {
        name        = "body",
        dimensions  = { body_r, body_r, body_r},
        color       = color_body,
        mesh_center = { 0.0, 0.0, 0.0},
        src         = "unit_sphere_medres.obj"
    },
    -- Upper Arm
    upper_arm = {
        name        = "upper_arm",
        dimensions  = {up_arm_l, up_arm_w, up_arm_h},
        color       = color_uppr,
        mesh_center = { 0.5*up_arm_l, 0., -0.2*eb_joint_h},
        src         = "meshes/unit_cube.obj",
    },
    -- Elbow Joint --
    elbow_joint = {
        name        = "elbow_joint",
        dimensions  = {eb_joint_r, eb_joint_r, eb_joint_h},
        color       = color_body,
        mesh_center = { 0.0, 0.0, 0.25*eb_joint_h},
        src         = "cylinder.obj"
    },
    -- Lower Arm
    lower_arm = {
        name        = "lower_arm",
        dimensions  = {lo_arm_l, lo_arm_w, lo_arm_h},
        color       = color_lowr,
        mesh_center = {-0.5*lo_arm_l, 0.0, 0.0},
        src         = "meshes/unit_cube.obj",
    },
    -- Wrist Joint --
    wrist_joint = {
        name        = "wrist_joint",
        dimensions  = {wr_joint_r, wr_joint_r, wr_joint_h},
        color       = color_body,
        mesh_center = { 0.0, 0.0, 0.0},
        src         = "cylinder.obj"
    },
    -- Hand
    hand = {
        name        = "hand",
        dimensions  = {hand_r, hand_r, hand_h},
        color       = color_body,
        mesh_center = {0.0, 0.0, 0.5*hand_h},
        src         = "cylinder.obj"
    },
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
   {name = "TCP", point = {0., 0., 0.}, body = "hand",},
   {name = "TCP_x",  point = {1., 0., 0.}, body = "hand",},
   {name = "TCP_y",  point = {0., 1., 0.}, body = "hand",},
   {name = "TCP_z",  point = {0., 0., 1.}, body = "hand",}
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
        -- BODY: Root body where arms are attached
        {
            name    = "body",
            parent  = "ROOT",
            joint   = joints.rot_z,
            joint_frame = {
                r = { 0.0, 0.0, body_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.body,
		meshes.upper_arm,
                meshes.body_cap,
            },
        },
        -- UPPER ARM: First segment attached to body
        {
            name    = "upper_arm",
            parent  = "body",
            joint   = joints.rot_z,
            -- TODO add joint limits
            joint_limits = {},
            joint_frame = {
                r = { up_arm_l, 0.0, -0.5*eb_joint_h},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.elbow_joint,
            },
        },
        -- LOWER ARM: Second segment attached to body
        {
            name    = "lower_arm",
            parent  = "upper_arm",
            joint   = joints.fixed,
            -- TODO add joint limits
            joint_limits = {},
            joint_frame = {
                r = { lo_arm_l, 0.0, 0.0},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.lower_arm,
                meshes.wrist_joint,
            },
        },
        -- HAND: Last segment
        {
            name    = "hand",
            parent  = "lower_arm",
            joint   = joints.trans_z,
            -- TODO add joint limits
            joint_limits = {},
            joint_frame = {
                r = { 0.0, 0.0, 0.0},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
            },
        },
        -- HAND: Last segment with TCP
        {
            name    = "TCP",
            parent  = "hand",
            joint   = joints.fixed,
            -- TODO add joint limits
            joint_frame = {
                r = { 0.0, 0.0, 0.0},
                E = {
                    { 1.0, 0.0, 0.0},
                    { 0.0, 1.0, 0.0},
                    { 0.0, 0.0, 1.0},
                    },
            },
            visuals = {
                meshes.hand,
            },
            points = { -- draw CoM
                body = get_point_by_name(contact_points, "TCP"),
            }
        },
    },
}

return model
