const mf = @import("math_functions-prelude.zig");

pub const WorldId = struct {
    index1: u16,
    generation: u16,
};

/// Body id references a body instance. This should be treated as an opaque handle.
pub const BodyId = struct {
    index1: i32,
    world0: u16,
    generation: u16,
};

/// Shape id references a shape instance. This should be treated as an opaque handle.
pub const ShapeId = struct {
    index1: i32,
    world0: u16,
    generation: u16,
};

/// Chain id references a chain instances. This should be treated as an opaque handle.
pub const ChainId = struct {
    index1: i32,
    world0: u16,
    generation: u16,
};

/// Joint id references a joint instance. This should be treated as an opaque handle.
pub const JointId = struct {
    index1: i32,
    world0: u16,
    generation: u16,
};
