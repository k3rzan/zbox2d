pub const Vec2 = struct {
    /// coordinates
    x: f32,
    y: f32,
};

/// Cosine and sine pair
/// This uses a custom implementation designed for cross-platform determinism
pub const CosSin = struct {
    /// cosine and sine
    cosine: f32,
    sine: f32,
};

/// 2D rotation
/// This is similar to using a complex number for rotation
pub const Rot = struct {
    /// cosine and sine
    c: f32,
    s: f32,
};

/// A 2D rigid transform
pub const Transform = struct {
    p: Vec2,
    q: Rot,
};

/// A 2-by-2 Matrix
pub const Mat22 = struct {
    /// columns
    cx: Vec2,
    cy: Vec2,
};

/// Axis-aligned bounding box
pub const AABB = struct {
    lowerBound: Vec2,
    upperBound: Vec2,
};

/// https://en.wikipedia.org/wiki/Pi
const PI = 3.14159265359;

const Vec2_zero = Vec2{ .x = 0.0, .y = 0.0 };
const Rot_identity = Rot{ .c = 1.0, .s = 0.0 };
const Transform_identity = Transform{
    .p = .{ .x = 0.0, .y = 0.0 },
    .q = .{ .x = 1.0, .y = 0.0 },
};
const Mat22_zero = Mat22{ .cx = .{ .x = 0.0, .y = 0.0 }, .cy = .{ .x = 0.0, .y = 0.0 } };
