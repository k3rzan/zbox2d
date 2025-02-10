const std = @import("std");
const box2d = @import("box2d.zig");

pub const maxPolygonVertices: usize = 8;

const b2FinishTaskCallback = fn (userTask: ?*anyopaque, userContext: ?*anyopaque) callconv(.C) void;
const b2TaskCallback = fn (i32, i32, u32, ?*anyopaque) callconv(.C) void;
const b2EnqueueTaskCallback = fn (?*const b2TaskCallback, i32, i32, ?*anyopaque, ?*anyopaque) callconv(.C) ?*anyopaque;

const b2RestitutionCallback = fn (restitutionA: f32, materialA: i32, restitutionB: f32, materialB: i32) callconv(.C) f32;

const b2FrictionCallback = fn (frictionA: f32, materialA: i32, frictionB: i32, materialB: i32) callconv(.C) f32;

pub const CustomFilterFn = fn (shapeIdA: ShapeId, shapeIdB: ShapeId, context: ?*anyopaque) callconv(.C) bool;
pub const CastResultFn = fn (shape: ShapeId, pos: Vec2, normal: Vec2, fraction: f32, context: ?*anyopaque) callconv(.C) f32;
pub const PreSolveFn = fn (shapeIdA: ShapeId, shapeIdB: ShapeId, manifold: *Manifold, context: ?*anyopaque) callconv(.C) bool;
pub const OverlapResultFn = fn (shape: ShapeId, context: ?*anyopaque) callconv(.C) bool;
pub const AllocFn = fn (size: c_uint, alignment: c_int) callconv(.C) ?*anyopaque;
pub const FreeFn = fn (mem: ?*anyopaque) callconv(.C) void;
pub const AssertFn = fn (condition: [*:0]const u8, fileName: [*:0]const u8, lineNumber: c_int) callconv(.C) c_int;

pub const CosSin = extern struct {
    cosine: f32,
    sine: f32,
};

pub const Mat22 = extern struct {
    cx: Vec2,
    cy: Vec2,
};

pub const Rot = extern struct {
    pub const identity = Rot{
        .c = 1,
        .s = 0,
    };

    pub inline fn fromRadians(angle: f32) Rot {
        return @bitCast(box2d.b2MakeRot(angle));
    }

    pub inline fn normalize(q: Rot) Rot {
        const mag: f32 = @sqrt((q.s * q.s) + (q.c * q.c));
        const invMag: f32 = if (mag > 0.0) 1.0 / mag else 0.0;
        const qn: Rot = Rot{
            .c = q.c * invMag,
            .s = q.s * invMag,
        };
        return qn;
    }

    pub inline fn isNormalized(q: Rot) bool {
        const qq: f32 = (q.s * q.s) + (q.c * q.c);
        // larger tolerance due to failure on mingw 32-bit
        return ((1.0 - 0.0006) < qq) and (qq < (1.0 + 0.0006));
    }

    pub inline fn nLerp(q1: Rot, q2: Rot, t: f32) Rot {
        const omt: f32 = 1.0 - t;
        const q: Rot = Rot{
            .c = (omt * q1.c) + (t * q2.c),
            .s = (omt * q1.s) + (t * q2.s),
        };
        return q.normalize();
    }

    pub inline fn integrateRotation(q1: Rot, deltaAngle: f32) Rot {
        const q2: Rot = Rot{
            .c = q1.c - (deltaAngle * q1.s),
            .s = q1.s + (deltaAngle * q1.c),
        };
        const mag: f32 = @sqrt((q2.s * q2.s) + (q2.c * q2.c));
        const invMag: f32 = if (mag > 0.0) 1.0 / mag else 0.0;
        const qn: Rot = Rot{
            .c = q2.c * invMag,
            .s = q2.s * invMag,
        };
        return qn;
    }

    pub inline fn computeAngularVelocity(q1: Rot, q2: Rot, inv_h: f32) f32 {
        const omega: f32 = inv_h * ((q2.s * q1.c) - (q2.c * q1.s));
        return omega;
    }

    pub inline fn toRadians(q: Rot) f32 {
        return box2d.b2Atan2(q.s, q.c);
    }

    pub inline fn getXAxis(q: Rot) Vec2 {
        const v: Vec2 = Vec2{
            .x = q.c,
            .y = q.s,
        };
        return v;
    }

    pub inline fn getYAxis(q: Rot) Vec2 {
        const v: Vec2 = Vec2{
            .x = -q.s,
            .y = q.c,
        };
        return v;
    }

    pub inline fn mul(q: Rot, r: Rot) Rot {
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        // s(q + r) = qs * rc + qc * rs
        // c(q + r) = qc * rc - qs * rs
        var qr: Rot = undefined;
        qr.s = (q.s * r.c) + (q.c * r.s);
        qr.c = (q.c * r.c) - (q.s * r.s);
        return qr;
    }

    pub inline fn invMul(q: Rot, r: Rot) Rot {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        // s(q - r) = qc * rs - qs * rc
        // c(q - r) = qc * rc + qs * rs
        var qr: Rot = undefined;
        qr.s = (q.c * r.s) - (q.s * r.c);
        qr.c = (q.c * r.c) + (q.s * r.s);
        return qr;
    }

    pub inline fn relativeAngle(b: Rot, a: Rot) f32 {
        const s: f32 = (b.s * a.c) - (b.c * a.s);
        const c: f32 = (b.c * a.c) + (b.s * a.s);
        return atan2(s, c);
    }

    pub inline fn rotateVector(q: Rot, v: Vec2) Vec2 {
        return Vec2{
            .x = (q.c * v.x) - (q.s * v.y),
            .y = (q.s * v.x) + (q.c * v.y),
        };
    }

    pub inline fn invRotateVector(q: Rot, v: Vec2) Vec2 {
        return Vec2{
            .x = (q.c * v.x) + (q.s * v.y),
            .y = (-q.s * v.x) + (q.c * v.y),
        };
    }

    pub inline fn isValid(q: Rot) bool {
        return box2d.b2Rot_IsValid(q);
    }

    /// Cosine component
    c: f32,
    /// Sine component
    s: f32,
};

pub const Transform = extern struct {
    pub const identity = Transform{
        .p = Vec2.zero,
        .q = Rot.identity,
    };

    pub inline fn transformPoint(t: Transform, p: Vec2) Vec2 {
        return Vec2{
            .x = ((t.q.c * p.x) - (t.q.s * p.y)) + t.p.x,
            .y = ((t.q.s * p.x) + (t.q.c * p.y)) + t.p.y,
        };
    }

    pub inline fn invTransformPoint(t: Transform, p: Vec2) Vec2 {
        const vx: f32 = p.x - t.p.x;
        const vy: f32 = p.y - t.p.y;
        return Vec2{
            .x = (t.q.c * vx) + (t.q.s * vy),
            .y = (-t.q.s * vx) + (t.q.c * vy),
        };
    }

    pub inline fn mul(A: Transform, B: Transform) Transform {
        return Transform{
            .q = A.q.mul(B.q),
            .p = A.q.rotateVector(B.p).add(A.p),
        };
    }

    pub inline fn invMul(A: Transform, B: Transform) Transform {
        return Transform{
            .q = A.q.invMul(B.q),
            .p = A.q.invRotateVector(B.p.sub(A.p)),
        };
    }
    p: Vec2,
    q: Rot,
};

pub const AABB = extern struct {
    lowerBound: Vec2,
    upperBound: Vec2,

    pub inline fn contains(a: AABB, b: AABB) bool {
        var s: bool = true;
        s = s and (a.lowerBound.x <= b.lowerBound.x);
        s = s and (a.lowerBound.y <= b.lowerBound.y);
        s = s and (b.upperBound.x <= a.upperBound.x);
        s = s and (b.upperBound.y <= a.upperBound.y);
        return s;
    }

    pub inline fn center(a: AABB) Vec2 {
        const b = Vec2{
            .x = 0.5 * (a.lowerBound.x + a.upperBound.x),
            .y = 0.5 * (a.lowerBound.y + a.upperBound.y),
        };
        return b;
    }

    pub inline fn extents(a: AABB) Vec2 {
        const b = Vec2{
            .x = 0.5 * (a.upperBound.x - a.lowerBound.x),
            .y = 0.5 * (a.upperBound.y - a.lowerBound.y),
        };
        return b;
    }

    /// Renamed from `union` due to a conflict with the union keyword
    pub inline fn add(a: AABB, b: AABB) AABB {
        var c: AABB = undefined;
        c.lowerBound.x = @min(a.lowerBound.x, b.lowerBound.x);
        c.lowerBound.y = @min(a.lowerBound.y, b.lowerBound.y);
        c.upperBound.x = @max(a.upperBound.x, b.upperBound.x);
        c.upperBound.y = @max(a.upperBound.y, b.upperBound.y);
        return c;
    }

    pub inline fn isValid(aabb: AABB) bool {
        return box2d.b2AABB_IsValid(@bitCast(aabb));
    }
};

pub const Vec2 = extern struct {
    pub inline fn dot(a: Vec2, b: Vec2) f32 {
        return (a.x * b.x) + (a.y * b.y);
    }

    pub inline fn cross(a: Vec2, b: Vec2) f32 {
        return (a.x * b.y) - (a.y * b.x);
    }

    pub inline fn crossVS(v: Vec2, s: f32) Vec2 {
        return Vec2{
            .x = s * v.y,
            .y = -s * v.x,
        };
    }

    pub inline fn crossSV(s: f32, v: Vec2) Vec2 {
        return Vec2{
            .x = -s * v.y,
            .y = s * v.x,
        };
    }

    pub inline fn leftPerp(v: Vec2) Vec2 {
        return Vec2{
            .x = -v.y,
            .y = v.x,
        };
    }

    pub inline fn rightPerp(v: Vec2) Vec2 {
        return Vec2{
            .x = v.y,
            .y = -v.x,
        };
    }

    pub inline fn add(a: Vec2, b: Vec2) Vec2 {
        return Vec2{
            .x = a.x + b.x,
            .y = a.y + b.y,
        };
    }

    pub inline fn sub(a: Vec2, b: Vec2) Vec2 {
        return Vec2{
            .x = a.x - b.x,
            .y = a.y - b.y,
        };
    }

    pub inline fn neg(a: Vec2) Vec2 {
        return Vec2{
            .x = -a.x,
            .y = -a.y,
        };
    }

    pub inline fn lerp(a: Vec2, b: Vec2, t: f32) Vec2 {
        return Vec2{
            .x = ((1.0 - t) * a.x) + (t * b.x),
            .y = ((1.0 - t) * a.y) + (t * b.y),
        };
    }

    pub inline fn mul(a: Vec2, b: Vec2) Vec2 {
        return Vec2{
            .x = a.x * b.x,
            .y = a.y * b.y,
        };
    }

    pub inline fn mulSV(s: f32, v: Vec2) Vec2 {
        return Vec2{
            .x = s * v.x,
            .y = s * v.y,
        };
    }

    pub inline fn mulAdd(a: Vec2, s: f32, b: Vec2) Vec2 {
        return Vec2{
            .x = a.x + (s * b.x),
            .y = a.y + (s * b.y),
        };
    }

    pub inline fn mulSub(a: Vec2, s: f32, b: Vec2) Vec2 {
        return Vec2{
            .x = a.x - (s * b.x),
            .y = a.y - (s * b.y),
        };
    }

    pub inline fn abs(a: Vec2) Vec2 {
        return Vec2{
            .x = @abs(a.x),
            .y = @abs(a.y),
        };
    }

    pub inline fn min(a: Vec2, b: Vec2) Vec2 {
        return Vec2{
            .x = @min(a.x, b.x),
            .y = @min(a.y, b.y),
        };
    }

    pub inline fn max(a: Vec2, b: Vec2) Vec2 {
        return Vec2{
            .x = @max(a.x, b.x),
            .y = @max(a.y, b.y),
        };
    }

    pub inline fn clamp(v: Vec2, lower: Vec2, upper: Vec2) Vec2 {
        return Vec2{
            .x = std.math.clamp(v.x, lower.x, upper.x),
            .y = std.math.clamp(v.y, lower.y, upper.y),
        };
    }

    pub inline fn length(v: Vec2) f32 {
        return @sqrt((v.x * v.x) + (v.y * v.y));
    }

    pub inline fn lengthSquared(v: Vec2) f32 {
        return (v.x * v.x) + (v.y * v.y);
    }

    pub inline fn distance(a: Vec2, b: Vec2) f32 {
        const dx: f32 = b.x - a.x;
        const dy: f32 = b.y - a.y;
        return @sqrt((dx * dx) + (dy * dy));
    }

    pub inline fn distanceSquared(a: Vec2, b: Vec2) f32 {
        const dx: f32 = b.x - a.x;
        const dy: f32 = b.y - a.y;
        return (dx * dx) + (dy * dy);
    }

    pub inline fn isValid(v: Vec2) bool {
        return box2d.b2Vec2_IsValid(@bitCast(v));
    }

    pub inline fn normalize(v: Vec2) Vec2 {
        const _length = v.length();
        if (_length < std.math.floatEps(f32)) {
            return Vec2{ .x = 0, .y = 0 };
        }
        const invLength = 1.0 / _length;
        return Vec2{ .x = invLength * v.x, .y = invLength * v.y };
    }

    pub inline fn getLengthAndNormalize(v: Vec2, len: *f32) Vec2 {
        len.* = v.length();
        if (len.* < std.math.floatEps(f32)) {
            return Vec2{ .x = 0, .y = 0 };
        }
        const invLength = 1.0 / len.*;
        return Vec2{ .x = invLength * v.x, .y = invLength * v.y };
    }

    pub const zero = Vec2{ .x = 0, .y = 0 };

    x: f32,
    y: f32,
};

pub inline fn atan2(y: f32, x: f32) f32 {
    return box2d.b2Atan2(y, x);
}

pub inline fn computeCosSin(angle: f32) CosSin {
    return @bitCast(box2d.b2ComputeCosSin(angle));
}

pub inline fn floatIsValid(a: f32) bool {
    return box2d.b2IsValid(a);
}

pub const Profile = extern struct {
    step: f32,
    pairs: f32,
    collide: f32,
    solve: f32,
    buildIslands: f32,
    solveConstraints: f32,
    prepareTasks: f32,
    solverTasks: f32,
    prepareConstraints: f32,
    integrateVelocities: f32,
    warmStart: f32,
    solveVelocities: f32,
    integratePositions: f32,
    relaxVelocities: f32,
    applyRestitution: f32,
    storeImpulses: f32,
    finalizeBodies: f32,
    splitIslands: f32,
    sleepIslands: f32,
    hitEvents: f32,
    broadphase: f32,
    continuous: f32,
};

pub const Counters = extern struct {
    staticBodyCount: i32,
    bodyCount: i32,
    shapeCount: i32,
    contactCount: i32,
    jointCount: i32,
    islandCount: i32,
    stackUsed: i32,
    staticTreeHeight: i32,
    treeHeight: i32,
    byteCount: i32,
    taskCount: i32,
    colorCounts: [12]i32,
};

pub const WorldDef = extern struct {
    gravity: Vec2,
    restitutionThreshold: f32,
    contactPushoutVelocity: f32,
    hitEventThreshold: f32,
    contactHertz: f32,
    contactDampingRatio: f32,
    jointHertz: f32,
    jointDampingRatio: f32,
    maximumLinearVelocity: f32,
    enableSleep: bool,
    enableContinuous: bool,
    workerCount: i32,
    internalValue: i32,
    enqueueTask: ?*const b2EnqueueTaskCallback = @import("std").mem.zeroes(?*const b2EnqueueTaskCallback),
    finishTask: ?*const b2FinishTaskCallback = @import("std").mem.zeroes(?*const b2FinishTaskCallback),
    userTaskContext: ?*const anyopaque,
};

pub const WorldId = extern struct {
    index1: u16,
    revision: u16,
};

pub inline fn getDefaultWorldDef() WorldDef {
    return @bitCast(box2d.b2DefaultWorldDef());
}

pub inline fn createWorld(world_def: WorldDef) WorldId {
    return @bitCast(box2d.b2CreateWorld(@ptrCast(&world_def)));
}

pub inline fn isWorldValid(world_id: WorldId) bool {
    return @bitCast(box2d.b2World_IsValid(@bitCast(world_id)));
}

pub inline fn WorldStep(world_id: WorldId, time_step: f32, sub_step_count: i32) void {
    box2d.b2World_Step(@bitCast(world_id), time_step, sub_step_count);
}

pub const BodyId = extern struct {
    index1: i32,
    world0: u16,
    revision: u16,
};

pub const BodyType = enum(c_int) {
    b2_staticBody = 0,
    b2_kinematicBody = 1,
    b2_dynamicBody = 2,
    b2_bodyTypeCount,
};

pub const BodyDef = extern struct {
    type: BodyType,
    position: Vec2,
    rotation: Rot,
    linearVelocity: Vec2,
    angularVelocity: f32,
    linearDamping: f32,
    angularDamping: f32,
    gravityScale: f32,
    sleepThreshold: f32,
    userData: ?*anyopaque,
    enableSleep: bool,
    isAwake: bool,
    fixedRotation: bool,
    isBullet: bool,
    isEnabled: bool,
    automaticMass: bool,
    allowFastRotation: bool,
    internalValue: i32,
};

pub const ShapeId = extern struct {
    index1: i32,
    world0: u16,
    revision: u16,
};

pub const ShapeType = enum {
    b2_circleShape,
    b2_capsuleShape,
    b2_segmentShape,
    b2_polygonShape,
    b2_smoothSegmentShape,
    b2_shapeTypeCount,
};

pub const Filter = extern struct {
    categoryBits: u32,
    maskBits: u32,
    groupIndex: i32,
};

pub const ShapeDef = extern struct {
    userData: ?*anyopaque,
    friction: f32,
    restitution: f32,
    density: f32,
    filter: Filter,
    customColor: u32,
    isSensor: bool,
    enableSensorEvents: bool,
    enableContactEvents: bool,
    enableHitEvents: bool,
    enablePreSolveEvents: bool,
    forceContactCreation: bool,
    internalValue: i32,
};

pub const B2_MAX_POLYGON_VERTICES: comptime_int = 8;

pub const TreeStats = extern struct {
    nodeVisits: i32,
    leafVisits: i32,
};

pub const Capsule = extern struct {
    center1: Vec2,
    center2: Vec2,
    radius: f32,

    pub inline fn containsPoint(shape: Capsule, point: Vec2) bool {
        return box2d.b2PointInCapsule(@bitCast(point), @ptrCast(&shape));
    }

    pub inline fn computeAABB(shape: Capsule, transform: Transform) AABB {
        return @bitCast(box2d.b2ComputeCapsuleAABB(@ptrCast(&shape), @bitCast(transform)));
    }

    pub inline fn computeMass(shape: Capsule, density: f32) MassData {
        return @bitCast(box2d.b2ComputeCapsuleMass(@ptrCast(&shape), density));
    }

    pub inline fn collideCircle(capsuleA: *const Capsule, xfA: Transform, circleB: *const Circle, xfB: Transform) Manifold {
        return @bitCast(box2d.b2CollideCapsuleAndCircle(@ptrCast(capsuleA), @bitCast(xfA), @ptrCast(circleB), @bitCast(xfB)));
    }

    pub inline fn collideCapsule(capsuleA: *const Capsule, xfA: Transform, capsuleB: *const Capsule, xfB: Transform) Manifold {
        return @bitCast(box2d.b2CollideCapsules(@ptrCast(capsuleA), @bitCast(xfA), @ptrCast(capsuleB), @bitCast(xfB)));
    }

    pub inline fn collideSegment(capsuleB: *const Capsule, xfB: Transform, segmentA: *const Segment, xfA: Transform) Manifold {
        return @bitCast(box2d.b2CollideSegmentAndCapsule(@ptrCast(segmentA), @bitCast(xfA), @ptrCast(capsuleB), @bitCast(xfB)));
    }

    pub inline fn collidePolygon(capsuleB: *const Capsule, xfB: Transform, polygonA: *const Polygon, xfA: Transform) Manifold {
        return @bitCast(box2d.b2CollidePolygonAndCapsule(@ptrCast(polygonA), @bitCast(xfA), @ptrCast(capsuleB), @bitCast(xfB)));
    }
};

pub const ChainSegment = extern struct {
    ghost1: Vec2,
    segment: Segment,
    ghost2: Vec2,
    chainId: i32,

    pub inline fn collideCircle(chainSegmentA: *const ChainSegment, xfA: Transform, circleB: *const Circle, xfB: Transform) Manifold {
        return @bitCast(box2d.b2CollideChainSegmentAndCircle(@ptrCast(chainSegmentA), @bitCast(xfA), @ptrCast(circleB), @bitCast(xfB)));
    }

    pub inline fn collideCapsule(chainSegmentA: *const ChainSegment, xfA: Transform, capsuleB: *const Capsule, xfB: Transform, cache: *DistanceCache) Manifold {
        return @bitCast(box2d.b2CollideChainSegmentAndCapsule(@ptrCast(chainSegmentA), @bitCast(xfA), @ptrCast(capsuleB), @bitCast(xfB), @ptrCast(cache)));
    }

    pub inline fn collidePolygon(chainSegmentA: *const ChainSegment, xfA: Transform, polygonB: *const Polygon, xfB: Transform, cache: *DistanceCache) Manifold {
        return @bitCast(box2d.b2CollideChainSegmentAndPolygon(@ptrCast(chainSegmentA), @bitCast(xfA), @ptrCast(polygonB), @bitCast(xfB), @ptrCast(cache)));
    }
};

pub const CastOutput = extern struct {
    normal: Vec2,
    point: Vec2,
    fraction: f32,
    iterations: i32,
    hit: bool,
};

pub const SimplexVertex = extern struct {
    wA: Vec2,
    wB: Vec2,
    w: Vec2,
    a: f32,
    indexA: i32,
    indexB: i32,
};

pub const Simplex = extern struct {
    v1: SimplexVertex,
    v2: SimplexVertex,
    v3: SimplexVertex,
    coint: i32,
};

pub const Segment = extern struct {
    point1: Vec2,
    point2: Vec2,

    pub inline fn computeAABB(shape: Segment, transform: Transform) AABB {
        return @bitCast(box2d.b2ComputeSegmentAABB(@ptrCast(&shape), @bitCast(transform)));
    }

    pub inline fn collideCircle(segmentA: *const Segment, xfA: Transform, circleB: *const Circle, xfB: Transform) Manifold {
        return @bitCast(box2d.b2CollideSegmentAndCircle(@ptrCast(segmentA), @bitCast(xfA), @ptrCast(circleB), @bitCast(xfB)));
    }

    pub inline fn collideCapsule(segmentA: *const Segment, xfA: Transform, capsuleB: *const Capsule, xfB: Transform) Manifold {
        return @bitCast(box2d.b2CollideSegmentAndCapsule(@ptrCast(segmentA), @bitCast(xfA), @ptrCast(capsuleB), @bitCast(xfB)));
    }

    pub inline fn collidePolygon(segmentA: *const Segment, xfA: Transform, polygonB: *const Polygon, xfB: Transform) Manifold {
        return @bitCast(box2d.b2CollideSegmentAndPolygon(@ptrCast(segmentA), @bitCast(xfA), @ptrCast(polygonB), @bitCast(xfB)));
    }
};

pub const DistanceCache = extern struct {
    count: u16,
    indexA: [3]u8,
    indexB: [3]u8,

    pub const empty = DistanceCache{
        .count = 0,
        .indexA = [3]u8{ 0, 0, 0 },
        .indexB = [3]u8{ 0, 0, 0 },
    };

    pub inline fn shapeDistanceDebug(cache: *DistanceCache, input: DistanceInput, simplexes: ?*Simplex, simplexCapacity: usize) DistanceOutput {
        return @bitCast(box2d.b2ShapeDistance(@ptrCast(cache), @ptrCast(&input), simplexes, @intCast(simplexCapacity)));
    }

    // This non-debug version is for normal people who don't need the one with the GJK debug output
    pub inline fn shapeDistance(cache: *DistanceCache, input: DistanceInput) DistanceOutput {
        return @bitCast(box2d.b2ShapeDistance(@ptrCast(cache), @ptrCast(&input), null, 0));
    }
};

pub const DistanceInput = extern struct {
    proxyA: DistanceProxy,
    proxyB: DistanceProxy,
    transformA: Transform,
    transformB: Transform,
    useRadii: bool,
};

pub const DistanceOutput = extern struct {
    pointA: Vec2,
    pointB: Vec2,
    distance: f32,
    iterations: i32,
    simplexCount: i32,
};

pub const ShapeCastPairInput = extern struct {
    proxyA: DistanceProxy,
    proxyB: DistanceProxy,
    transformA: Transform,
    transformB: Transform,
    translationB: Vec2,
    maxFraction: f32,

    pub inline fn shapeCast(input: ShapeCastPairInput) CastOutput {
        return @bitCast(box2d.b2ShapeCast(@ptrCast(&input)));
    }
};

pub const DistanceProxy = extern struct {
    points: [maxPolygonVertices]Vec2,
    count: i32,
    radius: f32,

    pub inline fn make(vertices: []const Vec2, radius: f32) DistanceProxy {
        return @bitCast(box2d.b2MakeProxy(@ptrCast(vertices.ptr), @intCast(vertices.len), radius));
    }
};

pub const Sweep = extern struct {
    localCenter: Vec2,
    c1: Vec2,
    c2: Vec2,
    q1: Rot,
    q2: Rot,

    pub inline fn getTransform(sweep: Sweep, time: f32) Transform {
        return @bitCast(box2d.b2GetSweepTransform(@ptrCast(&sweep), time));
    }
};

pub const MassData = extern struct {
    mass: f32,
    center: Vec2,
    rotationalInertia: f32,
};

pub const Circle = extern struct {
    center: Vec2,
    radius: f32,

    pub inline fn containsPoint(shape: Circle, point: Vec2) bool {
        return box2d.b2PointInCircle(@bitCast(point), @ptrCast(&shape));
    }

    pub inline fn computeAABB(shape: Circle, transform: Transform) AABB {
        return @bitCast(box2d.b2ComputeCircleAABB(@ptrCast(&shape), @bitCast(transform)));
    }

    pub inline fn computeMass(shape: Circle, density: f32) MassData {
        return @bitCast(box2d.b2ComputeCircleMass(@ptrCast(&shape), density));
    }

    pub inline fn collideCircle(circleA: *const Circle, xfA: Transform, circleB: *const Circle, xfB: Transform) Manifold {
        return @bitCast(box2d.b2CollideCircles(@ptrCast(circleA), @bitCast(xfA), @ptrCast(circleB), @bitCast(xfB)));
    }

    pub inline fn collideCapsule(circleB: *const Circle, xfB: Transform, capsuleA: *const Capsule, xfA: Transform) Manifold {
        return @bitCast(box2d.b2CollideCapsuleAndCircle(@ptrCast(capsuleA), @bitCast(xfA), @ptrCast(circleB), @bitCast(xfB)));
    }

    pub inline fn collideSegment(circleB: *const Circle, xfB: Transform, segmentA: *const Segment, xfA: Transform) Manifold {
        return @bitCast(box2d.b2CollideSegmentAndCircle(@ptrCast(segmentA), @bitCast(xfA), @ptrCast(circleB), @bitCast(xfB)));
    }

    pub inline fn collidePolygon(circleB: *const Circle, xfB: Transform, polygonA: *const Polygon, xfA: Transform) Manifold {
        return @bitCast(box2d.b2CollidePolygonAndCircle(@ptrCast(polygonA), @bitCast(xfA), @ptrCast(circleB), @bitCast(xfB)));
    }
};

pub const QueryFilter = extern struct {
    categoryBits: u64,
    maskBits: u64,

    pub inline fn default() QueryFilter {
        return @bitCast(box2d.b2DefaultQueryFilter());
    }
};

pub const Polygon = extern struct {
    vertices: [B2_MAX_POLYGON_VERTICES]Vec2,
    normals: [B2_MAX_POLYGON_VERTICES]Vec2,
    centroid: Vec2,
    radius: f32,
    count: i32,
};

pub const RayResult = extern struct {
    shapeId: ShapeId,
    point: Vec2,
    normal: Vec2,
    fraction: f32,
    hit: bool,
};

pub const SensorBeginTouchEvent = extern struct {
    sensorShapeId: ShapeId,
    visitorShapeId: ShapeId,
};

pub const SensorEndTouchEvent = extern struct {
    sensorShapeId: ShapeId,
    visitorShapeId: ShapeId,
};

pub const ManifoldPoint = extern struct {
    point: Vec2,
    anchorA: Vec2,
    anchorB: Vec2,
    separation: f32,
    normalImpulse: f32,
    tangentImpulse: f32,
    maxNormalImpulse: f32,
    normalVelocity: f32,
    id: u16,
    persisted: bool,
};

pub const Manifold = extern struct {
    points: [2]ManifoldPoint,
    normal: Vec2,
    pointCount: i32,
};

pub const ContactBeginTouchEvent = extern struct {
    shapeIdA: ShapeId,
    shapeIdB: ShapeId,
    manifold: Manifold,
};

pub const ContactEndTouchEvent = extern struct {
    shapeIdA: ShapeId,
    shapeIdB: ShapeId,
};

pub const ContactHitEvent = extern struct {
    shapeIdA: ShapeId,
    shapeIdB: ShapeId,
    point: Vec2,
    normal: Vec2,
    approachSpeed: f32,
};

pub const SensorEvents = extern struct {
    beginEvents: [*]SensorBeginTouchEvent,
    endEvents: [*]SensorEndTouchEvent,
    beginCount: i32,
    endCount: i32,
};

pub const ContactEvents = extern struct {
    beginEvents: [*]ContactBeginTouchEvent,
    endEvents: [*]ContactEndTouchEvent,
    hitEvents: [*]ContactHitEvent,
    beginCount: i32,
    endCount: i32,
    hitCount: i32,
};

pub const Hull = extern struct {
    points: [8]Vec2,
    count: i32,

    pub inline fn makePolygon(hull: Hull, radius: f32) Polygon {
        return @bitCast(box2d.b2MakePolygon(@ptrCast(&hull), radius));
    }

    pub inline fn makeOffsetPolygon(hull: Hull, position: Vec2, rotation: Rot) Polygon {
        return @bitCast(box2d.b2MakeOffsetPolygon(@ptrCast(&hull), @bitCast(position), @bitCast(rotation)));
    }

    pub inline fn compute(points: []const Vec2) Hull {
        return @bitCast(box2d.b2ComputeHull(@ptrCast(points.ptr), @intCast(points.len)));
    }

    pub inline fn validate(hull: Hull) bool {
        return box2d.b2ValidateHull(@ptrCast(&hull));
    }
};

pub inline fn isBodyValid(id: BodyId) bool {
    return @bitCast(box2d.b2Body_IsValid(@bitCast(id)));
}
pub inline fn getBodyPosition(bodyId: BodyId) Vec2 {
    return @bitCast(box2d.b2Body_GetPosition(@bitCast(bodyId)));
}

pub inline fn getBodyRotation(bodyId: BodyId) Rot {
    return @bitCast(box2d.b2Body_GetRotation(@bitCast(bodyId)));
}

pub inline fn getRotAngle(rot: Rot) f32 {
    return @bitCast(box2d.b2Rot_GetAngle(@bitCast(rot)));
}

pub inline fn getDefaultBodyDef() BodyDef {
    return @bitCast(box2d.b2DefaultBodyDef());
}

pub inline fn createBody(worldId: WorldId, def: *BodyDef) BodyId {
    return @bitCast(box2d.b2CreateBody(@bitCast(worldId), @ptrCast(def)));
}
pub inline fn getDefaultShapeDef() ShapeDef {
    return @bitCast(box2d.b2DefaultShapeDef());
}

pub inline fn createPolygonShape(bodyId: BodyId, def: *ShapeDef, polygon: *Polygon) ShapeId {
    return @bitCast(box2d.b2CreatePolygonShape(@bitCast(bodyId), @ptrCast(def), @ptrCast(polygon)));
}

pub inline fn makeBox(hx: f32, hy: f32) Polygon {
    return @bitCast(box2d.b2MakeBox(hx, hy));
}

pub inline fn getBodyLinearVelocity(bodyId: BodyId) Vec2 {
    return @bitCast(box2d.b2Body_GetLinearVelocity(@bitCast(bodyId)));
}

pub inline fn setBodyLinearVelocity(bodyId: BodyId, linear_velocity: Vec2) void {
    box2d.b2Body_SetLinearVelocity(@bitCast(bodyId), @bitCast(linear_velocity));
}

pub inline fn setTransform(bodyId: BodyId, position: Vec2, rotation: Vec2) void {
    box2d.b2Body_SetTransform(@bitCast(bodyId), @bitCast(position), @bitCast(rotation));
}

pub inline fn getWorldSensorEvents(world_id: WorldId) SensorEvents {
    return @bitCast(box2d.b2World_GetSensorEvents(@bitCast(world_id)));
}

pub inline fn getSensorEvents(worldId: WorldId) SensorEvents {
    return @bitCast(box2d.b2World_GetSensorEvents(@bitCast(worldId)));
}

pub inline fn getContactEvents(worldId: WorldId) ContactEvents {
    return @bitCast(box2d.b2World_GetContactEvents(@bitCast(worldId)));
}

pub inline fn overlapAABB(worldId: WorldId, aabb: AABB, filter: QueryFilter, overlapFn: *OverlapResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_OverlapAABB(@bitCast(worldId), @bitCast(aabb), @bitCast(filter), @ptrCast(overlapFn), @ptrCast(context)));
}

pub inline fn overlapCircle(worldId: WorldId, circle: Circle, transform: Transform, filter: QueryFilter, overlapFn: *OverlapResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_OverlapCircle(@bitCast(worldId), @ptrCast(&circle), @bitCast(transform), @bitCast(filter), @ptrCast(overlapFn), @ptrCast(context)));
}

pub inline fn overlapPoint(worldId: WorldId, point: Vec2, transform: Transform, filter: QueryFilter, overlapFn: OverlapResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_OverlapPoint(@bitCast(worldId), @bitCast(point), @bitCast(transform), @bitCast(filter), @ptrCast(overlapFn), context));
}

pub inline fn overlapCapsule(worldId: WorldId, capsule: Capsule, transform: Transform, filter: QueryFilter, overlapFn: *OverlapResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_OverlapCapsule(@bitCast(worldId), @ptrCast(&capsule), @bitCast(transform), @bitCast(filter), @ptrCast(overlapFn), @ptrCast(context)));
}

pub inline fn overlapPolygon(worldId: WorldId, polygon: Polygon, transform: Transform, filter: QueryFilter, overlapFn: *OverlapResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_OverlapPolygon(@bitCast(worldId), @ptrCast(&polygon), @bitCast(transform), @bitCast(filter), @ptrCast(overlapFn), @ptrCast(context)));
}

pub inline fn castRay(worldId: WorldId, origin: Vec2, translation: Vec2, filter: QueryFilter, castFn: *CastResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_CastRay(@bitCast(worldId), @bitCast(origin), @bitCast(translation), @bitCast(filter), @ptrCast(castFn), @ptrCast(context)));
}

pub inline fn castRayClosest(worldId: WorldId, origin: Vec2, translation: Vec2, filter: QueryFilter) RayResult {
    return @bitCast(box2d.b2World_CastRayClosest(@bitCast(worldId), @bitCast(origin), @bitCast(translation), @bitCast(filter)));
}

pub inline fn castCircle(worldId: WorldId, circle: Circle, originTransform: Transform, translation: Vec2, filter: QueryFilter, castFn: *CastResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_CastCircle(@bitCast(worldId), @ptrCast(&circle), @bitCast(originTransform), @bitCast(translation), @bitCast(filter), @ptrCast(castFn), @ptrCast(context)));
}

pub inline fn castCapsule(worldId: WorldId, capsule: Capsule, originTransform: Transform, translation: Vec2, filter: QueryFilter, castFn: *CastResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_CastCapsule(@bitCast(worldId), @ptrCast(&capsule), @bitCast(originTransform), @bitCast(translation), @bitCast(filter), @ptrCast(castFn), @ptrCast(context)));
}

pub inline fn castPolygon(worldId: WorldId, polygon: Polygon, originTransform: Transform, translation: Vec2, filter: QueryFilter, castFn: *CastResultFn, context: ?*anyopaque) TreeStats {
    return @bitCast(box2d.b2World_CastPolygon(@bitCast(worldId), @ptrCast(&polygon), @bitCast(originTransform), @bitCast(translation), @bitCast(filter), @ptrCast(castFn), @ptrCast(context)));
}

pub inline fn enableSleeping(worldId: WorldId, flag: bool) void {
    box2d.b2World_EnableSleeping(@bitCast(worldId), flag);
}

pub inline fn isSleepingEnabled(worldId: WorldId) bool {
    return box2d.b2World_IsSleepingEnabled(@bitCast(worldId));
}

pub inline fn enableWarmStarting(worldId: WorldId, flag: bool) void {
    box2d.b2World_EnableWarmStarting(@bitCast(worldId), flag);
}

pub inline fn isWarmStartingEnabled(worldId: WorldId) bool {
    return box2d.b2World_IsWarmStartingEnabled(@bitCast(worldId));
}

pub inline fn enableContinuous(worldId: WorldId, flag: bool) void {
    box2d.b2World_EnableContinuous(@bitCast(worldId), flag);
}

pub inline fn isContinuousEnabled(worldId: WorldId) bool {
    box2d.b2World_IsContinuousEnabled(@bitCast(worldId));
}

pub inline fn setRestitutionThreshold(worldId: WorldId, value: f32) void {
    box2d.b2World_SetRestitutionThreshold(@bitCast(worldId), value);
}

pub inline fn getRestitutionThreshold(worldId: WorldId) f32 {
    return box2d.b2World_GetRestitutionThreshold(worldId);
}

pub inline fn setHitEventThreshold(worldId: WorldId, value: f32) void {
    box2d.b2World_SetHitEventThreshold(@bitCast(worldId), value);
}

pub inline fn getHitEventThreshold(worldId: WorldId) f32 {
    return box2d.b2World_GetHitEventThreshold(worldId);
}

pub inline fn setPreSolveCallback(worldId: WorldId, preSolveFn: ?*PreSolveFn, context: ?*anyopaque) void {
    box2d.b2World_SetPreSolveCallback(@bitCast(worldId), @ptrCast(preSolveFn), @ptrCast(context));
}

pub inline fn setGravity(worldId: WorldId, gravity: Vec2) void {
    box2d.b2World_SetGravity(@bitCast(worldId), @bitCast(gravity));
}

pub inline fn getGravity(worldId: WorldId) Vec2 {
    return @bitCast(box2d.b2World_GetGravity(@bitCast(worldId)));
}

pub inline fn explode(worldId: WorldId, position: Vec2, radius: f32, impulse: f32) void {
    box2d.b2World_Explode(@bitCast(worldId), @bitCast(position), radius, impulse);
}

pub inline fn setContactTuning(worldId: WorldId, hertz: f32, dampingRatio: f32, pushVelocity: f32) void {
    box2d.b2World_SetContactTuning(@bitCast(worldId), hertz, dampingRatio, pushVelocity);
}

pub inline fn setJointTuning(worldId: WorldId, hertz: f32, dampingRatio: f32) void {
    box2d.b2World_SetJointTuning(@bitCast(worldId), hertz, dampingRatio);
}

pub inline fn setMaximumLinearVelocity(worldId: WorldId, maximumLinearVelocity: f32) void {
    box2d.b2World_SetMaximumLinearVelocity(@bitCast(worldId), maximumLinearVelocity);
}

pub inline fn getMaximumLinearVelocity(worldId: WorldId) f32 {
    return box2d.b2World_GetMaximumLinearVelocity(@bitCast(worldId));
}

pub inline fn setUserData(worldId: WorldId, userData: ?*anyopaque) void {
    box2d.b2World_SetUserData(@bitCast(worldId), userData);
}

pub inline fn getUserData(worldId: WorldId) ?*anyopaque {
    return box2d.b2World_GetUserData(@bitCast(worldId));
}

pub inline fn getProfile(worldId: WorldId) Profile {
    return @bitCast(box2d.b2World_GetProfile(@bitCast(worldId)));
}

pub inline fn getCounters(worldId: WorldId) Counters {
    return @bitCast(box2d.b2World_GetCounters(@bitCast(worldId)));
}

pub inline fn dumpMemoryStats(worldId: WorldId) void {
    box2d.b2World_DumpMemoryStats(@bitCast(worldId));
}

pub inline fn rebuildStaticTree(worldId: WorldId) void {
    box2d.b2World_RebuildStaticTree(@bitCast(worldId));
}

pub inline fn setCustomFilterCallback(worldId: WorldId, fcn: ?*const CustomFilterFn, context: ?*anyopaque) void {
    box2d.b2World_SetCustomFilterCallback(worldId, @ptrCast(fcn), context);
}

pub inline fn eql(worldId: WorldId, other: WorldId) bool {
    return worldId.index1 == other.index1 and worldId.revision == other.revision;
}

pub inline fn isNull(this: WorldId) bool {
    return this.index1 == 0;
}
