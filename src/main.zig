const std = @import("std");
const box2d = @cImport({
    @cInclude("box2d/box2d.h");
});

const b2FinishTaskCallback = fn (userTask: ?*anyopaque, userContext: ?*anyopaque) callconv(.C) void;
const b2TaskCallback = fn (i32, i32, u32, ?*anyopaque) callconv(.C) void;
const b2EnqueueTaskCallback = fn (?*const b2TaskCallback, i32, i32, ?*anyopaque, ?*anyopaque) callconv(.C) ?*anyopaque;

const b2RestitutionCallback = fn (restitutionA: f32, materialA: i32, restitutionB: f32, materialB: i32) callconv(.C) f32;

const b2FrictionCallback = fn (frictionA: f32, materialA: i32, frictionB: i32, materialB: i32) callconv(.C) f32;

pub const Vec2 = extern struct {
    x: f32,
    y: f32,
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

const BodyId = extern struct {
    index1: i32,
    world0: u16,
    revision: u16,
};

const Rot = extern struct {
    c: f32,
    s: f32,
};

const BodyType = enum(c_int) {
    b2_staticBody = 0,
    b2_kinematicBody = 1,
    b2_dynamicBody = 2,
    b2_bodyTypeCount,
};

const BodyDef = extern struct {
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

const ShapeId = extern struct {
    index1: i32,
    world0: u16,
    revision: u16,
};

const ShapeType = enum {
    b2_circleShape,
    b2_capsuleShape,
    b2_segmentShape,
    b2_polygonShape,
    b2_smoothSegmentShape,
    b2_shapeTypeCount,
};

const Filter = extern struct {
    categoryBits: u32,
    maskBits: u32,
    groupIndex: i32,
};

const ShapeDef = extern struct {
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

const B2_MAX_POLYGON_VERTICES: comptime_int = 8;

const Polygon = extern struct {
    vertices: [B2_MAX_POLYGON_VERTICES]Vec2,
    normals: [B2_MAX_POLYGON_VERTICES]Vec2,
    centroid: Vec2,
    radius: f32,
    count: i32,
};

pub inline fn isBodyValid(id: BodyId) bool {
    return @bitCast(box2d.b2Body_IsValid(@bitCast(id)));
}
pub inline fn getBodyPosition(bodyId: BodyId) Vec2 {
    return @bitCast(box2d.b2Body_GetPosition(@bitCast(bodyId)));
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
