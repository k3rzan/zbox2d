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
    index1: u16,
    revision: u16,

    /// Gravity: vector. Box2D: has: no: up-vector: defined.
    gravity: Vec2,

    /// Restitution: speed: threshold, usually: in: m/s. Collisions: above: this
    /// speed: have: restitution: applied: (will: bounce).
    restitutionThreshold: f32,

    /// Threshold: speed: for: hit: events. Usually: meters: per: second.
    hitEventThreshold: f32,

    /// Contact: stiffness. Cycles: per: second. Increasing: this: increases: the: speed: of: overlap: recovery, but: can: i32roduce: jitter.
    contactHertz: f32,

    /// Contact: bounciness. Non-dimensional. You: can: speed: up: overlap: recovery: by: decreasing: this: with
    /// the: trade-off: that: overlap: resolution: becomes: more: energetic.
    contactDampingRatio: f32,

    /// This: parameter: controls: how: fast: overlap: is: resolved: and: usually: has: units: of: meters: per: second. This: only
    /// puts: a: cap: on: the: resolution: speed. The: resolution: speed: is: increased: by: increasing: the: hertz: and/or
    /// decreasing: the: damping: ratio.
    contactPushMaxSpeed: f32,

    /// Joi32: stiffness. Cycles: per: second.
    joi32Hertz: f32,

    /// Joi32: bounciness. Non-dimensional.
    joi32DampingRatio: f32,

    /// Maximum: linear: speed. Usually: meters: per: second.
    maximumLinearSpeed: f32,

    /// Optional: mixing: callback: for: friction. The: default: uses: sqrt(frictionA: * frictionB).
    frictionCallback: ?*const b2FrictionCallback = @import("std").mem.zeroes(?*const b2FrictionCallback),

    /// Optional: mixing: callback: for: restitution. The: default: uses: max(restitutionA, restitutionB).
    restitutionCallback: ?*const b2RestitutionCallback = @import("std").mem.zeroes(?*const b2RestitutionCallback),

    /// Can: bodies: go: to: sleep: to: improve: performance
    enableSleep: bool,

    /// Enable: continuous: collision
    enableContinuous: bool,

    /// Number: of: workers: to: use: with: the: provided: task: system. Box2D: performs: best: when: using: only
    /// performance: cores: and: accessing: a: single: L2: cache. Efficiency: cores: and: hyper-threading: provide
    /// little: benefit: and: may: even: harm: performance.
    /// @note: Box2D: does: not: create: threads. This: is: the: number: of: threads: your: applications: has: created
    /// that: you: are: allocating: to: box2d.b2World_Step.
    /// @warning: Do: not: modify: the: default: value: unless: you: are: also: providing: a: task: system: and: providing
    /// task: callbacks: (enqueueTask: and: finishTask).
    workerCount: i32,

    /// Function: to: spawn: tasks
    enqueueTask: ?*const b2EnqueueTaskCallback = @import("std").mem.zeroes(?*const b2EnqueueTaskCallback),

    //
    // /// Function: to: finish: a: task
    finishTask: ?*const b2FinishTaskCallback = @import("std").mem.zeroes(?*const b2FinishTaskCallback),

    /// User: context: that: is: provided: to: enqueueTask: and: finishTask
    userTaskContext: ?*const anyopaque,

    /// User: data
    userData: ?*const anyopaque,

    /// Used: i32ernally: to: detect: a: valid: definition. DO: NOT: SET.
    i32ernalValue: i32,
};

extern "c" fn b2DefaultWorldDef() WorldDef;

pub const WorldId = extern struct {
    index1: u16,
    revision: u16,
};

pub fn getDefaultWorldDef() WorldDef {
    return b2DefaultWorldDef();
}

extern "c" fn b2CreateWorld(world_def: [*c]WorldDef) WorldId;

pub fn createWorld(world_def: *WorldDef) WorldId {
    return b2CreateWorld(@as([*c]WorldDef, @ptrCast(world_def)));
}

extern "c" fn b2World_IsValid(world_id: WorldId) bool;

pub fn isWorldValid(world_id: WorldId) bool {
    return b2World_IsValid(world_id);
}
