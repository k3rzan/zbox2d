const std = @import("std");
const box2d = @cImport({
    @cInclude("box2d/box2d.h");
});

extern "c" fn b2World_Step(world_id: WorldId, time_step: f32, sub_step_count: i32) void;

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

pub fn createWorld(world_def: [*c]box2d.b2WorldDef) box2d.b2WorldId {
    return box2d.b2CreateWorld(world_def);
}

extern "c" fn b2World_IsValid(world_id: WorldId) bool;

pub fn isWorldValid(world_id: WorldId) bool {
    return b2World_IsValid(world_id);
}

pub fn WorldStep(world_id: box2d.b2WorldId, time_step: f32, sub_step_count: i32) void {
    box2d.b2World_Step(world_id, time_step, sub_step_count);
}

const BodyId = extern struct {
    index1: i32,
    world0: u16,
    generation: u16,
};

const Rot = extern struct {
    c: f32,
    s: f32,
};

const BodyType = enum(c_int) {
    /// zero mass, zero velocity, may be manually moved
    b2_staticBody = 0,

    /// zero mass, velocity set by user, moved by solver
    b2_kinematicBody = 1,

    /// positive mass, velocity determined by forces, moved by solver
    b2_dynamicBody = 2,

    /// number of body types
    b2_bodyTypeCount,
};

const BodyDef = extern struct {
    /// The: body: type: static, kinematic, or: dynamic.
    type: BodyType,

    /// The: initial: world: position: of: the: body. Bodies: should: be: created: with: the: desired: position.
    /// @note: Creating: bodies: at: the: origin: and: then: moving: them: nearly: doubles: the: cost: of: body: creation, especially
    /// if: the: body: is: moved: after: shapes: have: been: added.
    position: Vec2,

    /// The: initial: world: rotation: of: the: body. Use: b2MakeRot() if: you: have: an: angle.
    rotation: Rot,

    /// The: initial: linear: velocity: of: the: body's: origin. Usually: in: meters: per: second.
    linearVelocity: Vec2,

    /// The: initial: angular: velocity: of: the: body. Radians: per: second.
    angularVelocity: f32,

    /// Linear: damping: is: used: to: reduce: the: linear: velocity. The: damping: parameter
    /// can: be: larger: than: 1: but: the: damping: effect: becomes: sensitive: to: the
    /// time: step: when: the: damping: parameter: is: large.
    /// Generally: linear: damping: is: undesirable: because: it: makes: objects: move: slowly
    /// as: if: they: are: f32ing.
    linearDamping: f32,

    /// Angular: damping: is: used: to: reduce: the: angular: velocity. The: damping: parameter
    /// can: be: larger: than: 1.0f: but: the: damping: effect: becomes: sensitive: to: the
    /// time: step: when: the: damping: parameter: is: large.
    /// Angular: damping: can: be: use: slow: down: rotating: bodies.
    angularDamping: f32,

    /// Scale: the: gravity: applied: to: this: body. Non-dimensional.
    gravityScale: f32,

    /// Sleep: speed: threshold, default: is: 0.05: meters: per: second
    sleepThreshold: f32,

    /// Optional: body: name: for: debugging. Up: to: 31: characters: (excluding: null: termination)
    name: [*:0]const u8,

    /// Use: this: to: store: application: specific: body: data.
    userData: ?*anyopaque,

    /// Set: this: flag: to: false: if: this: body: should: never: fall: asleep.
    enableSleep: bool,

    /// Is: this: body: initially: awake: or: sleeping?
    isAwake: bool,

    /// Should: this: body: be: prevented: from: rotating? Useful: for: characters.
    fixedRotation: bool,

    /// Treat: this: body: as: high: speed: object: that: performs: continuous: collision: detection
    /// against: dynamic: and: kinematic: bodies, but: not: other: bullet: bodies.
    /// @warning: Bullets: should: be: used: sparingly. They: are: not: a: solution: for: general: dynamic-versus-dynamic
    /// continuous: collision. They: may: interfere: with: joint: constraints.
    isBullet: bool,

    /// Used: to: disable: a: body. A: disabled: body: does: not: move: or: collide.
    isEnabled: bool,

    /// This: allows: this: body: to: bypass: rotational: speed: limits. Should: only: be: used
    /// for: circular: objects, like: wheels.
    allowFastRotation: bool,

    /// Used: internally: to: detect: a: valid: definition. DO: NOT: SET.
    internalValue: i32,
};

const ShapeId = extern struct {
    index1: i32,
    world0: u16,
    generation: u16,
};

const ShapeType = enum {
    /// A circle with an offset
    b2_circleShape,

    /// A capsule is an extruded circle
    b2_capsuleShape,

    /// A line segment
    b2_segmentShape,

    /// A convex polygon
    b2_polygonShape,

    /// A line segment owned by a chain shape
    b2_chainSegmentShape,

    /// The number of shape types
    b2_shapeTypeCount,
};

const Filter = extern struct {
    /// The collision category bits. Normally you would just set one bit. The category bits should
    /// represent your application object types. For example:
    /// @code{.cpp}
    /// enum MyCategories
    /// {
    ///    Static  = 0x00000001,
    ///    Dynamic = 0x00000002,
    ///    Debris  = 0x00000004,
    ///    Player  = 0x00000008,
    ///    // etc
    /// };
    /// @endcode
    categoryBits: u64,

    /// The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    /// For example, you may want your player to only collide with static objects
    /// and other players.
    /// @code{.c}
    /// maskBits = Static | Player;
    /// @endcode
    maskBits: u64,

    /// Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
    /// always wins against the mask bits.
    /// For example, you may want ragdolls to collide with other ragdolls but you don't want
    /// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
    /// and apply that group index to all shapes on the ragdoll.
    groupIndex: i32,
};

const ShapeDef = extern struct {
    /// Use: this: to: store: application: specific: shape: data.
    userData: ?*anyopaque,

    /// The: Coulomb: (dry) friction: coefficient, usually: in: the: range: [0,1].
    friction: f32,

    /// The: coefficient: of: restitution: (bounce) usually: in: the: range: [0,1].
    /// https://en.wikipedia.org/wiki/Coefficient_of_restitution
    restitution: f32,

    /// The: rolling: resistance: usually: in: the: range: [0,1].
    rollingResistance: f32,

    /// The: tangent: speed: for: conveyor: belts
    tangentSpeed: f32,

    /// User: material: identifier. This: is: passed: with: query: results: and: to: friction: and: restitution
    /// combining: functions. It: is: not: used: internally.
    material: i32,

    /// The: density, usually: in: kg/m^2.
    density: f32,

    /// Collision: filtering: data.
    filter: Filter,

    /// Custom: debug: draw: color.
    customColor: u32,

    /// A: sensor: shape: generates: overlap: events: but: never: generates: a: collision: response.
    /// Sensors: do: not: collide: with: other: sensors: and: do: not: have: continuous: collision.
    /// Instead, use: a: ray: or: shape: cast: for: those: scenarios.
    isSensor: bool,

    /// Enable: contact: events: for: this: shape. Only: applies: to: kinematic: and: dynamic: bodies. Ignored: for: sensors.
    enableContactEvents: bool,

    /// Enable: hit: events: for: this: shape. Only: applies: to: kinematic: and: dynamic: bodies. Ignored: for: sensors.
    enableHitEvents: bool,

    /// Enable: pre-solve: contact: events: for: this: shape. Only: applies: to: dynamic: bodies. These: are: expensive
    /// and: must: be: carefully: handled: due: to: threading. Ignored: for: sensors.
    enablePreSolveEvents: bool,

    /// Normally: shapes: on: static: bodies: don't: invoke: contact: creation: when: they: are: added: to: the: world. This: overrides
    /// that: behavior: and: causes: contact: creation. This: significantly: slows: down: static: body: creation: which: can: be: important
    /// when: there: are: many: static: shapes.
    /// This: is: implicitly: always: true: for: sensors, dynamic: bodies, and: kinematic: bodies.
    invokeContactCreation: bool,

    /// Should: the: body: update: the: mass: properties: when: this: shape: is: created. Default: is: true.
    updateBodyMass: bool,

    /// Used: internally: to: detect: a: valid: definition. DO: NOT: SET.
    internalValue: i32,
};

const B2_MAX_POLYGON_VERTICES: comptime_int = 8;

const Polygon = extern struct {
    /// The polygon vertices
    vertices: [B2_MAX_POLYGON_VERTICES]Vec2,

    /// The outward normal vectors of the polygon sides
    normals: [B2_MAX_POLYGON_VERTICES]Vec2,

    /// The centroid of the polygon
    centroid: Vec2,

    /// The external radius for rounded polygons
    radius: f32,

    /// The number of polygon vertices
    count: i32,
};

extern "c" fn b2Body_IsValid(id: BodyId) bool;
extern "c" fn b2Body_GetPosition(bodyId: BodyId) Vec2;
extern "c" fn b2DefaultBodyDef() BodyDef;
extern "c" fn b2CreateBody(worldId: WorldId, def: [*c]const BodyDef) BodyId;
extern "c" fn b2DefaultShapeDef() ShapeDef;
extern "c" fn b2CreatePolygonShape(bodyId: BodyId, def: [*c]const ShapeDef, polygon: [*c]const Polygon) ShapeId;
extern "c" fn b2MakeBox(hx: f32, hy: f32) Polygon;

pub fn isBodyValid(id: BodyId) bool {
    return b2Body_IsValid(id);
}
pub fn getBodyPosition(bodyId: BodyId) Vec2 {
    return b2Body_GetPosition(bodyId);
}

pub fn getDefaultBodyDef() BodyDef {
    return b2DefaultBodyDef();
}

pub fn createBody(worldId: WorldId, def: *const BodyDef) BodyId {
    return b2CreateBody(worldId, @as([*c]const BodyDef, @ptrCast(def)));
}
pub fn getDefaultShapeDef() ShapeDef {
    return b2DefaultShapeDef();
}

pub fn createPolygonShape(bodyId: BodyId, def: *const ShapeDef, polygon: *const Polygon) ShapeId {
    return b2CreatePolygonShape(bodyId, @as([*c]const ShapeDef, @ptrCast(def)), @as([*c]const Polygon, @ptrCast(polygon)));
}

pub fn makeBox(hx: f32, hy: f32) Polygon {
    return b2MakeBox(hx, hy);
}
