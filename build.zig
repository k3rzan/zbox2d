const std = @import("std");

const box2d_include_paths = &[_][]const u8{
    "src",
    "include",
    "extern/glad",
    "extern/jsmn",
    "extern/simde",
};
const box2d_source_files = &[_][]const u8{
    "aabb.c",
    "allocate.c",
    "array.c",
    "bitset.c",
    "block_array.c",
    "body.c",
    "broad_phase.c",
    "constraint_graph.c",
    "contact.c",
    "contact_solver.c",
    "core.c",
    "distance.c",
    "distance_joint.c",
    "dynamic_tree.c",
    "geometry.c",
    "hull.c",
    "id_pool.c",
    "island.c",
    "joint.c",
    "manifold.c",
    "math_functions.c",
    "motor_joint.c",
    "mouse_joint.c",
    "prismatic_joint.c",
    "revolute_joint.c",
    "shape.c",
    "solver.c",
    "solver_set.c",
    "stack_allocator.c",
    "table.c",
    "timer.c",
    "types.c",
    "weld_joint.c",
    "wheel_joint.c",
    "world.c",
};

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    const zbox2d_mod = b.addModule("zbox2d", .{
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    const lib = b.addStaticLibrary(.{
        .name = "zbox2d",
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    const box2d_dep = b.dependency("box2d", .{ .target = target, .optimize = optimize });

    // const box2d_mod = b.addModule("box2d", .{
    //     .target = target,
    //     .optimize = optimize,
    //     .link_libc = true,
    // });

    for (box2d_include_paths) |include_path| {
        lib.addIncludePath(box2d_dep.path(include_path));
    }
    for (box2d_source_files) |file| {
        lib.addCSourceFile(.{
            .file = box2d_dep.path(b.pathJoin(&.{ "src", file })),
        });
    }

    for (box2d_include_paths) |include_dir| {
        zbox2d_mod.addIncludePath(b.path(include_dir));
    }

    lib.linkLibC();
    b.installArtifact(lib);

    zbox2d_mod.linkLibrary(lib);

    const demo = b.addExecutable(.{
        .name = "demo",
        .root_source_file = b.path("src/test.zig"),
        .target = target,
        .optimize = optimize,
    });

    demo.root_module.addImport("zbox2d", zbox2d_mod);
    demo.linkLibrary(lib);

    if (b.option(bool, "enable-demo", "install demo exe") orelse false) {
        b.installArtifact(demo);
    }
}
