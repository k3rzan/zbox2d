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

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});

    const optimize = b.standardOptimizeOption(.{});

    const zbox2d_mod = b.addModule("zbox2d", .{
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    const lib = b.addStaticLibrary(.{
        .name = "zbox2d",
        .link_libc = true,
        .root_source_file = b.path("src/main.zig"),
        .optimize = optimize,
        .target = target,
    });

    const box2d_dep = b.dependency("box2d", .{ .target = target, .optimize = optimize });

    for (box2d_include_paths) |include_path| {
        zbox2d_mod.addIncludePath(box2d_dep.path(include_path));
        lib.addIncludePath(box2d_dep.path(include_path));
    }
    lib.addCSourceFiles(.{
        .root = box2d_dep.path("src"),
        .files = box2d_source_files,
    });

    zbox2d_mod.link_libc = true;
    zbox2d_mod.linkLibrary(lib);

    b.installArtifact(lib);
}
