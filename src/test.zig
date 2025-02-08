const std = @import("std");
const zbox2d = @import("zbox2d");

pub fn main() !void {
    const world = zbox2d.getWorldDefaultDef();
    std.debug.print("demo file\n", .{world});
}
