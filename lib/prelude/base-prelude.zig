const std = @import("std");

const b2Version = struct {
    /// Significant changes
    major: i32,

    /// Incremental changes
    minor: i32,

    /// Bug fixes
    revision: i32,
};
