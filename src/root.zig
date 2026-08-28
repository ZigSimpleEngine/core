const std = @import("std");

/// Hierarchical transform module, independent of graphics API.
///
/// Provides `Transform(Scalar)` — an opaque type storing position, rotation, scale
/// and parent-child hierarchy with cached world matrix.
pub const Transform = @import("transform.zig").Transform;

/// Global typed registry by comptime name.
///
/// `Map(name, T)` stores a single value `T` accessible via static `get`/`set`.
pub const Map = @import("map.zig").Map;

/// Re-export of math library for convenience of `core` consumers.
pub const math = @import("math");

test {
    std.testing.refAllDecls(@This());
}

test "map get/set" {
    const MyMap = Map(.test_map, i32);
    MyMap.set(42);
    try std.testing.expectEqual(@as(i32, 42), MyMap.get(0));
}

test "transform hierarchy basic" {
    const T = Transform(f32);
    const gpa = std.testing.allocator;
    const root = try T.create(gpa);
    defer root.destroy(gpa);
    const child = try T.create(gpa);
    defer child.destroy(gpa);
    child.position().* = math.Vec(3, f32).init(.{ 1, 0, 0 });
    try root.addChild(gpa, child);
    try std.testing.expectEqual(@as(usize, 1), root.getChildrenCount());
    try std.testing.expect(child.getParent() == root);
    root.recalculateTransformMatricesDownward();
    _ = root.getMatrix();
    _ = child.getMatrix();
}
