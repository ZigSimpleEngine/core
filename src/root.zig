const std = @import("std");

/// ECS-style transform component module, independent of graphics API.
///
/// Provides `Transform(Scalar)` — a plain value `struct` storing only
/// position, rotation and scale. No hierarchy, no cached matrix, no
/// allocation: matrices are built on demand via `toMatrix`/`toWorldMatrix`,
/// and any parent world matrix is passed explicitly.
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

test "transform ecs basic" {
    const T = Transform(f32);
    const V3 = math.Vec(3, f32);

    var t = T.identity();
    t.position = V3.init(.{ 1, 0, 0 });

    // Without a parent, the world matrix with an identity parent matches local.
    const no_parent = T.identity().toMatrix();
    try std.testing.expectEqual(t.toMatrix(), t.toWorldMatrix(no_parent));

    // Identity rotation: local axes match the canonical basis.
    try std.testing.expectEqual(@as(f32, 1), t.rightLocal().v[0]);
    try std.testing.expectEqual(@as(f32, 0), t.rightLocal().v[1]);
    try std.testing.expectEqual(@as(f32, 1), t.upLocal().v[1]);
    try std.testing.expectEqual(@as(f32, -1), t.forwardLocal().v[2]);

    // Point at local origin lands on the position.
    const p0 = t.transformPointLocal(V3.zero());
    try std.testing.expectEqual(@as(f32, 1), p0.v[0]);
    try std.testing.expectEqual(@as(f32, 0), p0.v[1]);
    try std.testing.expectEqual(@as(f32, 0), p0.v[2]);

    // Explicit parent world matrix: parent shifted by (0, 1, 0).
    var parent = T.identity();
    parent.position = V3.init(.{ 0, 1, 0 });
    const parent_world = parent.toMatrix();
    const pw = t.transformPointWorld(parent_world, V3.zero());
    try std.testing.expectEqual(@as(f32, 1), pw.v[0]);
    try std.testing.expectEqual(@as(f32, 1), pw.v[1]);
    try std.testing.expectEqual(@as(f32, 0), pw.v[2]);

    // Explicit parent rotation: parent yawed +90 deg about Y turns child forward (-Z) into (-X).
    var yawed = T.identity();
    yawed.rotation = math.quat.angleAxis(@as(f32, std.math.pi) / @as(f32, 2), V3.unit(1));
    const yawed_world = yawed.toMatrix();
    const fwd = T.identity().forwardWorld(yawed_world);
    try std.testing.expectApproxEqAbs(@as(f32, -1), fwd.v[0], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), fwd.v[1], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), fwd.v[2], 1e-5);
}

test "transform public helpers" {
    const T = Transform(f32);
    const V3 = math.Vec(3, f32);

    // safeNormalize: zero stays zero instead of NaN.
    const zn = T.safeNormalize(V3.zero());
    try std.testing.expectEqual(@as(f32, 0), zn.v[0]);
    try std.testing.expectEqual(@as(f32, 0), zn.v[1]);
    try std.testing.expectEqual(@as(f32, 0), zn.v[2]);

    // fromToRotation: parallel vectors give identity.
    const q_ident = T.fromToRotation(V3.unit(0), V3.unit(0));
    try std.testing.expectApproxEqAbs(@as(f32, 1), q_ident.w, 1e-6);

    // fromToRotation: +X to +Y rotates right into up.
    const q = T.fromToRotation(V3.unit(0), V3.unit(1));
    const rotated = q.mulVec3(V3.unit(0));
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.v[0], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), rotated.v[1], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.v[2], 1e-5);

    // translation/basis round-trip through toMatrix.
    var t = T.identity();
    t.position = V3.init(.{ 1, 2, 3 });
    const back_pos = T.translation(t.toMatrix());
    try std.testing.expectEqual(@as(f32, 1), back_pos.v[0]);
    try std.testing.expectEqual(@as(f32, 2), back_pos.v[1]);
    try std.testing.expectEqual(@as(f32, 3), back_pos.v[2]);
    const b = T.basis(t.toMatrix());
    try std.testing.expectEqual(@as(f32, 1), b[0].v[0]);
    try std.testing.expectEqual(@as(f32, 1), b[1].v[1]);
    try std.testing.expectEqual(@as(f32, 1), b[2].v[2]);

    // rotationFromMatrix: identity matrix gives identity rotation.
    const r_ident = T.rotationFromMatrix(T.identity().toMatrix());
    try std.testing.expectApproxEqAbs(@as(f32, 1), r_ident.w, 1e-5);

    // setAxisLocal: align an arbitrary axis (+X) to +Y.
    var a = T.identity();
    a.setAxisLocal(V3.unit(0), V3.unit(1));
    const aligned = a.rightLocal();
    try std.testing.expectApproxEqAbs(@as(f32, 0), aligned.v[0], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), aligned.v[1], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), aligned.v[2], 1e-5);

    // setAxisWorld with identity parent matches the local variant.
    var w = T.identity();
    w.setAxisWorld(T.identity().toMatrix(), V3.unit(0), V3.unit(1));
    const aligned_w = w.rightWorld(T.identity().toMatrix());
    try std.testing.expectApproxEqAbs(@as(f32, 0), aligned_w.v[0], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1), aligned_w.v[1], 1e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 0), aligned_w.v[2], 1e-5);
}
