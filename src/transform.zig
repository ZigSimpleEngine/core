const std = @import("std");
const math = @import("math");

/// Creates an ECS-style 3D transform component parameterized by scalar type.
///
/// Plain value `struct` holding only `position`, `rotation` and `scale` —
/// no hierarchy, no cached matrix, no heap allocation. Designed for direct
/// storage in ECS archetypes/tables and manipulation by value or pointer.
///
/// Matrices are never stored: `toMatrix` builds the local matrix and
/// `toWorldMatrix` combines it with an explicitly passed parent world
/// matrix; both return the result by value.
///
/// Queries come in two flavors with no runtime branching between them:
/// `*Local` variants work on the local transform only and take no matrix,
/// while `*World` variants take an explicit `parent_world: Mat4` (pass
/// `Mat4.identity()` when there is no parent).
///
/// Parameters:
/// - `scalar_type_` — scalar type for vector/quaternion/matrix components (`f32`, `f64`, etc.).
///
/// Returns: `struct` type `Transform(Scalar)` usable as an ECS component.
pub fn Transform(comptime scalar_type_: type) type {
    return struct {
        const Self = @This();

        /// Scalar type of the transform, as specified at specialization.
        pub const Scalar = scalar_type_;

        /// Three-component vector based on `Scalar`.
        pub const Vec3 = math.Vec(3, Scalar);

        /// Quaternion based on `Scalar`, representing rotation.
        pub const QuatT = math.Quat(Scalar);

        /// 4x4 matrix based on `Scalar`, representing the final transform.
        pub const Mat4 = math.Mat(4, 4, Scalar);

        /// Local position.
        position: Vec3 = Vec3.zero(),

        /// Local rotation as quaternion.
        rotation: QuatT = .{},

        /// Local scale per axis.
        scale: Vec3 = Vec3.one(),

        /// Creates a transform with given position, rotation and scale.
        ///
        /// Parameters:
        /// - `pos` — initial local position.
        /// - `rot` — initial rotation.
        /// - `sc` — initial scale.
        ///
        /// Returns: the transform value (no allocation).
        pub inline fn init(pos: Vec3, rot: QuatT, sc: Vec3) Self {
            return .{ .position = pos, .rotation = rot, .scale = sc };
        }

        /// Creates an identity transform with scale `1` and zero position.
        ///
        /// Returns: the identity transform value.
        pub inline fn identity() Self {
            return .{};
        }

        /// Builds the local matrix from `position`/`rotation`/`scale`.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        ///
        /// Returns: local `Mat4` matrix (`translate * rotate * scale`).
        pub fn toMatrix(self: Self) Mat4 {
            var mat = Mat4.identity();
            mat = mat.translate(self.position);
            const rot = math.quat.mat4_cast(self.rotation);
            mat = mat.mul(rot);
            mat = mat.scale(self.scale);
            return mat;
        }

        /// Builds the world matrix, combining the local matrix with an
        /// explicitly passed parent world matrix.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: `parent_world * local`.
        pub fn toWorldMatrix(self: Self, parent_world: Mat4) Mat4 {
            return parent_world.mul(self.toMatrix());
        }

        /// Orients the transform to look at a target with world up.
        ///
        /// Parameters:
        /// - `self` — pointer to the object to transform.
        /// - `target` — point to look at, in the same space as `position`.
        /// - `world_up` — up vector for orientation stability.
        pub fn lookAt(self: *Self, target: Vec3, world_up: Vec3) void {
            const direction = target.sub(self.position).normalize();
            self.rotation = QuatT.lookAt(direction, world_up);
        }

        /// Extracts the rotation part of an explicitly passed matrix
        /// (columns are normalized first, so uniform scale is ignored).
        ///
        /// Useful on its own to decompose a world matrix, e.g. to convert
        /// a world direction into the space of a parent transform.
        ///
        /// Parameters:
        /// - `matrix` — matrix to extract the rotation from, e.g. a parent world matrix.
        ///
        /// Returns: rotation as a normalized quaternion.
        pub fn rotationFromMatrix(matrix: Mat4) QuatT {
            const c0 = safeNormalize(Vec3.init(.{ matrix.data[0].v[0], matrix.data[0].v[1], matrix.data[0].v[2] }));
            const c1 = safeNormalize(Vec3.init(.{ matrix.data[1].v[0], matrix.data[1].v[1], matrix.data[1].v[2] }));
            const c2 = safeNormalize(Vec3.init(.{ matrix.data[2].v[0], matrix.data[2].v[1], matrix.data[2].v[2] }));
            const m3 = math.Mat(3, 3, Scalar).init(.{ c0, c1, c2 });
            return math.quat.quat_cast(m3).normalize();
        }

        /// Computes the world rotation from the local rotation and an
        /// explicitly passed parent world matrix.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: normalized world quaternion.
        pub inline fn rotationWorld(self: Self, parent_world: Mat4) QuatT {
            return rotationFromMatrix(parent_world).mul(self.rotation).normalize();
        }

        /// Rotates a local axis by the local rotation only.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `axis` — local axis.
        ///
        /// Returns: local axis as `Vec3`.
        inline fn localAxis(self: Self, axis: Vec3) Vec3 {
            return self.rotation.mulVec3(axis);
        }

        /// Rotates a local axis to a world direction via `rotationWorld`.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `axis` — local axis.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: world axis as `Vec3`.
        inline fn worldAxis(self: Self, axis: Vec3, parent_world: Mat4) Vec3 {
            return self.rotationWorld(parent_world).mulVec3(axis);
        }

        /// Extracts the scaled basis (first three columns) of an explicitly
        /// passed matrix.
        ///
        /// Note: scale is baked into the basis vectors — use their lengths
        /// to recover per-axis scale when decomposing a matrix.
        ///
        /// Parameters:
        /// - `matrix` — matrix to extract the basis from, e.g. from `toMatrix`.
        ///
        /// Returns: array of three `Vec3` basis vectors.
        pub inline fn basis(matrix: Mat4) [3]Vec3 {
            return .{
                Vec3.init(.{ matrix.data[0].v[0], matrix.data[0].v[1], matrix.data[0].v[2] }),
                Vec3.init(.{ matrix.data[1].v[0], matrix.data[1].v[1], matrix.data[1].v[2] }),
                Vec3.init(.{ matrix.data[2].v[0], matrix.data[2].v[1], matrix.data[2].v[2] }),
            };
        }

        /// Extracts the translation (fourth column) of an explicitly passed
        /// matrix.
        ///
        /// Handy to read back the world position from an already computed
        /// world matrix without keeping the transform around.
        ///
        /// Parameters:
        /// - `matrix` — matrix to extract the translation from, e.g. from `toMatrix`.
        ///
        /// Returns: translation as `Vec3`.
        pub inline fn translation(matrix: Mat4) Vec3 {
            return Vec3.init(.{ matrix.data[3].v[0], matrix.data[3].v[1], matrix.data[3].v[2] });
        }

        /// Safely normalizes a vector, returning zero for degenerate inputs.
        ///
        /// Unlike a plain division by length, never produces NaN/inf:
        /// use it for directions computed from arbitrary input
        /// (e.g. `target - position` when both may coincide).
        ///
        /// Parameters:
        /// - `v` — input vector to normalize.
        ///
        /// Returns: normalized vector or zero if length < 1e-9.
        pub inline fn safeNormalize(v: Vec3) Vec3 {
            const len = v.length();
            if (len <= @as(Scalar, 1e-9)) return Vec3.zero();
            return v.mul(@as(Scalar, 1) / len);
        }

        /// Builds a quaternion rotating vector `from` to `to`.
        ///
        /// Handles collinear cases (parallel and opposite directions).
        /// Useful on its own for aiming, alignment and procedural animation.
        ///
        /// Parameters:
        /// - `from` — source direction.
        /// - `to` — target direction.
        ///
        /// Returns: quaternion of shortest rotation.
        pub fn fromToRotation(from: Vec3, to: Vec3) QuatT {
            const a = safeNormalize(from);
            const b = safeNormalize(to);
            const d = a.dot(b);
            if (d > 0.99999) return QuatT.identity();
            if (d < -0.99999) {
                var axis = Vec3.unit(0);
                if (a.dot(axis) > 0.99) axis = Vec3.unit(1);
                axis = a.cross(axis).normalize();
                return math.quat.angleAxis(@as(Scalar, std.math.pi), axis);
            }
            const axis = a.cross(b).normalize();
            const angle = math.scalar.acos(math.scalar.clamp(d, @as(Scalar, -1), @as(Scalar, 1)));
            return math.quat.angleAxis(angle, axis);
        }

        /// Sets local rotation so that an arbitrary local axis aligns with
        /// a direction.
        ///
        /// Generic form of `setRightLocal`/`setUpLocal`/`setForwardLocal`
        /// for models whose meaningful axis is not one of the canonical ones
        /// (e.g. a barrel along +Y).
        ///
        /// Parameters:
        /// - `self` — pointer to the transform to modify.
        /// - `local_axis` — local axis to align.
        /// - `dir` — target direction in the same space as the rotation.
        pub fn setAxisLocal(self: *Self, local_axis: Vec3, dir: Vec3) void {
            self.rotation = fromToRotation(local_axis, dir);
        }

        /// Sets local rotation so that an arbitrary local axis aligns with
        /// a world direction, compensating the explicitly passed parent
        /// world matrix.
        ///
        /// Generic form of `setRightWorld`/`setUpWorld`/`setForwardWorld`.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform to modify.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `local_axis` — local axis to align.
        /// - `world_dir` — target world direction.
        pub fn setAxisWorld(self: *Self, parent_world: Mat4, local_axis: Vec3, world_dir: Vec3) void {
            const parent_rot = rotationFromMatrix(parent_world);
            const dir_local = parent_rot.conjugate().mulVec3(world_dir);
            self.rotation = fromToRotation(local_axis, dir_local);
        }

        /// Returns local right vector (local +X).
        ///
        /// Parameters:
        /// - `self` — the transform value.
        ///
        /// Returns: local right vector.
        pub inline fn rightLocal(self: Self) Vec3 {
            return self.localAxis(Vec3.unit(0));
        }

        /// Returns world right vector (local +X rotated to world).
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: world right vector.
        pub inline fn rightWorld(self: Self, parent_world: Mat4) Vec3 {
            return self.worldAxis(Vec3.unit(0), parent_world);
        }

        /// Returns local left vector, opposite of local right.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        ///
        /// Returns: left vector.
        pub inline fn leftLocal(self: Self) Vec3 {
            return self.rightLocal().neg();
        }

        /// Returns world left vector, opposite of world right.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: left vector.
        pub inline fn leftWorld(self: Self, parent_world: Mat4) Vec3 {
            return self.rightWorld(parent_world).neg();
        }

        /// Returns local up vector (local +Y).
        ///
        /// Parameters:
        /// - `self` — the transform value.
        ///
        /// Returns: local up vector.
        pub inline fn upLocal(self: Self) Vec3 {
            return self.localAxis(Vec3.unit(1));
        }

        /// Returns world up vector (local +Y rotated to world).
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: world up vector.
        pub inline fn upWorld(self: Self, parent_world: Mat4) Vec3 {
            return self.worldAxis(Vec3.unit(1), parent_world);
        }

        /// Returns local down vector, opposite of local up.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        ///
        /// Returns: down vector.
        pub inline fn downLocal(self: Self) Vec3 {
            return self.upLocal().neg();
        }

        /// Returns world down vector, opposite of world up.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: down vector.
        pub inline fn downWorld(self: Self, parent_world: Mat4) Vec3 {
            return self.upWorld(parent_world).neg();
        }

        /// Returns local forward vector (local -Z).
        ///
        /// Parameters:
        /// - `self` — the transform value.
        ///
        /// Returns: local forward vector.
        pub inline fn forwardLocal(self: Self) Vec3 {
            return self.localAxis(Vec3.unit(2).neg());
        }

        /// Returns world forward vector (local -Z rotated to world).
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: world forward vector.
        pub inline fn forwardWorld(self: Self, parent_world: Mat4) Vec3 {
            return self.worldAxis(Vec3.unit(2).neg(), parent_world);
        }

        /// Returns local backward vector, opposite of local forward.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        ///
        /// Returns: backward vector.
        pub inline fn backwardLocal(self: Self) Vec3 {
            return self.forwardLocal().neg();
        }

        /// Returns world backward vector, opposite of world forward.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        ///
        /// Returns: backward vector.
        pub inline fn backwardWorld(self: Self, parent_world: Mat4) Vec3 {
            return self.forwardWorld(parent_world).neg();
        }

        /// Aligns local +X axis to a direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target right direction in local space.
        pub inline fn setRightLocal(self: *Self, dir: Vec3) void {
            self.setAxisLocal(Vec3.unit(0), dir);
        }

        /// Aligns local +X axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dir` — target world right direction.
        pub inline fn setRightWorld(self: *Self, parent_world: Mat4, dir: Vec3) void {
            self.setAxisWorld(parent_world, Vec3.unit(0), dir);
        }

        /// Aligns local -X axis to a direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target left direction in local space.
        pub inline fn setLeftLocal(self: *Self, dir: Vec3) void {
            self.setRightLocal(dir.neg());
        }

        /// Aligns local -X axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dir` — target world left direction.
        pub inline fn setLeftWorld(self: *Self, parent_world: Mat4, dir: Vec3) void {
            self.setRightWorld(parent_world, dir.neg());
        }

        /// Aligns local +Y axis to a direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target up direction in local space.
        pub inline fn setUpLocal(self: *Self, dir: Vec3) void {
            self.setAxisLocal(Vec3.unit(1), dir);
        }

        /// Aligns local +Y axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dir` — target world up direction.
        pub inline fn setUpWorld(self: *Self, parent_world: Mat4, dir: Vec3) void {
            self.setAxisWorld(parent_world, Vec3.unit(1), dir);
        }

        /// Aligns local -Y axis to a direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target down direction in local space.
        pub inline fn setDownLocal(self: *Self, dir: Vec3) void {
            self.setUpLocal(dir.neg());
        }

        /// Aligns local -Y axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dir` — target world down direction.
        pub inline fn setDownWorld(self: *Self, parent_world: Mat4, dir: Vec3) void {
            self.setUpWorld(parent_world, dir.neg());
        }

        /// Aligns local -Z axis to a forward direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target forward direction in local space.
        pub inline fn setForwardLocal(self: *Self, dir: Vec3) void {
            self.setAxisLocal(Vec3.unit(2).neg(), dir);
        }

        /// Aligns local -Z axis to a world forward direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dir` — target world forward direction.
        pub inline fn setForwardWorld(self: *Self, parent_world: Mat4, dir: Vec3) void {
            self.setAxisWorld(parent_world, Vec3.unit(2).neg(), dir);
        }

        /// Aligns local +Z axis to a backward direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target backward direction in local space.
        pub inline fn setBackwardLocal(self: *Self, dir: Vec3) void {
            self.setForwardLocal(dir.neg());
        }

        /// Aligns local +Z axis to a world backward direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dir` — target world backward direction.
        pub inline fn setBackwardWorld(self: *Self, parent_world: Mat4, dir: Vec3) void {
            self.setForwardWorld(parent_world, dir.neg());
        }

        /// Translates the transform in its local basis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `offset` — offset in local axes (x — right, y — up, z — forward).
        pub fn translateLocal(self: *Self, offset: Vec3) void {
            const moved = self.rightLocal().mul(offset.v[0])
                .add(self.upLocal().mul(offset.v[1]))
                .add(self.forwardLocal().mul(offset.v[2]));
            self.position.addSelf(moved);
        }

        /// Translates the transform in its world basis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `offset` — offset in world axes (x — right, y — up, z — forward).
        pub fn translateWorld(self: *Self, parent_world: Mat4, offset: Vec3) void {
            const moved = self.rightWorld(parent_world).mul(offset.v[0])
                .add(self.upWorld(parent_world).mul(offset.v[1]))
                .add(self.forwardWorld(parent_world).mul(offset.v[2]));
            self.position.addSelf(moved);
        }

        /// Rotates the transform by Euler angles in local space.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `euler` — Euler angles (radians) per axis.
        pub fn rotate(self: *Self, euler: Vec3) void {
            self.rotation = self.rotation.mul(QuatT.fromEuler(euler)).normalize();
        }

        /// Rotates the transform around a point in the same space as `position`.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `point` — center point of rotation.
        /// - `axis` — rotation axis.
        /// - `angle` — rotation angle in radians.
        pub fn rotateAround(self: *Self, point: Vec3, axis: Vec3, angle: Scalar) void {
            const q = math.quat.angleAxis(angle, safeNormalize(axis));
            const dif = q.mulVec3(self.position.sub(point));
            self.position = point.add(dif);
            self.rotation = q.mul(self.rotation).normalize();
        }

        /// Transforms a point from local space with scale and translation.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `v` — local point.
        ///
        /// Returns: transformed point.
        pub fn transformPointLocal(self: Self, v: Vec3) Vec3 {
            const local = self.toMatrix();
            const b = basis(local);
            const p = translation(local);
            return b[0].mul(v.v[0]).add(b[1].mul(v.v[1])).add(b[2].mul(v.v[2])).add(p);
        }

        /// Transforms a point from local to world space with scale and
        /// translation, using an explicitly passed parent world matrix.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `v` — local point.
        ///
        /// Returns: world point.
        pub fn transformPointWorld(self: Self, parent_world: Mat4, v: Vec3) Vec3 {
            const world = self.toWorldMatrix(parent_world);
            const b = basis(world);
            const p = translation(world);
            return b[0].mul(v.v[0]).add(b[1].mul(v.v[1])).add(b[2].mul(v.v[2])).add(p);
        }

        /// Transforms a direction by the local rotation (without translation).
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `v` — local direction.
        ///
        /// Returns: transformed direction.
        pub inline fn transformDirectionLocal(self: Self, v: Vec3) Vec3 {
            return self.rotation.mulVec3(v);
        }

        /// Transforms a direction (without translation) via world rotation
        /// built from an explicitly passed parent world matrix.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `v` — local direction.
        ///
        /// Returns: world direction.
        pub fn transformDirectionWorld(self: Self, parent_world: Mat4, v: Vec3) Vec3 {
            return self.rotationWorld(parent_world).mulVec3(v);
        }

        /// Transforms a vector with local scale and rotation, without translation.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `v` — local vector.
        ///
        /// Returns: transformed vector.
        pub fn transformVectorLocal(self: Self, v: Vec3) Vec3 {
            const local = self.toMatrix();
            const b = basis(local);
            return b[0].mul(v.v[0]).add(b[1].mul(v.v[1])).add(b[2].mul(v.v[2]));
        }

        /// Transforms a vector with scale and rotation (without translation),
        /// using an explicitly passed parent world matrix.
        ///
        /// Parameters:
        /// - `self` — the transform value.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `v` — local vector.
        ///
        /// Returns: world vector.
        pub fn transformVectorWorld(self: Self, parent_world: Mat4, v: Vec3) Vec3 {
            const world = self.toWorldMatrix(parent_world);
            const b = basis(world);
            return b[0].mul(v.v[0]).add(b[1].mul(v.v[1])).add(b[2].mul(v.v[2]));
        }

        /// Translates position along an arbitrary direction by distance.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — translation direction.
        /// - `dist` — distance.
        pub inline fn translateAlongAxis(self: *Self, dir: Vec3, dist: Scalar) void {
            self.position.addSelf(dir.mul(dist));
        }

        /// Translates forward along the local forward axis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — translation distance.
        pub inline fn translateForwardLocal(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.forwardLocal(), dist);
        }

        /// Translates along world forward.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dist` — translation distance.
        pub inline fn translateForwardWorld(self: *Self, parent_world: Mat4, dist: Scalar) void {
            self.translateAlongAxis(self.forwardWorld(parent_world), dist);
        }

        /// Translates backward along the local backward axis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub inline fn translateBackwardLocal(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.backwardLocal(), dist);
        }

        /// Translates along world backward.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dist` — distance.
        pub inline fn translateBackwardWorld(self: *Self, parent_world: Mat4, dist: Scalar) void {
            self.translateAlongAxis(self.backwardWorld(parent_world), dist);
        }

        /// Translates right along the local right axis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub inline fn translateRightLocal(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.rightLocal(), dist);
        }

        /// Translates along world right.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dist` — distance.
        pub inline fn translateRightWorld(self: *Self, parent_world: Mat4, dist: Scalar) void {
            self.translateAlongAxis(self.rightWorld(parent_world), dist);
        }

        /// Translates left along the local left axis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub inline fn translateLeftLocal(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.leftLocal(), dist);
        }

        /// Translates along world left.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dist` — distance.
        pub inline fn translateLeftWorld(self: *Self, parent_world: Mat4, dist: Scalar) void {
            self.translateAlongAxis(self.leftWorld(parent_world), dist);
        }

        /// Translates up along the local up axis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub inline fn translateUpLocal(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.upLocal(), dist);
        }

        /// Translates along world up.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dist` — distance.
        pub inline fn translateUpWorld(self: *Self, parent_world: Mat4, dist: Scalar) void {
            self.translateAlongAxis(self.upWorld(parent_world), dist);
        }

        /// Translates down along the local down axis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub inline fn translateDownLocal(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.downLocal(), dist);
        }

        /// Translates along world down.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `parent_world` — world matrix of the parent (`Mat4.identity()` if parentless).
        /// - `dist` — distance.
        pub inline fn translateDownWorld(self: *Self, parent_world: Mat4, dist: Scalar) void {
            self.translateAlongAxis(self.downWorld(parent_world), dist);
        }
    };
}
