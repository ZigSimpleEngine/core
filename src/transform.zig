const std = @import("std");
const math = @import("math");

/// Creates a hierarchical 3D transform type parameterized by scalar type.
///
/// Implemented as `opaque` with heap-allocated `Impl`, storing position, rotation, scale
/// and cached world matrix. Supports parent-child hierarchy, local and world
/// operations, and recalculation of matrices upward/downward.
///
/// Parameters:
/// - `scalar_type_` — scalar type for vector/matrix components (`f32`, `f64`, etc.).
///
/// Returns: `opaque` type `Transform(Scalar)` with management methods.
pub fn Transform(comptime scalar_type_: type) type {
    return opaque {
        const Self = @This();

        /// Scalar type of the transform, as specified at specialization.
        pub const Scalar = scalar_type_;

        /// Three-component vector based on `Scalar`.
        pub const Vec3 = math.Vec(3, Scalar);

        /// Quaternion based on `Scalar`, representing rotation.
        pub const QuatT = math.Quat(Scalar);

        /// 4x4 matrix based on `Scalar`, representing the final transform.
        pub const Mat4 = math.Mat(4, 4, Scalar);

        /// Internal transform representation stored on the heap.
        const Impl = struct {
            /// List of child transforms.
            children: std.ArrayList(*Self) = .empty,

            /// Pointer to parent transform, `null` if root.
            parent: ?*Self = null,

            /// Local position in parent space.
            position: Vec3 = Vec3.zero(),

            /// Local rotation as quaternion.
            rotation: QuatT = .{},

            /// Local scale per axis.
            scale: Vec3 = Vec3.one(),

            /// Cached world matrix.
            matrix: Mat4 = Mat4.identity(),
        };

        /// Casts `*Self` to internal `Impl` pointer.
        ///
        /// Parameters:
        /// - `self` — pointer to the opaque transform object.
        ///
        /// Returns: mutable pointer to `Impl`.
        pub inline fn impl(self: *Self) *Impl {
            return @ptrCast(@alignCast(self));
        }

        /// Casts `*const Self` to const `Impl` pointer.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: const pointer to `Impl`.
        inline fn implConst(self: *const Self) *const Impl {
            return @ptrCast(@alignCast(self));
        }

        /// Creates a new transform with default identity parameters.
        ///
        /// Parameters:
        /// - `allocator` — allocator for `Impl` allocation.
        ///
        /// Returns: pointer to the created transform or allocation error.
        pub fn create(allocator: std.mem.Allocator) !*Self {
            const m = try allocator.create(Impl);
            m.* = .{};
            return @ptrCast(m);
        }

        /// Creates a transform with given position, rotation and scale.
        ///
        /// Parameters:
        /// - `allocator` — allocator for object allocation.
        /// - `pos` — initial local position.
        /// - `rot` — initial rotation.
        /// - `sc` — initial scale.
        ///
        /// Returns: pointer to the created and recalculated transform.
        pub fn init(allocator: std.mem.Allocator, pos: Vec3, rot: QuatT, sc: Vec3) !*Self {
            const self = try create(allocator);
            const m = self.impl();
            m.position = pos;
            m.rotation = rot;
            m.scale = sc;
            self.recalculateTransformMatrix();
            return self;
        }

        /// Creates an identity transform with scale `1` and zero position.
        ///
        /// Parameters:
        /// - `allocator` — allocator for allocation.
        ///
        /// Returns: pointer to the identity transform.
        pub fn identity(allocator: std.mem.Allocator) !*Self {
            var transform = try create(allocator);
            transform.scale().* = .one();
            return transform;
        }

        /// Destroys the transform, freeing child list and `Impl` memory.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform to destroy.
        /// - `allocator` — same allocator used for `create`.
        pub fn destroy(self: *Self, allocator: std.mem.Allocator) void {
            const m = self.impl();
            m.children.deinit(allocator);
            allocator.destroy(m);
        }

        /// Returns a mutable pointer to the local position.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        ///
        /// Returns: pointer to `Vec3` position for direct modification.
        pub fn position(self: *Self) *Vec3 {
            return &self.impl().position;
        }

        /// Returns a mutable pointer to the rotation quaternion.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        ///
        /// Returns: pointer to `QuatT` for direct modification.
        pub fn rotation(self: *Self) *QuatT {
            return &self.impl().rotation;
        }

        /// Returns a mutable pointer to the scale vector.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        ///
        /// Returns: pointer to `Vec3` scale.
        pub fn scale(self: *Self) *Vec3 {
            return &self.impl().scale;
        }

        /// Orients the transform to look at a target with world up.
        ///
        /// Parameters:
        /// - `self` — pointer to the object to transform.
        /// - `target` — world point to look at.
        /// - `world_up` — world up vector for orientation stability.
        pub fn lookAt(self: *Self, target: Vec3, world_up: Vec3) void {
            const direction = target.sub(self.impl().position).normalize();
            self.impl().rotation = QuatT.lookAt(direction, world_up);
        }

        /// Returns the cached world matrix.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: `Mat4` current world matrix.
        pub fn getMatrix(self: *const Self) Mat4 {
            return self.implConst().matrix;
        }

        /// Extracts world basis from cached matrix (X, Y, Z axes).
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: array of three `Vec3` basis vectors.
        fn worldBasis(self: *const Self) [3]Vec3 {
            const m = self.implConst().matrix;
            return .{
                Vec3.init(.{ m.data[0].v[0], m.data[0].v[1], m.data[0].v[2] }),
                Vec3.init(.{ m.data[1].v[0], m.data[1].v[1], m.data[1].v[2] }),
                Vec3.init(.{ m.data[2].v[0], m.data[2].v[1], m.data[2].v[2] }),
            };
        }

        /// Safely normalizes a vector, returning zero for degenerate inputs.
        ///
        /// Parameters:
        /// - `v` — input vector to normalize.
        ///
        /// Returns: normalized vector or zero if length < 1e-9.
        fn safeNormalize(v: Vec3) Vec3 {
            const len = v.length();
            if (len <= @as(Scalar, 1e-9)) return Vec3.zero();
            return v.mul(@as(Scalar, 1) / len);
        }

        /// Builds a quaternion rotating vector `from` to `to`.
        ///
        /// Handles collinear cases (parallel and opposite directions).
        ///
        /// Parameters:
        /// - `from` — source direction.
        /// - `to` — target direction.
        ///
        /// Returns: quaternion of shortest rotation.
        fn fromToRotation(from: Vec3, to: Vec3) QuatT {
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

        /// Computes world rotation including parent hierarchy.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: normalized world quaternion.
        fn worldRotation(self: *const Self) QuatT {
            const local = self.implConst().rotation;
            const parent = self.implConst().parent;
            if (parent) |p| return p.worldRotation().mul(local).normalize();
            return local;
        }

        /// Rotates a local axis to world direction via `worldRotation`.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        /// - `axis` — local axis.
        ///
        /// Returns: world axis as `Vec3`.
        fn worldAxis(self: *const Self, axis: Vec3) Vec3 {
            return self.worldRotation().mulVec3(axis);
        }

        /// Returns parent world rotation or identity quaternion.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: parent quaternion or identity.
        fn parentWorldRotation(self: *const Self) QuatT {
            if (self.impl().parent) |p| return p.worldRotation();
            return QuatT.identity();
        }

        /// Sets local rotation so that a local axis aligns with a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform to modify.
        /// - `local_axis` — local axis to align.
        /// - `world_dir` — target world direction.
        fn setLocalAxis(self: *Self, local_axis: Vec3, world_dir: Vec3) void {
            const parent_rot = self.parentWorldRotation();
            const dir_local = parent_rot.conjugate().mulVec3(world_dir);
            self.impl().rotation = fromToRotation(local_axis, dir_local);
        }

        /// Returns world right vector (local +X).
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: normalized world right vector.
        pub fn right(self: *const Self) Vec3 {
            return self.worldAxis(Vec3.unit(0));
        }

        /// Returns world left vector, opposite of right.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: left vector.
        pub fn left(self: *const Self) Vec3 {
            return self.right().neg();
        }

        /// Returns world up vector (local +Y).
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: world up vector.
        pub fn up(self: *const Self) Vec3 {
            return self.worldAxis(Vec3.unit(1));
        }

        /// Returns world down vector, opposite of up.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: down vector.
        pub fn down(self: *const Self) Vec3 {
            return self.up().neg();
        }

        /// Returns world forward vector (local -Z).
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: world forward vector.
        pub fn forward(self: *const Self) Vec3 {
            return self.worldAxis(Vec3.unit(2).neg());
        }

        /// Returns world backward vector, opposite of forward.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: backward vector.
        pub fn backward(self: *const Self) Vec3 {
            return self.forward().neg();
        }

        /// Aligns local +X axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target world right direction.
        pub fn setRight(self: *Self, dir: Vec3) void {
            self.setLocalAxis(Vec3.unit(0), dir);
        }

        /// Aligns local -X axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target world left direction.
        pub fn setLeft(self: *Self, dir: Vec3) void {
            self.setRight(dir.neg());
        }

        /// Aligns local +Y axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target world up direction.
        pub fn setUp(self: *Self, dir: Vec3) void {
            self.setLocalAxis(Vec3.unit(1), dir);
        }

        /// Aligns local -Y axis to a world direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target world down direction.
        pub fn setDown(self: *Self, dir: Vec3) void {
            self.setUp(dir.neg());
        }

        /// Aligns local -Z axis to a world forward direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target world forward direction.
        pub fn setForward(self: *Self, dir: Vec3) void {
            self.setLocalAxis(Vec3.unit(2).neg(), dir);
        }

        /// Aligns local +Z axis to a world backward direction.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — target world backward direction.
        pub fn setBackward(self: *Self, dir: Vec3) void {
            self.setForward(dir.neg());
        }

        /// Translates the transform in its local basis.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `offset` — offset in local axes (x — right, y — up, z — forward).
        pub fn translate(self: *Self, offset: Vec3) void {
            const moved = self.right().mul(offset.v[0])
                .add(self.up().mul(offset.v[1]))
                .add(self.forward().mul(offset.v[2]));
            self.impl().position.addSelf(moved);
        }

        /// Rotates the transform by Euler angles in local space.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `euler` — Euler angles (radians) per axis.
        pub fn rotate(self: *Self, euler: Vec3) void {
            self.impl().rotation = self.impl().rotation.mul(QuatT.fromEuler(euler)).normalize();
        }

        /// Rotates the transform around a point in world space.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `point` — world center point of rotation.
        /// - `axis` — world rotation axis.
        /// - `angle` — rotation angle in radians.
        pub fn rotateAround(self: *Self, point: Vec3, axis: Vec3, angle: Scalar) void {
            const q = math.quat.angleAxis(angle, safeNormalize(axis));
            const dif = q.mulVec3(self.impl().position.sub(point));
            self.impl().position = point.add(dif);
            self.impl().rotation = q.mul(self.impl().rotation).normalize();
        }

        /// Transforms a point from local to world space with scale and translation.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        /// - `v` — local point.
        ///
        /// Returns: world point.
        pub fn transformPoint(self: *const Self, v: Vec3) Vec3 {
            const b = self.worldBasis();
            const m = self.implConst().matrix;
            const p = Vec3.init(.{ m.data[3].v[0], m.data[3].v[1], m.data[3].v[2] });
            return b[0].mul(v.v[0]).add(b[1].mul(v.v[1])).add(b[2].mul(v.v[2])).add(p);
        }

        /// Transforms a direction (without translation) via world rotation.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        /// - `v` — local direction.
        ///
        /// Returns: world direction.
        pub fn transformDirection(self: *const Self, v: Vec3) Vec3 {
            return self.worldRotation().mulVec3(v);
        }

        /// Transforms a vector with scale and rotation, without translation.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        /// - `v` — local vector.
        ///
        /// Returns: world vector.
        pub fn transformVector(self: *const Self, v: Vec3) Vec3 {
            const b = self.worldBasis();
            return b[0].mul(v.v[0]).add(b[1].mul(v.v[1])).add(b[2].mul(v.v[2]));
        }

        /// Translates position along an arbitrary direction by distance.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dir` — translation direction.
        /// - `dist` — distance.
        pub fn translateAlongAxis(self: *Self, dir: Vec3, dist: Scalar) void {
            self.impl().position.addSelf(dir.mul(dist));
        }

        /// Translates forward along world forward.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — translation distance.
        pub fn translateForward(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.forward(), dist);
        }

        /// Translates backward along world backward.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub fn translateBackward(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.backward(), dist);
        }

        /// Translates right along world right.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub fn translateRight(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.right(), dist);
        }

        /// Translates left along world left.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub fn translateLeft(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.left(), dist);
        }

        /// Translates up along world up.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub fn translateUp(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.up(), dist);
        }

        /// Translates down along world down.
        ///
        /// Parameters:
        /// - `self` — pointer to the transform.
        /// - `dist` — distance.
        pub fn translateDown(self: *Self, dist: Scalar) void {
            self.translateAlongAxis(self.down(), dist);
        }

        /// Returns a child transform by index.
        ///
        /// Parameters:
        /// - `self` — const pointer to the parent.
        /// - `id` — child index.
        ///
        /// Returns: child pointer or `null` if out of range.
        pub fn getChild(self: *const Self, id: usize) ?*Self {
            const m = self.implConst();
            if (id >= m.children.items.len) return null;
            return m.children.items[id];
        }

        /// Returns the number of child transforms.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: number of children.
        pub fn getChildrenCount(self: *const Self) usize {
            return self.implConst().children.items.len;
        }

        /// Returns an iterator over child transforms.
        ///
        /// Parameters:
        /// - `self` — const pointer to the parent.
        ///
        /// Returns: `ChildrenIterator` over child slice.
        pub fn getChildren(self: *const Self) ChildrenIterator {
            return .{ .children = self.implConst().children.items, .index = 0 };
        }

        /// Iterator over child transforms.
        pub const ChildrenIterator = struct {
            /// Slice of child transform pointers.
            children: []*Self,

            /// Current iteration index.
            index: usize,

            /// Returns the next element or `null`.
            ///
            /// Parameters:
            /// - `self` — pointer to the iterator.
            ///
            /// Returns: next child or `null` if at end.
            pub fn next(self: *ChildrenIterator) ?*Self {
                if (self.index >= self.children.len) return null;
                const v = self.children[self.index];
                self.index += 1;
                return v;
            }

            /// Resets the iterator to the start.
            ///
            /// Parameters:
            /// - `self` — pointer to the iterator.
            pub fn reset(self: *ChildrenIterator) void {
                self.index = 0;
            }
        };

        /// Replaces a child at an index, detaching the previous one.
        ///
        /// Parameters:
        /// - `self` — pointer to the parent.
        /// - `id` — index of the child to replace.
        /// - `transform` — new child transform.
        pub fn setChild(self: *Self, id: usize, transform: *Self) void {
            const m = self.impl();
            if (id >= m.children.items.len) return;
            const old = m.children.items[id];
            if (old.impl().parent == self) old.impl().parent = null;
            m.children.items[id] = transform;
            transform.impl().parent = self;
        }

        /// Appends a child transform to the list.
        ///
        /// Parameters:
        /// - `self` — pointer to the parent.
        /// - `allocator` — allocator for list expansion.
        /// - `transform` — child to add, its `parent` will be set.
        ///
        /// Returns: allocation error on OOM.
        pub fn addChild(self: *Self, allocator: std.mem.Allocator, transform: *Self) !void {
            const m = self.impl();
            try m.children.append(allocator, transform);
            transform.impl().parent = self;
        }

        /// Removes a child at an index, detaching it.
        ///
        /// Parameters:
        /// - `self` — pointer to the parent.
        /// - `allocator` — allocator (reserved for API symmetry).
        /// - `id` — index of the child to remove.
        pub fn removeChild(self: *Self, allocator: std.mem.Allocator, id: usize) void {
            _ = allocator;
            const m = self.impl();
            if (id >= m.children.items.len) return;
            const child = m.children.items[id];
            if (child.impl().parent == self) child.impl().parent = null;
            _ = m.children.orderedRemove(id);
        }

        /// Returns the parent transform pointer.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: parent or `null`.
        pub fn getParent(self: *const Self) ?*Self {
            return self.implConst().parent;
        }

        /// Sets the parent transform without modifying child lists.
        ///
        /// Parameters:
        /// - `self` — pointer to the object to transform.
        /// - `parent` — new parent or `null` to detach.
        pub fn setParent(self: *Self, parent: ?*Self) void {
            self.impl().parent = parent;
        }

        /// Computes the local matrix from `position`/`rotation`/`scale`.
        ///
        /// Parameters:
        /// - `self` — const pointer to the transform.
        ///
        /// Returns: local `Mat4` matrix.
        fn computeLocalMatrix(self: *const Self) Mat4 {
            const m = self.implConst();
            var mat = Mat4.identity();
            mat = mat.translate(m.position);
            const rot = math.quat.mat4_cast(m.rotation);
            mat = mat.mul(rot);
            mat = mat.scale(m.scale);
            return mat;
        }

        /// Recalculates the world matrix including the parent.
        ///
        /// Parameters:
        /// - `self` — pointer to the object to transform.
        pub fn recalculateTransformMatrix(self: *Self) void {
            const local = self.computeLocalMatrix();
            const m = self.impl();
            if (m.parent) |p| m.matrix = p.implConst().matrix.mul(local) else m.matrix = local;
        }

        /// Recalculates matrices from the current node up to the root and then down.
        ///
        /// Collects ancestor chain on a stack and recalculates from root to `self`.
        ///
        /// Parameters:
        /// - `self` — pointer to the start node.
        /// - `allocator` — allocator for temporary stack.
        ///
        /// Returns: stack allocation error.
        pub fn recalculateTransformMatricesUpward(self: *Self, allocator: std.mem.Allocator) !void {
            var stack: std.ArrayList(*Self) = .empty;
            defer stack.deinit(allocator);
            var cur: ?*Self = self;
            while (cur) |node| {
                try stack.append(allocator, node);
                cur = node.implConst().parent;
            }
            var i: usize = stack.items.len;
            while (i > 0) {
                i -= 1;
                stack.items[i].recalculateTransformMatrix();
            }
        }

        /// Recursively recalculates world matrices for the subtree.
        ///
        /// Parameters:
        /// - `self` — pointer to the subtree root.
        pub fn recalculateTransformMatricesDownward(self: *Self) void {
            self.recalculateTransformMatrix();
            const m = self.implConst();
            for (m.children.items) |child| child.recalculateTransformMatricesDownward();
        }
    };
}
