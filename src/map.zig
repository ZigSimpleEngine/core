const std = @import("std");

/// Creates a global registry type parameterized by name and data type.
///
/// Each specialization `Map(name, T)` creates a separate static container.
/// The registry stores a single optional value of type `T` accessible via `get`/`set`.
///
/// Parameters:
/// - `name` — comptime identifier distinguishing specializations with the same `T`.
/// - `data_` — type of the stored value.
///
/// Returns: `Map` struct type with accessors and `Data` alias.
pub fn Map(comptime name: anytype, comptime data_: type) type {
    _ = name;
    return struct {
        /// Internal registry storage.
        ///
        /// Holds `null` until the first `set`, then the last stored value.
        var common_data: ?Data = null;

        /// Alias for the stored data type.
        pub const Data = data_;

        /// Returns the current registry value or a default.
        ///
        /// Parameters:
        /// - `default` — value returned if the registry is not yet initialized.
        ///
        /// Returns: stored value or `default`.
        pub fn get(default: Data) Data {
            return common_data orelse default;
        }

        /// Stores a value in the global registry, overwriting the previous one.
        ///
        /// Parameters:
        /// - `data` — new value to store.
        pub fn set(data: Data) void {
            common_data = data;
        }
    };
}
