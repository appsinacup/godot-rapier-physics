#ifndef RAPIER_WRAPPER_H
#define RAPIER_WRAPPER_H

/* Generated with cbindgen:0.26.0 */

namespace rapier2d {

#if !defined(DEFINE_SPIN_NO_STD)
template <typename T = void>
struct Lazy;
#endif

#if defined(DEFINE_SPIN_NO_STD)
template <typename T = void>
struct Lazy;
#endif

/// A feature id where the feature type is packed into the same value as the feature index.
struct PackedFeatureId;

struct Handle {
	uint32_t id;
	uint32_t generation;
};

#if defined(REAL_T_IS_DOUBLE)
/// The scalar type used throughout this crate.
using Real = double;
#endif

#if defined(REAL_T_IS_FLOAT)
/// The scalar type used throughout this crate.
using Real = float;
#endif

#if defined(REAL_T_IS_DOUBLE)
/// The scalar type used throughout this crate.
using Real = double;
#endif

#if defined(REAL_T_IS_FLOAT)
/// The scalar type used throughout this crate.
using Real = float;
#endif

struct Vector {
	Real x;
	Real y;
};

struct UserData {
	uint64_t part1;
	uint64_t part2;
};

struct Material {
	Real friction;
	Real restitution;
};

struct QueryExcludedInfo {
	uint32_t query_collision_layer_mask;
	uint64_t query_canvas_instance_id;
	Handle *query_exclude;
	uint32_t query_exclude_size;
	int64_t query_exclude_body;
};

struct WorldSettings {
	Real sleep_linear_threshold;
	Real sleep_angular_threshold;
	Real sleep_time_until_sleep;
	Real solver_prediction_distance;
};

struct PointHitInfo {
	Handle collider;
	UserData user_data;
};

using QueryHandleExcludedCallback = bool (*)(Handle world_handle,
		Handle collider_handle,
		const UserData *user_data,
		const QueryExcludedInfo *handle_excluded_info);

struct RayHitInfo {
	Vector position;
	Vector normal;
	Handle collider;
	UserData user_data;
};

struct ShapeCastResult {
	bool collided;
	Real toi;
	Vector witness1;
	Vector witness2;
	Vector normal1;
	Vector normal2;
	Handle collider;
	UserData user_data;
};

struct ShapeInfo {
	Handle handle;
	Vector position;
	Real rotation;
};

struct ContactResult {
	bool collided;
	bool within_margin;
	Real distance;
	Vector point1;
	Vector point2;
	Vector normal1;
	Vector normal2;
};

struct ActiveBodyInfo {
	Handle body_handle;
	UserData body_user_data;
};

using ActiveBodyCallback = void (*)(Handle world_handle, const ActiveBodyInfo *active_body_info);

struct CollisionFilterInfo {
	UserData user_data1;
	UserData user_data2;
};

using CollisionFilterCallback = bool (*)(Handle world_handle, const CollisionFilterInfo *filter_info);

struct CollisionEventInfo {
	Handle collider1;
	Handle collider2;
	UserData user_data1;
	UserData user_data2;
	bool is_sensor;
	bool is_started;
	bool is_removed;
};

using CollisionEventCallback = void (*)(Handle world_handle, const CollisionEventInfo *event_info);

struct ContactForceEventInfo {
	Handle collider1;
	Handle collider2;
	UserData user_data1;
	UserData user_data2;
};

using ContactForceEventCallback = bool (*)(Handle world_handle,
		const ContactForceEventInfo *event_info);

struct ContactPointInfo {
	Vector local_pos_1;
	Vector local_pos_2;
	Vector velocity_pos_1;
	Vector velocity_pos_2;
	Vector normal;
	Real distance;
	Real impulse;
	Real tangent_impulse;
};

using ContactPointCallback = bool (*)(Handle world_handle,
		const ContactPointInfo *contact_info,
		const ContactForceEventInfo *event_info);

struct OneWayDirection {
	bool body1;
	bool body2;
};

using CollisionModifyContactsCallback = OneWayDirection (*)(Handle world_handle,
		const CollisionFilterInfo *filter_info);

struct SimulationSettings {
	/// The timestep length (default: `1.0 / 60.0`)
	Real dt;
	/// Minimum timestep size when using CCD with multiple substeps (default `1.0 / 60.0 / 100.0`)
	///
	/// When CCD with multiple substeps is enabled, the timestep is subdivided
	/// into smaller pieces. This timestep subdivision won't generate timestep
	/// lengths smaller than `min_ccd_dt`.
	///
	/// Setting this to a large value will reduce the opportunity to performing
	/// CCD substepping, resulting in potentially more time dropped by the
	/// motion-clamping mechanism. Setting this to an very small value may lead
	/// to numerical instabilities.
	Real min_ccd_dt;
	/// 0-1: multiplier for how much of the constraint violation (e.g. contact penetration)
	/// will be compensated for during the velocity solve.
	/// (default `0.8`).
	Real erp;
	/// 0-1: the damping ratio used by the springs for Baumgarte constraints stabilization.
	/// Lower values make the constraints more compliant (more "springy", allowing more visible penetrations
	/// before stabilization).
	/// (default `0.25`).
	Real damping_ratio;
	/// 0-1: multiplier for how much of the joint violation
	/// will be compensated for during the velocity solve.
	/// (default `1.0`).
	Real joint_erp;
	/// The fraction of critical damping applied to the joint for constraints regularization.
	/// (default `0.25`).
	Real joint_damping_ratio;
	/// Amount of penetration the engine wont attempt to correct (default: `0.001m`).
	Real allowed_linear_error;
	/// Maximum amount of penetration the solver will attempt to resolve in one timestep.
	Real max_penetration_correction;
	/// The maximal distance separating two objects that will generate predictive contacts (default: `0.002`).
	Real prediction_distance;
	/// Maximum number of iterations performed to solve non-penetration and joint constraints (default: `4`).
	size_t max_velocity_iterations;
	/// Maximum number of iterations performed to solve friction constraints (default: `8`).
	size_t max_velocity_friction_iterations;
	/// Maximum number of iterations performed to remove the energy introduced by penetration corrections  (default: `1`).
	size_t max_stabilization_iterations;
	/// If `false`, friction and non-penetration constraints will be solved in the same loop. Otherwise,
	/// non-penetration constraints are solved first, and friction constraints are solved after (default: `true`).
	bool interleave_restitution_and_friction_resolution;
	/// Minimum number of dynamic bodies in each active island (default: `128`).
	size_t min_island_size;
	/// Maximum number of substeps performed by the  solver (default: `1`).
	size_t max_ccd_substeps;
	Vector gravity;
};

extern "C" {

bool are_handles_equal(Handle handle1, Handle handle2);

void body_add_force(Handle world_handle, Handle body_handle, const Vector *force);

void body_add_force_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *force,
		const Vector *point);

void body_add_torque(Handle world_handle, Handle body_handle, Real torque);

void body_apply_impulse(Handle world_handle, Handle body_handle, const Vector *impulse);

void body_apply_impulse_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *impulse,
		const Vector *point);

void body_apply_torque_impulse(Handle world_handle, Handle body_handle, Real torque_impulse);

Handle body_create_dynamic(Handle world_handle,
		const Vector *pos,
		Real rot,
		const UserData *user_data);

Handle body_create_fixed(Handle world_handle,
		const Vector *pos,
		Real rot,
		const UserData *user_data);

void body_destroy(Handle world_handle, Handle body_handle);

void body_force_sleep(Handle world_handle, Handle body_handle);

Real body_get_angle(Handle world_handle, Handle body_handle);

Real body_get_angular_velocity(Handle world_handle, Handle body_handle);

Vector body_get_constant_force(Handle world_handle, Handle body_handle);

Real body_get_constant_torque(Handle world_handle, Handle body_handle);

Vector body_get_linear_velocity(Handle world_handle, Handle body_handle);

Vector body_get_position(Handle world_handle, Handle body_handle);

bool body_is_ccd_enabled(Handle world_handle, Handle body_handle);

void body_reset_forces(Handle world_handle, Handle body_handle);

void body_reset_torques(Handle world_handle, Handle body_handle);

void body_set_angular_damping(Handle world_handle, Handle body_handle, Real angular_damping);

void body_set_angular_velocity(Handle world_handle, Handle body_handle, Real vel);

void body_set_can_sleep(Handle world_handle, Handle body_handle, bool can_sleep);

void body_set_ccd_enabled(Handle world_handle, Handle body_handle, bool enable);

void body_set_gravity_scale(Handle world_handle,
		Handle body_handle,
		Real gravity_scale,
		bool wake_up);

void body_set_linear_damping(Handle world_handle, Handle body_handle, Real linear_damping);

void body_set_linear_velocity(Handle world_handle, Handle body_handle, const Vector *vel);

void body_set_mass_properties(Handle world_handle,
		Handle body_handle,
		Real mass,
		Real inertia,
		const Vector *local_com,
		bool wake_up,
		bool force_update);

void body_set_transform(Handle world_handle,
		Handle body_handle,
		const Vector *pos,
		Real rot,
		bool wake_up);

void body_update_material(Handle world_handle, Handle body_handle, const Material *mat);

void body_wake_up(Handle world_handle, Handle body_handle, bool strong);

Handle collider_create_sensor(Handle world_handle,
		Handle shape_handle,
		Handle body_handle,
		const UserData *user_data);

Handle collider_create_solid(Handle world_handle,
		Handle shape_handle,
		const Material *mat,
		Handle body_handle,
		const UserData *user_data);

void collider_destroy(Handle world_handle, Handle handle);

Real collider_get_angle(Handle world_handle, Handle handle);

Vector collider_get_position(Handle world_handle, Handle handle);

void collider_set_collision_events_enabled(Handle world_handle, Handle handle, bool enable);

void collider_set_contact_force_events_enabled(Handle world_handle, Handle handle, bool enable);

void collider_set_transform(Handle world_handle,
		Handle handle,
		Handle shape_handle,
		const Vector *pos,
		Real rot,
		const Vector *scale);

Material default_material();

QueryExcludedInfo default_query_excluded_info();

WorldSettings default_world_settings();

size_t intersect_aabb(Handle world_handle,
		const Vector *aabb_min,
		const Vector *aabb_max,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

size_t intersect_point(Handle world_handle,
		const Vector *position,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

bool intersect_ray(Handle world_handle,
		const Vector *from,
		const Vector *dir,
		Real length,
		bool collide_with_body,
		bool collide_with_area,
		bool hit_from_inside,
		RayHitInfo *hit_info,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

size_t intersect_shape(Handle world_handle,
		const Vector *position,
		Real rotation,
		Handle shape_handle,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

Handle invalid_handle();

UserData invalid_user_data();

bool is_handle_valid(Handle handle);

bool is_user_data_valid(UserData user_data);

void joint_change_revolute_params(Handle world_handle,
		Handle joint_handle,
		Real angular_limit_lower,
		Real angular_limit_upper,
		bool angular_limit_enabled,
		Real motor_target_velocity,
		bool motor_enabled);

Handle joint_create_prismatic(Handle world_handle,
		Handle body_handle_1,
		Handle body_handle_2,
		const Vector *axis,
		const Vector *anchor_1,
		const Vector *anchor_2,
		const Vector *limits);

Handle joint_create_revolute(Handle world_handle,
		Handle body_handle_1,
		Handle body_handle_2,
		const Vector *anchor_1,
		const Vector *anchor_2,
		Real angular_limit_lower,
		Real angular_limit_upper,
		bool angular_limit_enabled,
		Real motor_target_velocity,
		bool motor_enabled);

void joint_destroy(Handle world_handle, Handle joint_handle);

ShapeCastResult shape_casting(Handle world_handle,
		const Vector *motion,
		const Vector *position,
		Real rotation,
		Handle shape_handle,
		bool collide_with_body,
		bool collide_with_area,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

Handle shape_create_box(const Vector *size);

Handle shape_create_capsule(Real half_height, Real radius);

Handle shape_create_circle(Real radius);

Handle shape_create_compound(const ShapeInfo *shapes, size_t shape_count);

Handle shape_create_convave_polyline(const Vector *points, size_t point_count);

Handle shape_create_convex_polyline(const Vector *points, size_t point_count);

Handle shape_create_halfspace(const Vector *normal);

void shape_destroy(Handle shape_handle);

ContactResult shapes_contact(Handle world_handle,
		Handle shape_handle1,
		const Vector *position1,
		Real rotation1,
		Handle shape_handle2,
		const Vector *position2,
		Real rotation2,
		Real margin);

Handle world_create(const WorldSettings *settings);

void world_destroy(Handle world_handle);

void world_set_active_body_callback(Handle world_handle, ActiveBodyCallback callback);

void world_set_body_collision_filter_callback(Handle world_handle,
		CollisionFilterCallback callback);

void world_set_collision_event_callback(Handle world_handle, CollisionEventCallback callback);

void world_set_contact_force_event_callback(Handle world_handle,
		ContactForceEventCallback callback);

void world_set_contact_point_callback(Handle world_handle, ContactPointCallback callback);

void world_set_modify_contacts_callback(Handle world_handle,
		CollisionModifyContactsCallback callback);

void world_set_sensor_collision_filter_callback(Handle world_handle,
		CollisionFilterCallback callback);

void world_step(Handle world_handle, const SimulationSettings *settings);

} // extern "C"

} // namespace rapier2d

#endif // RAPIER_WRAPPER_H
