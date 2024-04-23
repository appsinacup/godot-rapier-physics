#ifndef RAPIER_WRAPPER_H
#define RAPIER_WRAPPER_H

/* Generated with cbindgen:0.26.0 */

namespace rapier2d {

enum class BodyType {
	Dynamic,
	Kinematic,
	Static,
};

/// A feature id where the feature type is packed into the same value as the feature index.
struct PackedFeatureId;

struct Handle {
	uint32_t id;
	uint32_t generation;
};

struct HandleDouble {
	size_t id;
	uint64_t generation;
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

/// The scalar type.
using Real = float;

/// The scalar type.
using Real = float;

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

struct ShapeInfo {
	Handle handle;
	Vector pixel_position;
	Real rotation;
	Real skew;
	Vector scale;
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
	size_t max_ccd_substeps;
	Real particle_radius;
	Real smoothing_factor;
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
	Vector pixel_position;
	Vector normal;
	Handle collider;
	UserData user_data;
};

struct ShapeCastResult {
	bool collided;
	Real toi;
	Vector pixel_witness1;
	Vector pixel_witness2;
	Vector normal1;
	Vector normal2;
	Handle collider;
	UserData user_data;
};

struct ContactResult {
	bool collided;
	bool within_margin;
	Real pixel_distance;
	Vector pixel_point1;
	Vector pixel_point2;
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
	bool is_stopped;
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
	Vector pixel_local_pos_1;
	Vector pixel_local_pos_2;
	Vector pixel_velocity_pos_1;
	Vector pixel_velocity_pos_2;
	Vector normal;
	Real pixel_distance;
	Real pixel_impulse;
	Real pixel_tangent_impulse;
};

using ContactPointCallback = bool (*)(Handle world_handle,
		const ContactPointInfo *contact_info,
		const ContactForceEventInfo *event_info);

struct OneWayDirection {
	bool body1;
	bool body2;
	Real pixel_body1_margin;
	Real pixel_body2_margin;
	Real last_timestep;
};

using CollisionModifyContactsCallback = OneWayDirection (*)(Handle world_handle,
		const CollisionFilterInfo *filter_info);

struct SimulationSettings {
	/// The timestep length (default: `1.0 / 60.0`)
	Real dt;
	/// The number of solver iterations run by the constraints solver for calculating forces (default: `4`).
	size_t num_solver_iterations;
	/// Number of addition friction resolution iteration run during the last solver sub-step (default: `4`).
	size_t num_additional_friction_iterations;
	/// Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration (default: `1`).
	size_t num_internal_pgs_iterations;
	Vector pixel_gravity;
	Vector pixel_liquid_gravity;
	size_t max_ccd_substeps;
};

extern "C" {

bool are_handles_equal(Handle handle1, Handle handle2);

bool are_handles_equal_double(HandleDouble handle1, HandleDouble handle2);

void body_add_force(Handle world_handle, Handle body_handle, const Vector *pixel_force);

void body_add_force_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *pixel_force,
		const Vector *pixel_point);

void body_add_torque(Handle world_handle, Handle body_handle, Real pixel_torque);

void body_apply_impulse(Handle world_handle, Handle body_handle, const Vector *pixel_impulse);

void body_apply_impulse_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *pixel_impulse,
		const Vector *pixel_point);

void body_apply_torque_impulse(Handle world_handle, Handle body_handle, Real pixel_torque_impulse);

void body_change_mode(Handle world_handle, Handle body_handle, BodyType body_type, bool wakeup);

Handle body_create(Handle world_handle,
		const Vector *pixel_pos,
		Real rot,
		const UserData *user_data,
		BodyType body_type);

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

void body_set_linear_velocity(Handle world_handle, Handle body_handle, const Vector *pixel_vel);

void body_set_mass_properties(Handle world_handle,
		Handle body_handle,
		Real mass,
		Real pixel_inertia,
		const Vector *pixel_local_com,
		bool wake_up,
		bool force_update);

void body_set_transform(Handle world_handle,
		Handle body_handle,
		const Vector *pixel_pos,
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

void collider_set_transform(Handle world_handle, Handle handle, ShapeInfo shape_info);

Material default_material();

QueryExcludedInfo default_query_excluded_info();

WorldSettings default_world_settings();

void fluid_add_effect_elasticity(Handle world_handle,
		HandleDouble fluid_handle,
		Real young_modulus,
		Real poisson_ratio,
		bool nonlinear_strain);

void fluid_add_effect_surface_tension_akinci(Handle world_handle,
		HandleDouble fluid_handle,
		Real fluid_tension_coefficient,
		Real boundary_adhesion_coefficient);

void fluid_add_effect_surface_tension_he(Handle world_handle,
		HandleDouble fluid_handle,
		Real fluid_tension_coefficient,
		Real boundary_adhesion_coefficient);

void fluid_add_effect_surface_tension_wcsph(Handle world_handle,
		HandleDouble fluid_handle,
		Real fluid_tension_coefficient,
		Real boundary_adhesion_coefficient);

void fluid_add_effect_viscosity_artificial(Handle world_handle,
		HandleDouble fluid_handle,
		Real fluid_viscosity_coefficient,
		Real boundary_viscosity_coefficient);

void fluid_add_effect_viscosity_dfsph(Handle world_handle,
		HandleDouble fluid_handle,
		Real fluid_viscosity_coefficient);

void fluid_add_effect_viscosity_xsph(Handle world_handle,
		HandleDouble fluid_handle,
		Real fluid_viscosity_coefficient,
		Real boundary_viscosity_coefficient);

void fluid_add_points_and_velocities(Handle world_handle,
		HandleDouble fluid_handle,
		const Vector *pixel_points,
		size_t point_count,
		const Vector *velocity_points);

void fluid_change_density(Handle world_handle, HandleDouble fluid_handle, Real density);

void fluid_change_points(Handle world_handle,
		HandleDouble fluid_handle,
		const Vector *pixel_points,
		size_t point_count);

void fluid_change_points_and_velocities(Handle world_handle,
		HandleDouble fluid_handle,
		const Vector *pixel_points,
		size_t point_count,
		const Vector *velocity_points);

void fluid_clear_effects(Handle world_handle, HandleDouble fluid_handle);

HandleDouble fluid_create(Handle world_handle, Real density);

void fluid_delete_points(Handle world_handle,
		HandleDouble fluid_handle,
		const size_t *indexes,
		size_t indexes_count);

void fluid_destroy(Handle world_handle, HandleDouble fluid_handle);

void fluid_get_accelerations(Handle world_handle,
		HandleDouble fluid_handle,
		Vector *pixel_acceleration,
		size_t acceleration_count);

void fluid_get_points(Handle world_handle,
		HandleDouble fluid_handle,
		Vector *pixel_points,
		size_t point_count);

void fluid_get_velocities(Handle world_handle,
		HandleDouble fluid_handle,
		Vector *pixel_velocities,
		size_t velocity_count);

size_t intersect_aabb(Handle world_handle,
		const Vector *pixel_aabb_min,
		const Vector *pixel_aabb_max,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

size_t intersect_point(Handle world_handle,
		const Vector *pixel_position,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

bool intersect_ray(Handle world_handle,
		const Vector *pixel_from,
		const Vector *dir,
		Real pixel_length,
		bool collide_with_body,
		bool collide_with_area,
		bool hit_from_inside,
		RayHitInfo *hit_info,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

size_t intersect_shape(Handle world_handle,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

Handle invalid_handle();

HandleDouble invalid_handle_double();

UserData invalid_user_data();

bool is_handle_valid(Handle handle);

bool is_handle_valid_double(HandleDouble handle);

bool is_user_data_valid(UserData user_data);

void joint_change_disable_collision(Handle world_handle,
		Handle joint_handle,
		bool disable_collision);

void joint_change_revolute_params(Handle world_handle,
		Handle joint_handle,
		Real angular_limit_lower,
		Real angular_limit_upper,
		bool angular_limit_enabled,
		Real pixel_motor_target_velocity,
		bool motor_enabled);

void joint_change_spring_params(Handle world_handle,
		Handle joint_handle,
		Real stiffness,
		Real damping,
		Real pixel_rest_length);

Handle joint_create_prismatic(Handle world_handle,
		Handle body_handle_1,
		Handle body_handle_2,
		const Vector *axis,
		const Vector *pixel_anchor_1,
		const Vector *pixel_anchor_2,
		const Vector *pixel_limits,
		bool disable_collision);

Handle joint_create_revolute(Handle world_handle,
		Handle body_handle_1,
		Handle body_handle_2,
		const Vector *pixel_anchor_1,
		const Vector *pixel_anchor_2,
		Real angular_limit_lower,
		Real angular_limit_upper,
		bool angular_limit_enabled,
		Real pixel_motor_target_velocity,
		bool motor_enabled,
		bool disable_collision);

Handle joint_create_spring(Handle world_handle,
		Handle body_handle_1,
		Handle body_handle_2,
		const Vector *pixel_anchor_1,
		const Vector *pixel_anchor_2,
		Real stiffness,
		Real damping,
		Real pixel_rest_length,
		bool disable_collision);

void joint_destroy(Handle world_handle, Handle joint_handle);

ShapeCastResult shape_casting(Handle world_handle,
		const Vector *pixel_motion,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info,
		bool ignore_intersecting);

ShapeCastResult shape_collide(const Vector *pixel_motion1,
		ShapeInfo shape_info1,
		const Vector *pixel_motion2,
		ShapeInfo shape_info2);

Handle shape_create_box(const Vector *pixel_size);

Handle shape_create_capsule(Real pixel_half_height, Real pixel_radius);

Handle shape_create_circle(Real pixel_radius);

Handle shape_create_convave_polyline(const Vector *pixel_points, size_t point_count);

Handle shape_create_convex_polyline(const Vector *pixel_points, size_t point_count);

Handle shape_create_halfspace(const Vector *normal, Real pixel_distance);

void shape_destroy(Handle shape_handle);

ContactResult shapes_contact(Handle world_handle,
		ShapeInfo shape_info1,
		ShapeInfo shape_info2,
		Real pixel_margin);

Handle world_create(const WorldSettings *settings);

void world_destroy(Handle world_handle);

size_t world_get_active_objects_count(Handle world_handle);

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
