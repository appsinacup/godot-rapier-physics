#ifndef RAPIER_SPACE_2D_H
#define RAPIER_SPACE_2D_H

#include "../bodies/rapier_area_2d.h"
#include "../bodies/rapier_body_2d.h"

#include <gdextension_interface.h>
#include <godot_cpp/classes/physics_direct_space_state2d.hpp>
#include <godot_cpp/classes/physics_direct_space_state2d_extension.hpp>
#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/physics_server2d_extension_motion_result.hpp>
#include <godot_cpp/classes/physics_server2d_extension_ray_result.hpp>
#include <godot_cpp/classes/physics_server2d_extension_shape_rest_info.hpp>
#include <godot_cpp/classes/physics_server2d_extension_shape_result.hpp>
#include <godot_cpp/classes/physics_test_motion_result2d.hpp>
#include <godot_cpp/templates/hash_set.hpp>

using namespace godot;

class RapierSpace2D;
class RapierDirectSpaceState2D;

class RapierSpace2D {
	RapierDirectSpaceState2D *direct_access = nullptr;
	RID rid;

	rapier2d::Handle handle = rapier2d::invalid_handle();

	struct RemovedColliderInfo {
		RID rid;
		ObjectID instance_id;
		uint32_t shape_index = 0;
		RapierCollisionObject2D::Type type = RapierCollisionObject2D::TYPE_BODY;
	};

	HashMap<uint32_t, RemovedColliderInfo> removed_colliders;

	SelfList<RapierBody2D>::List active_list;
	SelfList<RapierBody2D>::List mass_properties_update_list;
	SelfList<RapierBody2D>::List gravity_update_list;

	SelfList<RapierBody2D>::List state_query_list;
	SelfList<RapierArea2D>::List monitor_query_list;

	SelfList<RapierArea2D>::List area_update_list;
	SelfList<RapierBody2D>::List body_area_update_list;

	int solver_iterations = 0;

	real_t contact_recycle_radius = 0.0;
	real_t contact_max_separation = 0.0;
	real_t contact_max_allowed_penetration = 0.0;
	real_t contact_bias = 0.0;
	real_t constraint_bias = 0.0;

	Vector2 fluid_default_gravity_dir = Vector2(0.0, -1.0);
	real_t fluid_default_gravity_value = -9.81;

	Vector2 default_gravity_dir = Vector2(0.0, -1.0);
	real_t default_gravity_value = -9.81;
	real_t default_linear_damping;
	real_t default_angular_damping;

	bool locked = false;

	real_t last_step = 0.001;

	int island_count = 0;
	int active_objects = 0;
	int collision_pairs = 0;

	PackedVector2Array contact_debug;
	int contact_debug_count = 0;

	friend class RapierDirectSpaceState2D;

	static void active_body_callback(rapier2d::Handle world_handle, const rapier2d::ActiveBodyInfo *active_body_info);

	struct CollidersInfo {
		uint32_t shape1 = 0;
		RapierCollisionObject2D *object1 = nullptr;
		uint32_t shape2 = 0;
		RapierCollisionObject2D *object2 = nullptr;
	};

	static bool collision_filter_common_callback(rapier2d::Handle world_handle, const rapier2d::CollisionFilterInfo *filter_info, CollidersInfo &r_colliders_info);
	static bool collision_filter_body_callback(rapier2d::Handle world_handle, const rapier2d::CollisionFilterInfo *filter_info);
	static bool collision_filter_sensor_callback(rapier2d::Handle world_handle, const rapier2d::CollisionFilterInfo *filter_info);
	static rapier2d::OneWayDirection collision_modify_contacts_callback(rapier2d::Handle world_handle, const rapier2d::CollisionFilterInfo *filter_info);

	static void collision_event_callback(rapier2d::Handle world_handle, const rapier2d::CollisionEventInfo *event_info);

	static bool contact_force_event_callback(rapier2d::Handle world_handle, const rapier2d::ContactForceEventInfo *event_info);
	static bool contact_point_callback(rapier2d::Handle world_handle, const rapier2d::ContactPointInfo *contact_info, const rapier2d::ContactForceEventInfo *event_info);

	static bool _is_handle_excluded_callback(const rapier2d::Handle world_handle, const rapier2d::Handle collider_handle, const rapier2d::UserData *collider, const rapier2d::QueryExcludedInfo *handle_excluded_info);

	static Object *_get_object_instance_hack(uint64_t p_object_id) {
		return reinterpret_cast<Object *>((GodotObject *)(internal::gdextension_interface_object_get_instance_from_id(p_object_id)));
	}

public:
	_FORCE_INLINE_ rapier2d::Handle get_handle() const { return handle; }

	_FORCE_INLINE_ void set_rid(const RID &p_rid) { rid = p_rid; }
	_FORCE_INLINE_ RID get_rid() const { return rid; }

	void body_add_to_mass_properties_update_list(SelfList<RapierBody2D> *p_body);
	void body_add_to_gravity_update_list(SelfList<RapierBody2D> *p_body);

	void body_add_to_active_list(SelfList<RapierBody2D> *p_body);
	void body_add_to_state_query_list(SelfList<RapierBody2D> *p_body);

	void area_add_to_monitor_query_list(SelfList<RapierArea2D> *p_area);
	void area_add_to_area_update_list(SelfList<RapierArea2D> *p_area);
	void body_add_to_area_update_list(SelfList<RapierBody2D> *p_body);

	void add_removed_collider(rapier2d::Handle p_handle, RapierCollisionObject2D *p_object, uint32_t p_shape_index);
	bool get_removed_collider_info(rapier2d::Handle p_handle, RID &r_rid, ObjectID &r_instance_id, uint32_t &r_shape_index, RapierCollisionObject2D::Type &r_type) const;

	_FORCE_INLINE_ int get_solver_iterations() const { return solver_iterations; }
	_FORCE_INLINE_ real_t get_contact_recycle_radius() const { return contact_recycle_radius; }
	_FORCE_INLINE_ real_t get_contact_max_separation() const { return contact_max_separation; }
	_FORCE_INLINE_ real_t get_contact_max_allowed_penetration() const { return contact_max_allowed_penetration; }
	_FORCE_INLINE_ real_t get_contact_bias() const { return contact_bias; }
	_FORCE_INLINE_ real_t get_constraint_bias() const { return constraint_bias; }

	void step(real_t p_step);
	void call_queries();

	bool is_locked() const;
	void lock();
	void unlock();

	real_t get_last_step() const { return last_step; }
	void set_last_step(real_t p_step) { last_step = p_step; }

	void set_param(PhysicsServer2D::SpaceParameter p_param, real_t p_value);
	real_t get_param(PhysicsServer2D::SpaceParameter p_param) const;

	void set_default_area_param(PhysicsServer2D::AreaParameter p_param, const Variant &p_value);
	Variant get_default_area_param(PhysicsServer2D::AreaParameter p_param) const;

	void set_island_count(int p_island_count) { island_count = p_island_count; }
	int get_island_count() const { return island_count; }

	int get_active_objects() const { return active_objects; }

	int get_collision_pairs() const { return collision_pairs; }

	RapierArea2D *get_area_from_rid(RID p_area_rid) const;
	RapierBody2D *get_body_from_rid(RID p_body_rid) const;
	RapierShape2D *get_shape_from_rid(RID p_shape_rid) const;

	void set_debug_contacts(int p_amount) { contact_debug.resize(p_amount); }
	_FORCE_INLINE_ bool is_debugging_contacts() const { return !contact_debug.is_empty(); }
	_FORCE_INLINE_ void add_debug_contact(const Vector2 &p_contact) {
		if (contact_debug_count < contact_debug.size()) {
			contact_debug[contact_debug_count++] = p_contact;
		}
	}
	_FORCE_INLINE_ const PackedVector2Array &get_debug_contacts() { return contact_debug; }
	_FORCE_INLINE_ int get_debug_contact_count() { return contact_debug_count; }

	RapierDirectSpaceState2D *get_direct_state();

	bool test_body_motion(RapierBody2D *p_body, const Transform2D &p_from, const Vector2 &p_motion, double p_margin, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer2DExtensionMotionResult *r_result) const;

	int rapier_intersect_aabb(Rect2 p_aabb, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, rapier2d::PointHitInfo *p_results, int32_t p_max_results, int32_t *p_result_count, RID p_exclude_body) const;
	RapierSpace2D();
	~RapierSpace2D();
};

#endif // RAPIER_SPACE_2D_H
