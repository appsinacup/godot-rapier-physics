#ifndef RAPIER_AREA_2D_H
#define RAPIER_AREA_2D_H

#include "rapier_collision_object_2d.h"

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/templates/hash_map.hpp>

using namespace godot;

class RapierSpace2D;
class RapierBody2D;

class RapierArea2D : public RapierCollisionObject2D {
	PhysicsServer2D::AreaSpaceOverrideMode gravity_override_mode = PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED;
	PhysicsServer2D::AreaSpaceOverrideMode linear_damping_override_mode = PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED;
	PhysicsServer2D::AreaSpaceOverrideMode angular_damping_override_mode = PhysicsServer2D::AREA_SPACE_OVERRIDE_DISABLED;

	real_t gravity = 9.80665;
	Vector2 gravity_vector = Vector2(0, -1);
	bool gravity_is_point = false;
	real_t gravity_point_unit_distance = 0.0;
	real_t linear_damp = 0.1;
	real_t angular_damp = 1.0;
	int priority = 0;
	bool monitorable = false;

	Callable monitor_callback;
	Callable area_monitor_callback;

	struct MonitorInfo {
		RID rid;
		ObjectID instance_id;
		uint32_t object_shape_index = 0;
		uint32_t area_shape_index = 0;
		Type type = Type::TYPE_BODY;
		int state = 0;
	};

	HashMap<uint64_t, MonitorInfo> monitored_objects;

	struct BodyRefCount {
		uint32_t count = 0;
	};
	HashMap<RID, BodyRefCount> detected_bodies;

	SelfList<RapierArea2D> monitor_query_list;
	SelfList<RapierArea2D> area_override_update_list;

	virtual void _shapes_changed() override {}

	void _set_space_override_mode(PhysicsServer2D::AreaSpaceOverrideMode &r_mode, PhysicsServer2D::AreaSpaceOverrideMode p_value);
	void _enable_space_override();
	void _disable_space_override();
	void _reset_space_override();

public:
	void on_body_enter(rapier2d::Handle p_collider_handle, RapierBody2D *p_body, uint32_t p_body_shape, RID p_body_rid, ObjectID p_body_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape);
	void on_body_exit(rapier2d::Handle p_collider_handle, RapierBody2D *p_body, uint32_t p_body_shape, RID p_body_rid, ObjectID p_body_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape, bool p_update_detection = true);

	void on_area_enter(rapier2d::Handle p_collider_handle, RapierArea2D *p_other_area, uint32_t p_other_area_shape, RID p_other_area_rid, ObjectID p_other_area_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape);
	void on_area_exit(rapier2d::Handle p_collider_handle, RapierArea2D *p_other_area, uint32_t p_other_area_shape, RID p_other_area_rid, ObjectID p_other_area_instance_id, rapier2d::Handle p_area_collider_handle, uint32_t p_area_shape);

	void update_area_override();
	bool has_any_space_override() const;

	void set_monitor_callback(const Callable &p_callback);
	_FORCE_INLINE_ bool has_monitor_callback() const { return !monitor_callback.is_null(); }

	void set_area_monitor_callback(const Callable &p_callback);
	_FORCE_INLINE_ bool has_area_monitor_callback() const { return !area_monitor_callback.is_null(); }

	void set_param(PhysicsServer2D::AreaParameter p_param, const Variant &p_value);
	Variant get_param(PhysicsServer2D::AreaParameter p_param) const;

	_FORCE_INLINE_ void set_gravity(real_t p_gravity) { gravity = p_gravity; }
	_FORCE_INLINE_ real_t get_gravity() const { return gravity; }

	_FORCE_INLINE_ void set_gravity_vector(const Vector2 &p_gravity) { gravity_vector = p_gravity; }
	_FORCE_INLINE_ Vector2 get_gravity_vector() const { return gravity_vector; }

	_FORCE_INLINE_ void set_gravity_as_point(bool p_enable) { gravity_is_point = p_enable; }
	_FORCE_INLINE_ bool is_gravity_point() const { return gravity_is_point; }

	_FORCE_INLINE_ void set_gravity_point_unit_distance(real_t scale) { gravity_point_unit_distance = scale; }
	_FORCE_INLINE_ real_t get_gravity_point_unit_distance() const { return gravity_point_unit_distance; }

	_FORCE_INLINE_ void set_linear_damp(real_t p_linear_damp) { linear_damp = p_linear_damp; }
	_FORCE_INLINE_ real_t get_linear_damp() const { return linear_damp; }

	_FORCE_INLINE_ void set_angular_damp(real_t p_angular_damp) { angular_damp = p_angular_damp; }
	_FORCE_INLINE_ real_t get_angular_damp() const { return angular_damp; }

	_FORCE_INLINE_ void set_priority(int p_priority) { priority = p_priority; }
	_FORCE_INLINE_ int get_priority() const { return priority; }

	void set_monitorable(bool p_monitorable);
	_FORCE_INLINE_ bool is_monitorable() const { return monitorable; }

	void set_space(RapierSpace2D *p_space) override;

	void call_queries();

	void compute_gravity(const Vector2 &p_position, Vector2 &r_gravity) const;

	RapierArea2D();
	~RapierArea2D();
};

#endif // RAPIER_AREA_2D_H
