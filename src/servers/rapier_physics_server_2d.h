#ifndef RAPIER_PHYSICS_SERVER_2D_H
#define RAPIER_PHYSICS_SERVER_2D_H

#include <godot_cpp/classes/physics_direct_body_state2d.hpp>
#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/physics_server2d_extension.hpp>
#include <godot_cpp/classes/physics_test_motion_parameters2d.hpp>
#include <godot_cpp/classes/physics_test_motion_result2d.hpp>
#include <godot_cpp/variant/callable.hpp>

#include <godot_cpp/core/binder_common.hpp>

#include <godot_cpp/templates/rid_owner.hpp>

#include "../bodies/rapier_area_2d.h"
#include "../bodies/rapier_body_2d.h"
#include "../fluids/rapier_fluid_2d.h"
#include "../joints/rapier_joint_2d.h"
#include "../shapes/rapier_shape_2d.h"
#include "../spaces/rapier_space_2d.h"

#include "../rapier_include.h"

using namespace godot;

class RapierPhysicsServer2D : public PhysicsServer2DExtension {
	GDCLASS(RapierPhysicsServer2D, PhysicsServer2DExtension);

	friend class RapierSpace2D;

	bool active = true;
	bool doing_sync = false;

	int frame = 0;

	int island_count = 0;
	int active_objects = 0;
	int collision_pairs = 0;

	bool using_threads = false;

	bool flushing_queries = false;

	HashMap<uint32_t, RapierSpace2D *> active_spaces;

	mutable RID_PtrOwner<RapierShape2D, true> shape_owner;
	mutable RID_PtrOwner<RapierSpace2D, true> space_owner;
	mutable RID_PtrOwner<RapierArea2D, true> area_owner;
	mutable RID_PtrOwner<RapierBody2D, true> body_owner;
	mutable RID_PtrOwner<RapierJoint2D, true> joint_owner;
	mutable RID_PtrOwner<RapierFluid2D, true> fluid_owner;

	RID _shape_create(ShapeType p_shape);

protected:
	static void _bind_methods();

public:
	static RapierPhysicsServer2D *singleton;

	struct CollCbkData {
		Vector2 valid_dir;
		real_t valid_depth = 0.0;
		int max = 0;
		int amount = 0;
		int passed = 0;
		int invalid_by_dir = 0;
		Vector2 *ptr = nullptr;
	};

	virtual RID _world_boundary_shape_create() override;
	virtual RID _separation_ray_shape_create() override;
	virtual RID _segment_shape_create() override;
	virtual RID _circle_shape_create() override;
	virtual RID _rectangle_shape_create() override;
	virtual RID _capsule_shape_create() override;
	virtual RID _convex_polygon_shape_create() override;
	virtual RID _concave_polygon_shape_create() override;

	static void _shape_col_cbk(const Vector2 &p_point_A, const Vector2 &p_point_B, void *p_userdata);

	virtual void _shape_set_data(const RID &p_shape, const Variant &p_data) override;
	virtual void _shape_set_custom_solver_bias(const RID &p_shape, double p_bias) override;

	virtual ShapeType _shape_get_type(const RID &p_shape) const override;
	virtual Variant _shape_get_data(const RID &p_shape) const override;
	virtual double _shape_get_custom_solver_bias(const RID &p_shape) const override;

	virtual bool _shape_collide(const RID &p_shape_A, const Transform2D &p_xform_A, const Vector2 &p_motion_A, const RID &p_shape_B, const Transform2D &p_xform_B, const Vector2 &p_motion_B, void *r_results, int32_t p_result_max, int32_t *p_result_count) override;

	/* SPACE API */

	virtual RID _space_create() override;
	virtual void _space_set_active(const RID &p_space, bool p_active) override;
	virtual bool _space_is_active(const RID &p_space) const override;

	virtual void _space_set_param(const RID &p_space, SpaceParameter p_param, double p_value) override;
	virtual double _space_get_param(const RID &p_space, SpaceParameter p_param) const override;

	virtual void _space_set_debug_contacts(const RID &p_space, int p_max_contacts) override;
	virtual PackedVector2Array _space_get_contacts(const RID &p_space) const override;
	virtual int _space_get_contact_count(const RID &p_space) const override;

	// this function only works on physics process, errors and returns null otherwise
	virtual PhysicsDirectSpaceState2D *_space_get_direct_state(const RID &p_space) override;

	/* AREA API */

	virtual RID _area_create() override;

	virtual void _area_set_space(const RID &p_area, const RID &p_space) override;
	virtual RID _area_get_space(const RID &p_area) const override;

	virtual void _area_add_shape(const RID &p_area, const RID &p_shape, const Transform2D &p_transform = Transform2D(), bool p_disabled = false) override;
	virtual void _area_set_shape(const RID &p_area, int p_shape_idx, const RID &p_shape) override;
	virtual void _area_set_shape_transform(const RID &p_area, int p_shape_idx, const Transform2D &p_transform) override;

	virtual int _area_get_shape_count(const RID &p_area) const override;
	virtual RID _area_get_shape(const RID &p_area, int p_shape_idx) const override;
	virtual Transform2D _area_get_shape_transform(const RID &p_area, int p_shape_idx) const override;

	virtual void _area_set_shape_disabled(const RID &p_area, int p_shape, bool p_disabled) override;

	virtual void _area_remove_shape(const RID &p_area, int p_shape_idx) override;
	virtual void _area_clear_shapes(const RID &p_area) override;

	virtual void _area_attach_object_instance_id(const RID &p_area, uint64_t p_id) override;
	virtual uint64_t _area_get_object_instance_id(const RID &p_area) const override;

	virtual void _area_attach_canvas_instance_id(const RID &p_area, uint64_t p_id) override;
	virtual uint64_t _area_get_canvas_instance_id(const RID &p_area) const override;

	virtual void _area_set_param(const RID &p_area, AreaParameter p_param, const Variant &p_value) override;
	virtual void _area_set_transform(const RID &p_area, const Transform2D &p_transform) override;

	virtual Variant _area_get_param(const RID &p_area, AreaParameter p_param) const override;
	virtual Transform2D _area_get_transform(const RID &p_area) const override;

	virtual void _area_set_monitorable(const RID &p_area, bool p_monitorable) override;

	virtual void _area_set_collision_layer(const RID &p_area, uint32_t p_layer) override;
	virtual uint32_t _area_get_collision_layer(const RID &p_area) const override;

	virtual void _area_set_collision_mask(const RID &p_area, uint32_t p_mask) override;
	virtual uint32_t _area_get_collision_mask(const RID &p_area) const override;

	virtual void _area_set_monitor_callback(const RID &p_area, const Callable &p_callback) override;
	virtual void _area_set_area_monitor_callback(const RID &p_area, const Callable &p_callback) override;

	virtual void _area_set_pickable(const RID &p_area, bool p_pickable) override;

	/* BODY API */

	virtual RID _body_create() override;

	virtual void _body_set_space(const RID &p_body, const RID &p_space) override;
	virtual RID _body_get_space(const RID &p_body) const override;

	virtual void _body_set_mode(const RID &p_body, BodyMode p_mode) override;
	virtual BodyMode _body_get_mode(const RID &p_body) const override;

	virtual void _body_add_shape(const RID &p_body, const RID &p_shape, const Transform2D &p_transform = Transform2D(), bool p_disabled = false) override;
	virtual void _body_set_shape(const RID &p_body, int p_shape_idx, const RID &p_shape) override;
	virtual void _body_set_shape_transform(const RID &p_body, int p_shape_idx, const Transform2D &p_transform) override;

	virtual int _body_get_shape_count(const RID &p_body) const override;
	virtual RID _body_get_shape(const RID &p_body, int p_shape_idx) const override;
	virtual Transform2D _body_get_shape_transform(const RID &p_body, int p_shape_idx) const override;

	virtual void _body_remove_shape(const RID &p_body, int p_shape_idx) override;
	virtual void _body_clear_shapes(const RID &p_body) override;

	virtual void _body_set_shape_disabled(const RID &p_body, int p_shape_idx, bool p_disabled) override;
	virtual void _body_set_shape_as_one_way_collision(const RID &p_body, int p_shape_idx, bool p_enable, double p_margin) override;

	virtual void _body_attach_object_instance_id(const RID &p_body, uint64_t p_id) override;
	virtual uint64_t _body_get_object_instance_id(const RID &p_body) const override;

	virtual void _body_attach_canvas_instance_id(const RID &p_body, uint64_t p_id) override;
	virtual uint64_t _body_get_canvas_instance_id(const RID &p_body) const override;

	virtual void _body_set_continuous_collision_detection_mode(const RID &p_body, CCDMode p_mode) override;
	virtual CCDMode _body_get_continuous_collision_detection_mode(const RID &p_body) const override;

	virtual void _body_set_collision_layer(const RID &p_body, uint32_t p_layer) override;
	virtual uint32_t _body_get_collision_layer(const RID &p_body) const override;

	virtual void _body_set_collision_mask(const RID &p_body, uint32_t p_mask) override;
	virtual uint32_t _body_get_collision_mask(const RID &p_body) const override;

	virtual void _body_set_collision_priority(const RID &p_body, double p_priority) override;
	virtual double _body_get_collision_priority(const RID &p_body) const override;

	virtual void _body_set_param(const RID &p_body, BodyParameter p_param, const Variant &p_value) override;
	virtual Variant _body_get_param(const RID &p_body, BodyParameter p_param) const override;

	virtual void _body_reset_mass_properties(const RID &p_body) override;

	virtual void _body_set_state(const RID &p_body, BodyState p_state, const Variant &p_variant) override;
	virtual Variant _body_get_state(const RID &p_body, BodyState p_state) const override;

	virtual void _body_apply_central_impulse(const RID &p_body, const Vector2 &p_impulse) override;
	virtual void _body_apply_torque_impulse(const RID &p_body, double p_torque) override;
	virtual void _body_apply_impulse(const RID &p_body, const Vector2 &p_impulse, const Vector2 &p_position = Vector2()) override;

	virtual void _body_apply_central_force(const RID &p_body, const Vector2 &p_force) override;
	virtual void _body_apply_force(const RID &p_body, const Vector2 &p_force, const Vector2 &p_position = Vector2()) override;
	virtual void _body_apply_torque(const RID &p_body, double p_torque) override;

	virtual void _body_add_constant_central_force(const RID &p_body, const Vector2 &p_force) override;
	virtual void _body_add_constant_force(const RID &p_body, const Vector2 &p_force, const Vector2 &p_position = Vector2()) override;
	virtual void _body_add_constant_torque(const RID &p_body, double p_torque) override;

	virtual void _body_set_constant_force(const RID &p_body, const Vector2 &p_force) override;
	virtual Vector2 _body_get_constant_force(const RID &p_body) const override;

	virtual void _body_set_constant_torque(const RID &p_body, double p_torque) override;
	virtual double _body_get_constant_torque(const RID &p_body) const override;

	virtual void _body_set_axis_velocity(const RID &p_body, const Vector2 &p_axis_velocity) override;

	virtual void _body_add_collision_exception(const RID &p_body, const RID &p_body_b) override;
	virtual void _body_remove_collision_exception(const RID &p_body, const RID &p_body_b) override;
	virtual TypedArray<RID> _body_get_collision_exceptions(const RID &p_body) const override;

	virtual void _body_set_contacts_reported_depth_threshold(const RID &p_body, double p_threshold) override;
	virtual double _body_get_contacts_reported_depth_threshold(const RID &p_body) const override;

	virtual void _body_set_omit_force_integration(const RID &p_body, bool p_omit) override;
	virtual bool _body_is_omitting_force_integration(const RID &p_body) const override;

	virtual void _body_set_max_contacts_reported(const RID &p_body, int p_contacts) override;
	virtual int _body_get_max_contacts_reported(const RID &p_body) const override;

	virtual void _body_set_state_sync_callback(const RID &p_body, const Callable &p_callable) override;
	virtual void _body_set_force_integration_callback(const RID &p_body, const Callable &p_callable, const Variant &p_udata = Variant()) override;

	virtual bool _body_collide_shape(const RID &p_body, int32_t p_body_shape, const RID &p_shape, const Transform2D &p_shape_xform, const Vector2 &p_motion, void *r_results, int32_t p_result_max, int32_t *r_result_count) override;

	virtual void _body_set_pickable(const RID &p_body, bool p_pickable) override;

	virtual bool _body_test_motion(const RID &p_body, const Transform2D &p_from, const Vector2 &p_motion, double p_margin, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer2DExtensionMotionResult *r_result) const override;

	// this function only works on physics process, errors and returns null otherwise
	virtual PhysicsDirectBodyState2D *_body_get_direct_state(const RID &p_body) override;

	/* JOINT API */

	virtual RID _joint_create() override;

	virtual void _joint_clear(const RID &p_joint) override;

	virtual void _joint_set_param(const RID &p_joint, JointParam p_param, double p_value) override;
	virtual double _joint_get_param(const RID &p_joint, JointParam p_param) const override;

	virtual void _joint_disable_collisions_between_bodies(const RID &p_joint, const bool p_disabled) override;
	virtual bool _joint_is_disabled_collisions_between_bodies(const RID &p_joint) const override;

	virtual void _joint_make_pin(const RID &p_joint, const Vector2 &p_anchor, const RID &p_body_a, const RID &p_body_b = RID()) override;
	virtual void _joint_make_groove(const RID &p_joint, const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, const RID &p_body_a, const RID &p_body_b) override;
	virtual void _joint_make_damped_spring(const RID &p_joint, const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, const RID &p_body_a, const RID &p_body_b = RID()) override;

	virtual void _pin_joint_set_flag(const RID &joint, PhysicsServer2D::PinJointFlag flag, bool enabled) override;
	virtual bool _pin_joint_get_flag(const RID &joint, PhysicsServer2D::PinJointFlag flag) const override;
	virtual void _pin_joint_set_param(const RID &p_joint, PinJointParam p_param, double p_value) override;
	virtual double _pin_joint_get_param(const RID &p_joint, PinJointParam p_param) const override;
	virtual void _damped_spring_joint_set_param(const RID &p_joint, DampedSpringParam p_param, double p_value) override;
	virtual double _damped_spring_joint_get_param(const RID &p_joint, DampedSpringParam p_param) const override;

	virtual JointType _joint_get_type(const RID &p_joint) const override;

	/* LIQUID API */

	RID fluid_create();
	void fluid_set_space(const RID &fluid_rid, const RID &space_rid);
	void fluid_set_density(const RID &fluid_rid, real_t density);
	/* MISC */

	virtual void _free_rid(const RID &p_rid) override;

	virtual void _set_active(bool p_active) override;
	virtual void _init() override;
	virtual void _step(double p_step) override;
	virtual void _sync() override;
	virtual void _flush_queries() override;
	virtual void _end_sync() override;
	virtual void _finish() override;

	virtual bool _is_flushing_queries() const override { return flushing_queries; }

	virtual int _get_process_info(ProcessInfo p_info) override;

	int get_frame() { return frame; }

	RapierSpace2D *get_active_space(rapier2d::Handle p_handle) const {
		ERR_FAIL_COND_V(!rapier2d::is_handle_valid(p_handle), nullptr);
		return active_spaces.get(rapier2d::handle_hash(p_handle));
	}

	RapierPhysicsServer2D(bool p_using_threads = false);
	~RapierPhysicsServer2D() {}
};

class RapierPhysicsServer2DFactory : public Object {
	GDCLASS(RapierPhysicsServer2DFactory, Object);

protected:
	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("create_rapier_2d_callback"), &RapierPhysicsServer2DFactory::_create_rapier_2d_callback);
	}

public:
	PhysicsServer2D *_create_rapier_2d_callback() {
		PhysicsServer2D *physics_server_2d = memnew(RapierPhysicsServer2D());
		return physics_server_2d;
	}
};

#endif // RAPIER_PHYSICS_SERVER_2D_H
