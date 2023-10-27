#ifndef RAPIER_BODY_2D_H
#define RAPIER_BODY_2D_H

#include "rapier_area_2d.h"
#include "rapier_collision_object_2d.h"

#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/templates/vset.hpp>

using namespace godot;

class RapierDirectBodyState2D;

class RapierBody2D : public RapierCollisionObject2D {
	PhysicsServer2D::BodyMode mode = PhysicsServer2D::BODY_MODE_RIGID;

	PhysicsServer2D::BodyDampMode linear_damping_mode = PhysicsServer2D::BODY_DAMP_MODE_COMBINE;
	PhysicsServer2D::BodyDampMode angular_damping_mode = PhysicsServer2D::BODY_DAMP_MODE_COMBINE;

	real_t linear_damping = 0.0;
	real_t angular_damping = 0.0;

	real_t total_linear_damping = 0.0;
	real_t total_angular_damping = 0.0;

	Vector2 total_gravity;
	real_t gravity_scale = 1.0;

	real_t bounce = 0.0;
	real_t friction = 1.0;

	real_t mass = 1.0;
	real_t inertia = 0.0;
	Vector2 center_of_mass;

	bool calculate_inertia = true;
	bool calculate_center_of_mass = true;

	bool using_area_gravity = false;
	bool using_area_linear_damping = false;
	bool using_area_angular_damping = false;

	VSet<RID> exceptions;
	PhysicsServer2D::CCDMode ccd_mode = PhysicsServer2D::CCD_MODE_DISABLED;
	bool omit_force_integration = false;
	bool active = true;
	bool marked_active = false;
	bool can_sleep = true;

	Vector2 constant_force;
	Vector2 linear_velocity;
	Vector2 impulse;
	real_t torque = 0.0;
	real_t angular_velocity = 0.0;
	real_t constant_torque = 0.0;

	bool sleep = false;

	void _mass_properties_changed();
	void _apply_mass_properties(bool force_update = false);

	virtual void _shapes_changed() override;

	SelfList<RapierBody2D> active_list;
	SelfList<RapierBody2D> mass_properties_update_list;
	SelfList<RapierBody2D> gravity_update_list;

	//List<Pair<RapierConstraint2D *, int>> constraint_list;

	struct AreaCMP {
		RapierArea2D *area = nullptr;
		_FORCE_INLINE_ bool operator==(const AreaCMP &p_cmp) const { return area->get_rid() == p_cmp.area->get_rid(); }
		_FORCE_INLINE_ bool operator<(const AreaCMP &p_cmp) const { return area->get_priority() < p_cmp.area->get_priority(); }
		_FORCE_INLINE_ AreaCMP() {}
		_FORCE_INLINE_ AreaCMP(RapierArea2D *p_area) {
			area = p_area;
		}
	};

	LocalVector<AreaCMP> areas;
	SelfList<RapierBody2D> area_override_update_list;

	struct Contact {
		Vector2 local_pos;
		Vector2 local_normal;
		real_t depth = 0.0;
		int local_shape = 0;
		Vector2 collider_pos;
		int collider_shape = 0;
		ObjectID collider_instance_id;
		RID collider;
		Vector2 collider_velocity_at_pos;
		Vector2 impulse;
	};

	LocalVector<Contact> contacts; //no contacts by default
	int contact_count = 0;

	Callable body_state_callback;

	struct ForceIntegrationCallbackData {
		Callable callable;
		Variant udata;
	};

	ForceIntegrationCallbackData *fi_callback_data = nullptr;

	RapierDirectBodyState2D *direct_state = nullptr;

	SelfList<RapierBody2D> direct_state_query_list;

	friend class RapierDirectBodyState2D; // i give up, too many functions to expose

	void _apply_linear_damping(real_t new_value, bool apply_default = true);
	void _apply_angular_damping(real_t new_value, bool apply_default = true);

	void _apply_gravity_scale(real_t new_value);

protected:
	virtual void _init_material(rapier2d::Material &mat) const override;
	virtual void _init_collider(rapier2d::Handle collider_handle) const override;

public:
	void set_linear_velocity(const Vector2 &linear_velocity);
	Vector2 get_linear_velocity() const;

	void set_angular_velocity(real_t angular_velocity);
	real_t get_angular_velocity() const;
	
	void set_state_sync_callback(const Callable &p_callable);
	void set_force_integration_callback(const Callable &p_callable, const Variant &p_udata = Variant());

	RapierDirectBodyState2D *get_direct_state();

	void add_area(RapierArea2D *p_area);
	void remove_area(RapierArea2D *p_area);
	void on_area_updated(RapierArea2D *p_area);

	void update_area_override();

	void update_gravity(real_t p_step);

	_FORCE_INLINE_ void set_max_contacts_reported(int p_size) {
		contacts.resize(p_size);
		contact_count = 0;
	}
	_FORCE_INLINE_ int get_max_contacts_reported() const { return contacts.size(); }
	_FORCE_INLINE_ bool can_report_contacts() const { return !contacts.is_empty(); }
	_FORCE_INLINE_ void add_contact(const Vector2 &p_local_pos, const Vector2 &p_local_normal, real_t p_depth, int p_local_shape, const Vector2 &p_collider_pos, int p_collider_shape, ObjectID p_collider_instance_id, const RID &p_collider, const Vector2 &p_collider_velocity_at_pos, const Vector2 &p_impulse);

	_FORCE_INLINE_ void add_exception(const RID &p_exception) { exceptions.insert(p_exception); }
	_FORCE_INLINE_ void remove_exception(const RID &p_exception) { exceptions.erase(p_exception); }
	_FORCE_INLINE_ bool has_exception(const RID &p_exception) const { return exceptions.has(p_exception); }
	_FORCE_INLINE_ const VSet<RID> &get_exceptions() const { return exceptions; }

	_FORCE_INLINE_ void set_omit_force_integration(bool p_omit_force_integration) { omit_force_integration = p_omit_force_integration; }
	_FORCE_INLINE_ bool get_omit_force_integration() const { return omit_force_integration; }

	void apply_central_impulse(const Vector2 &p_impulse);

	void apply_impulse(const Vector2 &p_impulse, const Vector2 &p_position = Vector2());

	void apply_torque_impulse(real_t p_torque);

	void apply_central_force(const Vector2 &p_force);

	void apply_force(const Vector2 &p_force, const Vector2 &p_position = Vector2());

	void apply_torque(real_t p_torque);

	void add_constant_central_force(const Vector2 &p_force);

	void add_constant_force(const Vector2 &p_force, const Vector2 &p_position = Vector2());

	void add_constant_torque(real_t p_torque);

	void set_constant_force(const Vector2 &p_force);
	Vector2 get_constant_force() const;

	void set_constant_torque(real_t p_torque);
	real_t get_constant_torque() const;

	void set_active(bool p_active);
	_FORCE_INLINE_ bool is_active() const { return active; }

	void set_can_sleep(bool p_can_sleep);

	void on_marked_active();
	void on_update_active();

	void wakeup();
	void force_sleep();

	void set_param(PhysicsServer2D::BodyParameter p_param, const Variant &p_value);
	Variant get_param(PhysicsServer2D::BodyParameter p_param) const;

	void set_mode(PhysicsServer2D::BodyMode p_mode);
	PhysicsServer2D::BodyMode get_mode() const;

	void set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer2D::BodyState p_state) const;

	void set_continuous_collision_detection_mode(PhysicsServer2D::CCDMode p_mode);
	_FORCE_INLINE_ PhysicsServer2D::CCDMode get_continuous_collision_detection_mode() const { return ccd_mode; }

	void set_space(RapierSpace2D *p_space) override;

	void update_mass_properties(bool force_update = false);
	void reset_mass_properties();

	_FORCE_INLINE_ const Vector2 &get_center_of_mass() const { return center_of_mass; }
	_FORCE_INLINE_ real_t get_mass() const { return mass; }
	_FORCE_INLINE_ real_t get_inv_mass() const { return mass ? 1.0f / mass : 0.0f; }
	_FORCE_INLINE_ real_t get_inertia() const { return inertia; }
	_FORCE_INLINE_ real_t get_inv_inertia() const { return inertia ? 1.0f / inertia : 0.0f; }
	_FORCE_INLINE_ real_t get_friction() const { return friction; }
	_FORCE_INLINE_ real_t get_bounce() const { return bounce; }

	_FORCE_INLINE_ Vector2 get_velocity_at_local_point(const Vector2 &rel_pos) const {
		Vector2 linear_velocity = get_linear_velocity();
		real_t angular_velocity = get_angular_velocity();
		return linear_velocity + Vector2(-angular_velocity * (rel_pos.y - center_of_mass.y), angular_velocity * (rel_pos.x - center_of_mass.x));
	}

	void call_queries();

	Rect2 get_aabb();

	RapierBody2D();
	~RapierBody2D();
};

void RapierBody2D::add_contact(const Vector2 &p_local_pos, const Vector2 &p_local_normal, real_t p_depth, int p_local_shape, const Vector2 &p_collider_pos, int p_collider_shape, ObjectID p_collider_instance_id, const RID &p_collider, const Vector2 &p_collider_velocity_at_pos, const Vector2 &p_impulse) {
	int c_max = contacts.size();

	if (c_max == 0) {
		return;
	}

	Contact *c = contacts.ptr();

	int idx = -1;

	if (contact_count < c_max) {
		idx = contact_count++;
	} else {
		real_t least_depth = 1e20;
		int least_deep = -1;
		for (int i = 0; i < c_max; i++) {
			if (i == 0 || c[i].depth < least_depth) {
				least_deep = i;
				least_depth = c[i].depth;
			}
		}

		if (least_deep >= 0 && least_depth < p_depth) {
			idx = least_deep;
		}
		if (idx == -1) {
			return; //none less deep than this
		}
	}

	c[idx].local_pos = p_local_pos;
	c[idx].local_normal = p_local_normal;
	c[idx].depth = p_depth;
	c[idx].local_shape = p_local_shape;
	c[idx].collider_pos = p_collider_pos;
	c[idx].collider_shape = p_collider_shape;
	c[idx].collider_instance_id = p_collider_instance_id;
	c[idx].collider = p_collider;
	c[idx].collider_velocity_at_pos = p_collider_velocity_at_pos;
	c[idx].impulse = p_impulse;
}

#endif // RAPIER_BODY_2D_H
