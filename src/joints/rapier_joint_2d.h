#ifndef RAPIER_JOINT_2D_H
#define RAPIER_JOINT_2D_H

#include "../bodies/rapier_body_2d.h"

using namespace godot;

class RapierJoint2D {
	real_t bias = 0;
	real_t max_bias = 3.40282e+38;
	real_t max_force = 3.40282e+38;

	bool disabled_collisions_between_bodies = true;
	RID rid;

protected:
	RapierBody2D *A;
	RapierBody2D *B;

	rapier2d::Handle space_handle = rapier2d::invalid_handle();
	rapier2d::Handle handle = rapier2d::invalid_handle();

public:
	_FORCE_INLINE_ void set_rid(const RID &p_rid) { rid = p_rid; }
	_FORCE_INLINE_ RID get_rid() const { return rid; }

	_FORCE_INLINE_ void disable_collisions_between_bodies(const bool p_disabled) { disabled_collisions_between_bodies = p_disabled; }
	_FORCE_INLINE_ bool is_disabled_collisions_between_bodies() const { return disabled_collisions_between_bodies; }

	_FORCE_INLINE_ void set_max_force(real_t p_force) { max_force = p_force; }
	_FORCE_INLINE_ real_t get_max_force() const { return max_force; }

	_FORCE_INLINE_ void set_bias(real_t p_bias) { bias = p_bias; }
	_FORCE_INLINE_ real_t get_bias() const { return bias; }

	_FORCE_INLINE_ void set_max_bias(real_t p_bias) { max_bias = p_bias; }
	_FORCE_INLINE_ real_t get_max_bias() const { return max_bias; }

	void copy_settings_from(RapierJoint2D *p_joint);

	virtual PhysicsServer2D::JointType get_type() const { return PhysicsServer2D::JOINT_TYPE_MAX; }

	RapierJoint2D(RapierBody2D *p_body_a, RapierBody2D *p_body_b = nullptr);
	RapierJoint2D() {}
	virtual ~RapierJoint2D();
};

#endif // RAPIER_JOINT_2D_H
