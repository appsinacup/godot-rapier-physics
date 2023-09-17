#ifndef RAPIER_JOINTS_2D_H
#define RAPIER_JOINTS_2D_H

#include "rapier_body_2d.h"
#include "rapier_constraint_2d.h"

using namespace godot;

class RapierJoint2D : public RapierConstraint2D {
	real_t bias = 0;
	real_t max_bias = 3.40282e+38;
	real_t max_force = 3.40282e+38;

protected:
	rapier2d::Handle space_handle = rapier2d::invalid_handle();
	rapier2d::Handle handle = rapier2d::invalid_handle();

public:
	_FORCE_INLINE_ void set_max_force(real_t p_force) { max_force = p_force; }
	_FORCE_INLINE_ real_t get_max_force() const { return max_force; }

	_FORCE_INLINE_ void set_bias(real_t p_bias) { bias = p_bias; }
	_FORCE_INLINE_ real_t get_bias() const { return bias; }

	_FORCE_INLINE_ void set_max_bias(real_t p_bias) { max_bias = p_bias; }
	_FORCE_INLINE_ real_t get_max_bias() const { return max_bias; }

	void copy_settings_from(RapierJoint2D *p_joint);

	virtual PhysicsServer2D::JointType get_type() const { return PhysicsServer2D::JOINT_TYPE_MAX; }
	RapierJoint2D(RapierBody2D **p_body_ptr = nullptr, int p_body_count = 0) :
			RapierConstraint2D(p_body_ptr, p_body_count) {}

	virtual ~RapierJoint2D();
};

class RapierPinJoint2D : public RapierJoint2D {
	union {
		struct {
			RapierBody2D *A;
			RapierBody2D *B;
		};

		RapierBody2D *_arr[2] = { nullptr, nullptr };
	};

public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_PIN; }

	RapierPinJoint2D(const Vector2 &p_pos, RapierBody2D *p_body_a, RapierBody2D *p_body_b = nullptr);
};

class RapierGrooveJoint2D : public RapierJoint2D {
	union {
		struct {
			RapierBody2D *A;
			RapierBody2D *B;
		};

		RapierBody2D *_arr[2] = { nullptr, nullptr };
	};

public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_GROOVE; }

	RapierGrooveJoint2D(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, RapierBody2D *p_body_a, RapierBody2D *p_body_b);
};

class RapierDampedSpringJoint2D : public RapierJoint2D {
	union {
		struct {
			RapierBody2D *A;
			RapierBody2D *B;
		};

		RapierBody2D *_arr[2] = { nullptr, nullptr };
	};

	real_t rest_length = 0.0;
	real_t damping = 1.5;
	real_t stiffness = 20.0;

public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_DAMPED_SPRING; }

	void set_param(PhysicsServer2D::DampedSpringParam p_param, real_t p_value);
	real_t get_param(PhysicsServer2D::DampedSpringParam p_param) const;

	RapierDampedSpringJoint2D(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, RapierBody2D *p_body_a, RapierBody2D *p_body_b);
};

#endif // RAPIER_JOINTS_2D_H
