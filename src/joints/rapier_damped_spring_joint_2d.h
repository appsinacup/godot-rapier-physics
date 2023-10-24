#ifndef RAPIER_DAMPED_SPRING_JOINT_2D_H
#define RAPIER_DAMPED_SPRING_JOINT_2D_H

#include "rapier_joint_2d.h"

class RapierDampedSpringJoint2D : public RapierJoint2D {
	real_t rest_length = 0.0;
	real_t damping = 1.5;
	real_t stiffness = 20.0;

public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_DAMPED_SPRING; }

	void set_param(PhysicsServer2D::DampedSpringParam p_param, real_t p_value);
	real_t get_param(PhysicsServer2D::DampedSpringParam p_param) const;

	RapierDampedSpringJoint2D(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, RapierBody2D *p_body_a, RapierBody2D *p_body_b);
};

#endif // RAPIER_DAMPED_SPRING_JOINT_2D_H
