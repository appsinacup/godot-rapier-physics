#include "rapier_damped_spring_joint_2d.h"

void RapierDampedSpringJoint2D::set_param(PhysicsServer2D::DampedSpringParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer2D::DAMPED_SPRING_REST_LENGTH: {
			rest_length = p_value;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_DAMPING: {
			damping = p_value;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_STIFFNESS: {
			stiffness = p_value;
		} break;
	}
}

real_t RapierDampedSpringJoint2D::get_param(PhysicsServer2D::DampedSpringParam p_param) const {
	switch (p_param) {
		case PhysicsServer2D::DAMPED_SPRING_REST_LENGTH: {
			return rest_length;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_DAMPING: {
			return damping;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_STIFFNESS: {
			return stiffness;
		} break;
	}

	ERR_FAIL_V(0);
}

RapierDampedSpringJoint2D::RapierDampedSpringJoint2D(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, RapierBody2D *p_body_a, RapierBody2D *p_body_b) :
		RapierJoint2D(p_body_a, p_body_b) {
	Vector2 anchor_A = A->get_inv_transform().xform(p_anchor_a);
	Vector2 anchor_B = B->get_inv_transform().xform(p_anchor_b);

	rest_length = p_anchor_a.distance_to(p_anchor_b);

	// TODO: create rapier joint when available
	// See https://github.com/dimforge/rapier/issues/241
	ERR_FAIL_MSG("Spring joints not supported for now");

	//A->add_constraint(this, 0);
	//B->add_constraint(this, 1);
}
