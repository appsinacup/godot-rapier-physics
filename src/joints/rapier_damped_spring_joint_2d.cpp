#include "rapier_damped_spring_joint_2d.h"
#include "../spaces/rapier_space_2d.h"

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
	rapier2d::joint_change_spring_params(space_handle, handle, rest_length, damping, stiffness);
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

	rapier2d::Vector rapier_anchor_A = { anchor_A.x, anchor_A.y };
	rapier2d::Vector rapier_anchor_B = { anchor_B.x, anchor_B.y };

	ERR_FAIL_COND(!p_body_a->get_space());
	ERR_FAIL_COND(p_body_a->get_space() != p_body_b->get_space());
	space_handle = p_body_a->get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	handle = rapier2d::joint_create_spring(space_handle, p_body_a->get_body_handle(), p_body_b->get_body_handle(), &rapier_anchor_A, &rapier_anchor_B, stiffness, damping, rest_length, is_disabled_collisions_between_bodies());
	ERR_FAIL_COND(!rapier2d::is_handle_valid(handle));
}
