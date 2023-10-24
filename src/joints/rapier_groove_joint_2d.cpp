#include "rapier_groove_joint_2d.h"
#include "../spaces/rapier_space_2d.h"

RapierGrooveJoint2D::RapierGrooveJoint2D(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, RapierBody2D *p_body_a, RapierBody2D *p_body_b) :
		RapierJoint2D(p_body_a, p_body_b) {
	Vector2 point_A_1 = A->get_inv_transform().xform(p_a_groove1);
	Vector2 point_A_2 = A->get_inv_transform().xform(p_a_groove2);

	Vector2 anchor_B = B->get_inv_transform().xform(p_b_anchor);

	Vector2 axis = (point_A_2 - point_A_1).normalized();
	real_t length = (point_A_2 - point_A_1).length();

	rapier2d::Vector rapier_anchor_A = { point_A_1.x, point_A_1.y };
	rapier2d::Vector rapier_anchor_B = { anchor_B.x, anchor_B.y };

	rapier2d::Vector rapier_axis = { axis.x, axis.y };
	rapier2d::Vector rapier_limits = { 0.0, length };

	ERR_FAIL_COND(!p_body_a->get_space());
	ERR_FAIL_COND(p_body_a->get_space() != p_body_b->get_space());
	space_handle = p_body_a->get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));

	handle = rapier2d::joint_create_prismatic(space_handle, p_body_a->get_body_handle(), p_body_b->get_body_handle(), &rapier_axis, &rapier_anchor_A, &rapier_anchor_B, &rapier_limits);
	ERR_FAIL_COND(!rapier2d::is_handle_valid(handle));

	//A->add_constraint(this, 0);
	//B->add_constraint(this, 1);
}
