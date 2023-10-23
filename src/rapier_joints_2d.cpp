#include "rapier_joints_2d.h"

#include "rapier_space_2d.h"

void RapierJoint2D::copy_settings_from(RapierJoint2D *p_joint) {
	set_rid(p_joint->get_rid());
	set_max_force(p_joint->get_max_force());
	set_bias(p_joint->get_bias());
	set_max_bias(p_joint->get_max_bias());
	disable_collisions_between_bodies(p_joint->is_disabled_collisions_between_bodies());
}

RapierJoint2D::~RapierJoint2D() {
	if (rapier2d::is_handle_valid(handle)) {
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		rapier2d::joint_destroy(space_handle, handle);
	}

	/*for (int i = 0; i < get_body_count(); i++) {
		RapierBody2D *body = get_body_ptr()[i];
		if (body) {
			body->remove_constraint(this, i);
		}
	}*/
};

//////////////////////////////////////////////

void RapierPinJoint2D::set_param(PhysicsServer2D::PinJointParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer2D::PIN_JOINT_LIMIT_UPPER: {
			angular_limit_upper = p_value;
		} break;
		case PhysicsServer2D::PIN_JOINT_LIMIT_LOWER: {
			angular_limit_lower = p_value;
		} break;
		case PhysicsServer2D::PIN_JOINT_MOTOR_TARGET_VELOCITY: {
			motor_target_velocity = p_value;
		} break;
		case PhysicsServer2D::PIN_JOINT_SOFTNESS: {
			WARN_PRINT_ONCE("PIN_JOINT_SOFTNESS is unused");
			return;
		}
	}
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
	ERR_FAIL_COND(!rapier2d::is_handle_valid(handle));
	rapier2d::joint_change_revolute_params(space_handle, handle, angular_limit_lower, angular_limit_upper, angular_limit_enabled, motor_target_velocity, motor_enabled);
}

real_t RapierPinJoint2D::get_param(PhysicsServer2D::PinJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer2D::PIN_JOINT_LIMIT_UPPER: {
			return angular_limit_upper;
		}
		case PhysicsServer2D::PIN_JOINT_LIMIT_LOWER: {
			return angular_limit_lower;
		}
		case PhysicsServer2D::PIN_JOINT_MOTOR_TARGET_VELOCITY: {
			return motor_target_velocity;
		}
		case PhysicsServer2D::PIN_JOINT_SOFTNESS: {
			WARN_PRINT_ONCE("PIN_JOINT_SOFTNESS is unused");
			return 0.0;
		}
	}
	ERR_FAIL_V(0);
}

void RapierPinJoint2D::set_flag(PhysicsServer2D::PinJointFlag p_flag, bool p_enabled) {
	switch (p_flag) {
		case PhysicsServer2D::PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED: {
			angular_limit_enabled = p_enabled;
		} break;
		case PhysicsServer2D::PIN_JOINT_FLAG_MOTOR_ENABLED: {
			motor_enabled = p_enabled;
		} break;
	}
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
	ERR_FAIL_COND(!rapier2d::is_handle_valid(handle));
	rapier2d::joint_change_revolute_params(space_handle, handle, angular_limit_lower, angular_limit_upper, angular_limit_enabled, motor_target_velocity, motor_enabled);
}

bool RapierPinJoint2D::get_flag(PhysicsServer2D::PinJointFlag p_flag) const {
	switch (p_flag) {
		case PhysicsServer2D::PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED: {
			return angular_limit_enabled;
		}
		case PhysicsServer2D::PIN_JOINT_FLAG_MOTOR_ENABLED: {
			return motor_enabled;
		}
	}
	ERR_FAIL_V(0);
}

RapierPinJoint2D::RapierPinJoint2D(const Vector2 &p_pos, RapierBody2D *p_body_a, RapierBody2D *p_body_b) :
		RapierJoint2D(_arr, p_body_b ? 2 : 1) {
	A = p_body_a;
	B = p_body_b;

	Vector2 anchor_A = p_body_a->get_inv_transform().xform(p_pos);
	Vector2 anchor_B = p_body_b ? p_body_b->get_inv_transform().xform(p_pos) : p_pos;

	rapier2d::Vector rapier_anchor_A = { anchor_A.x, anchor_A.y };
	rapier2d::Vector rapier_anchor_B = { anchor_B.x, anchor_B.y };

	ERR_FAIL_COND(!p_body_a->get_space());
	ERR_FAIL_COND(p_body_a->get_space() != p_body_b->get_space());
	space_handle = p_body_a->get_space()->get_handle();
	ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
	handle = rapier2d::joint_create_revolute(space_handle, p_body_a->get_body_handle(), p_body_b->get_body_handle(), &rapier_anchor_A, &rapier_anchor_B, angular_limit_lower, angular_limit_upper, angular_limit_enabled, motor_target_velocity, motor_enabled);
	ERR_FAIL_COND(!rapier2d::is_handle_valid(handle));

	/*p_body_a->add_constraint(this, 0);
	if (p_body_b) {
		p_body_b->add_constraint(this, 1);
	}*/
}

//////////////////////////////////////////////

RapierGrooveJoint2D::RapierGrooveJoint2D(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, RapierBody2D *p_body_a, RapierBody2D *p_body_b) :
		RapierJoint2D(_arr, 2) {
	A = p_body_a;
	B = p_body_b;

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

//////////////////////////////////////////////

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
		RapierJoint2D(_arr, 2) {
	A = p_body_a;
	B = p_body_b;

	Vector2 anchor_A = A->get_inv_transform().xform(p_anchor_a);
	Vector2 anchor_B = B->get_inv_transform().xform(p_anchor_b);

	rest_length = p_anchor_a.distance_to(p_anchor_b);

	// TODO: create rapier joint when available
	// See https://github.com/dimforge/rapier/issues/241
	ERR_FAIL_MSG("Spring joints not supported for now");

	//A->add_constraint(this, 0);
	//B->add_constraint(this, 1);
}
