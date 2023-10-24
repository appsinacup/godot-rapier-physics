#ifndef RAPIER_PIN_JOINT_2D_H
#define RAPIER_PIN_JOINT_2D_H

#include "rapier_joint_2d.h"

using namespace godot;

class RapierPinJoint2D : public RapierJoint2D {
	real_t angular_limit_lower = 0.0;
	real_t angular_limit_upper = 0.0;
	real_t motor_target_velocity = 0.0;
	bool motor_enabled = false;
	bool angular_limit_enabled = false;

public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_PIN; }

	void set_param(PhysicsServer2D::PinJointParam p_param, real_t p_value);
	real_t get_param(PhysicsServer2D::PinJointParam p_param) const;

	void set_flag(PhysicsServer2D::PinJointFlag p_flag, bool p_enabled);
	bool get_flag(PhysicsServer2D::PinJointFlag p_flag) const;

	RapierPinJoint2D(const Vector2 &p_pos, RapierBody2D *p_body_a, RapierBody2D *p_body_b = nullptr);
};

#endif // RAPIER_PIN_JOINT_2D_H
