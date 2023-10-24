#ifndef RAPIER_GROOVE_JOINT_2D_H
#define RAPIER_GROOVE_JOINT_2D_H

#include "rapier_joint_2d.h"

class RapierGrooveJoint2D : public RapierJoint2D {
public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_GROOVE; }

	RapierGrooveJoint2D(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, RapierBody2D *p_body_a, RapierBody2D *p_body_b);
};

#endif // RAPIER_GROOVE_JOINT_2D_H
