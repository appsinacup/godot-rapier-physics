#include "rapier_joint_2d.h"
#include "../spaces/rapier_space_2d.h"

RapierJoint2D::RapierJoint2D(RapierBody2D *p_body_a, RapierBody2D *p_body_b) {
	A = p_body_a;
	B = p_body_b;
}

void RapierJoint2D::copy_settings_from(RapierJoint2D *p_joint) {
	set_rid(p_joint->get_rid());
	set_max_force(p_joint->get_max_force());
	set_bias(p_joint->get_bias());
	set_max_bias(p_joint->get_max_bias());
	disable_collisions_between_bodies(p_joint->is_disabled_collisions_between_bodies());
}

void RapierJoint2D::disable_collisions_between_bodies(const bool p_disabled) {
	disabled_collisions_between_bodies = p_disabled;
	if (rapier2d::is_handle_valid(handle)) {
		// Joint not yet created, when it will be created it will have disable collision flag set
		rapier2d::joint_change_disable_collision(space_handle, handle, is_disabled_collisions_between_bodies());
	}
}

RapierJoint2D::~RapierJoint2D() {
	if (rapier2d::is_handle_valid(handle)) {
		ERR_FAIL_COND(!rapier2d::is_handle_valid(space_handle));
		rapier2d::joint_destroy(space_handle, handle);
	}
};
