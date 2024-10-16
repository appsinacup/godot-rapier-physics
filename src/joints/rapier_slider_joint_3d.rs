use godot::classes::*;
use godot::prelude::*;

use super::rapier_joint_base::RapierJointBase;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::RapierId;
pub struct RapierSliderJoint3D {
    base: RapierJointBase,
}
impl RapierSliderJoint3D {
    pub fn new(
        id: RapierId,
        rid: Rid,
        anchor_a: Transform3D,
        anchor_b: Transform3D,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            base: RapierJointBase::default(),
        };
        let body_a_rid = body_a.get_base().get_rid();
        let body_b_rid = body_b.get_base().get_rid();
        if body_a_rid == body_b_rid {
            return invalid_joint;
        }
        if !body_a.get_base().is_valid()
            || !body_b.get_base().is_valid()
            || body_a.get_base().get_space_id() != body_b.get_base().get_space_id()
        {
            return invalid_joint;
        }
        let rapier_anchor_a = vector_to_rapier(anchor_a.origin);
        let rapier_anchor_b = vector_to_rapier(anchor_b.origin);
        let space_handle = body_a.get_base().get_space_id();
        let space_id = body_a.get_base().get_space_id();
        let handle = physics_engine.joint_create_slider(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            rapier_anchor_a,
            rapier_anchor_b,
            false,
            false,
            true,
        );
        Self {
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle),
        }
    }
}
impl IRapierJoint for RapierSliderJoint3D {
    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    fn get_type(&self) -> physics_server_3d::JointType {
        physics_server_3d::JointType::TYPE_6DOF
    }
}
