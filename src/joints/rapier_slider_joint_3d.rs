use godot::classes::*;
use godot::prelude::*;

use crate::bodies::rapier_collision_object::IRapierCollisionObjectBase;
use crate::joints::rapier_joint::IRapierJoint;
use crate::joints::rapier_joint::RapierJointBase;
use crate::rapier_wrapper::prelude::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierSliderJoint3D {
    anchor_a: Vector3,
    anchor_b: Vector3,
    base: RapierJointBase,
}
impl RapierSliderJoint3D {
    pub fn new(
        anchor_a: Transform3D,
        anchor_b: Transform3D,
        body_a: &Box<dyn IRapierCollisionObjectBase>,
        body_b: &Box<dyn IRapierCollisionObjectBase>,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            anchor_a: anchor_a.origin,
            anchor_b: anchor_b.origin,
            base: RapierJointBase::default(),
        };
        let body_a_rid = body_a.get_base().get_rid();
        let body_b_rid = body_b.get_base().get_rid();
        if body_a_rid == body_b_rid {
            return invalid_joint;
        }
        if !body_a.get_base().is_valid()
            || !body_b.get_base().is_valid()
            || body_a.get_base().get_space_handle() != body_b.get_base().get_space_handle()
        {
            return invalid_joint;
        }
        let rapier_anchor_a = vector_to_rapier(anchor_a.origin);
        let rapier_anchor_b = vector_to_rapier(anchor_b.origin);
        let space_handle = body_a.get_base().get_space_handle();
        let space_rid = body_a.get_base().get_space();
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
            anchor_a: anchor_a.origin,
            anchor_b: anchor_b.origin,
            base: RapierJointBase::new(space_handle, space_rid, handle),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
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
