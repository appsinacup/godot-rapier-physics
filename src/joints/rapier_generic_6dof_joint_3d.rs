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
pub struct RapierGeneric6DOFJoint3D {
    anchor_a: Vector3,
    anchor_b: Vector3,
    base: RapierJointBase,
}
impl RapierGeneric6DOFJoint3D {
    pub fn new(
        anchor_a: Vector3,
        anchor_b: Vector3,
        body_a: &Box<dyn IRapierCollisionObjectBase>,
        body_b: &Box<dyn IRapierCollisionObjectBase>,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            anchor_a,
            anchor_b,
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
        let rapier_anchor_a = vector_to_rapier(anchor_a);
        let rapier_anchor_b = vector_to_rapier(anchor_b);
        let space_handle = body_a.get_base().get_space_handle();
        let space_rid = body_a.get_base().get_space();
        let handle = physics_engine.joint_create_spherical(
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
            anchor_a,
            anchor_b,
            base: RapierJointBase::new(space_handle, space_rid, handle),
        }
    }

    pub fn set_anchor_a(&mut self, anchor_a: Vector3, physics_engine: &mut PhysicsEngine) {
        self.anchor_a = anchor_a;
        if !self.base.is_valid() {
            return;
        }
        let anchor_a = vector_to_rapier(self.anchor_a);
        let anchor_b = vector_to_rapier(self.anchor_a);
        physics_engine.join_change_sperical_anchors(
            self.base.get_space_handle(),
            self.base.get_handle(),
            anchor_a,
            anchor_b,
        );
    }

    pub fn set_anchor_b(&mut self, anchor_b: Vector3, physics_engine: &mut PhysicsEngine) {
        self.anchor_b = anchor_b;
        if !self.base.is_valid() {
            return;
        }
        let anchor_a = vector_to_rapier(self.anchor_a);
        let anchor_b = vector_to_rapier(self.anchor_a);
        physics_engine.join_change_sperical_anchors(
            self.base.get_space_handle(),
            self.base.get_handle(),
            anchor_a,
            anchor_b,
        );
    }

    pub fn get_anchor_a(&self) -> Vector3 {
        self.anchor_a
    }

    pub fn get_anchor_b(&self) -> Vector3 {
        self.anchor_b
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierJoint for RapierGeneric6DOFJoint3D {
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
