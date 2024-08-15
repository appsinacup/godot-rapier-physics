use godot::classes::*;
use godot::prelude::*;

use super::rapier_cone_twist_joint_3d::RapierConeTwistJoint3D;
use super::rapier_revolute_joint::RapierRevoluteJoint;
use super::rapier_slider_joint_3d::RapierSliderJoint3D;
use super::rapier_spherical_joint_3d::RapierSphericalJoint3D;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
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
        body_a: &Box<dyn IRapierCollisionObject>,
        body_b: &Box<dyn IRapierCollisionObject>,
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

    fn get_spherical(&self) -> Option<&RapierSphericalJoint3D> {
        None
    }

    fn get_cone_twist(&self) -> Option<&RapierConeTwistJoint3D> {
        None
    }

    fn get_generic_6dof(&self) -> Option<&RapierGeneric6DOFJoint3D> {
        Some(self)
    }

    fn get_slider(&self) -> Option<&RapierSliderJoint3D> {
        None
    }

    fn get_revolute(&self) -> Option<&RapierRevoluteJoint> {
        None
    }

    fn get_mut_spherical(&mut self) -> Option<&mut RapierSphericalJoint3D> {
        None
    }

    fn get_mut_revolute(&mut self) -> Option<&mut RapierRevoluteJoint> {
        None
    }

    fn get_mut_cone_twist(&mut self) -> Option<&mut RapierConeTwistJoint3D> {
        None
    }

    fn get_mut_generic_6dof(&mut self) -> Option<&mut RapierGeneric6DOFJoint3D> {
        Some(self)
    }

    fn get_mut_slider(&mut self) -> Option<&mut RapierSliderJoint3D> {
        None
    }
}
