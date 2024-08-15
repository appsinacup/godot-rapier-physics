use godot::classes::*;
use godot::prelude::*;

use super::rapier_cone_twist_joint_3d::RapierConeTwistJoint3D;
use super::rapier_generic_6dof_joint_3d::RapierGeneric6DOFJoint3D;
use super::rapier_revolute_joint::RapierRevoluteJoint;
use super::rapier_spherical_joint_3d::RapierSphericalJoint3D;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
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
        body_a: &Box<dyn IRapierCollisionObject>,
        body_b: &Box<dyn IRapierCollisionObject>,
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

    fn get_spherical(&self) -> Option<&RapierSphericalJoint3D> {
        None
    }

    fn get_cone_twist(&self) -> Option<&RapierConeTwistJoint3D> {
        None
    }

    fn get_generic_6dof(&self) -> Option<&RapierGeneric6DOFJoint3D> {
        None
    }

    fn get_slider(&self) -> Option<&RapierSliderJoint3D> {
        Some(self)
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
        None
    }

    fn get_mut_slider(&mut self) -> Option<&mut RapierSliderJoint3D> {
        Some(self)
    }
}
