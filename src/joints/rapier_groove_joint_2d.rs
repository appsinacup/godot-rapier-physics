use godot::classes::*;
use godot::prelude::*;

use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
use super::rapier_joint::RapierJointBase;
use super::rapier_revolute_joint::RapierRevoluteJoint;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::types::*;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierGrooveJoint2D {
    base: RapierJointBase,
}
impl RapierGrooveJoint2D {
    pub fn new(
        p_a_groove1: Vector,
        p_a_groove2: Vector,
        p_b_anchor: Vector,
        body_a: &dyn IRapierCollisionObject,
        body_b: &dyn IRapierCollisionObject,
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
            || body_a.get_base().get_space_handle() != body_b.get_base().get_space_handle()
        {
            return invalid_joint;
        }
        let base_a = body_a.get_base();
        let point_a_1 = base_a.get_inv_transform() * p_a_groove1;
        let point_a_2 = base_a.get_inv_transform() * p_a_groove2;
        let axis = vector_normalized(point_a_2 - point_a_1);
        let length = (point_a_2 - point_a_1).length();
        let rapier_axis = vector_to_rapier(axis);
        let rapier_limits = vector_to_rapier(Vector2::new(0.0, length));
        let rapier_anchor_a = vector_to_rapier(point_a_1);
        let base_b = body_b.get_base();
        let anchor_b = base_b.get_inv_transform() * p_b_anchor;
        let rapier_anchor_b = vector_to_rapier(anchor_b);
        let space_handle = body_a.get_base().get_space_handle();
        let space_rid = body_a.get_base().get_space();
        let handle = physics_engine.joint_create_prismatic(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            rapier_axis,
            rapier_anchor_a,
            rapier_anchor_b,
            rapier_limits,
            false,
            false,
            true,
        );
        Self {
            base: RapierJointBase::new(space_handle, space_rid, handle),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierJoint for RapierGrooveJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::GROOVE
    }

    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D> {
        None
    }

    fn get_revolute(&self) -> Option<&RapierRevoluteJoint> {
        None
    }

    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }

    fn get_mut_revolute(&mut self) -> Option<&mut RapierRevoluteJoint> {
        None
    }
}
