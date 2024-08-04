use godot::classes::*;
use godot::prelude::*;

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
pub struct RapierDampedSpringJoint2D {
    rest_length: real,
    stiffness: real,
    damping: real,
    base: RapierJointBase,
}
impl RapierDampedSpringJoint2D {
    pub fn new(
        p_anchor_a: Vector,
        p_anchor_b: Vector,
        body_a: &dyn IRapierCollisionObject,
        body_b: &dyn IRapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            rest_length: 0.0,
            stiffness: 20.0,
            damping: 1.0,
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
        let rapier_anchor_a = body_a.get_base().get_inv_transform() * p_anchor_a;
        let rapier_anchor_b = body_b.get_base().get_inv_transform() * p_anchor_b;
        let rest_length = (p_anchor_a - p_anchor_b).length();
        let space_handle = body_a.get_base().get_space_handle();
        let space_rid = body_a.get_base().get_space();
        let handle = physics_engine.joint_create_spring(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            vector_to_rapier(rapier_anchor_a),
            vector_to_rapier(rapier_anchor_b),
            20.0,
            1.0,
            rest_length,
            false,
            false,
            true,
        );
        Self {
            rest_length,
            stiffness: 20.0,
            damping: 1.0,
            base: RapierJointBase::new(space_handle, space_rid, handle),
        }
    }

    pub fn set_param(
        &mut self,
        p_param: physics_server_2d::DampedSpringParam,
        p_value: f32,
        physics_engine: &mut PhysicsEngine,
    ) {
        match p_param {
            physics_server_2d::DampedSpringParam::DAMPING => {
                self.damping = p_value;
            }
            physics_server_2d::DampedSpringParam::STIFFNESS => {
                self.stiffness = p_value;
            }
            physics_server_2d::DampedSpringParam::REST_LENGTH => {
                self.rest_length = p_value;
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.joint_change_spring_params(
            self.base.get_space_handle(),
            self.base.get_handle(),
            self.stiffness,
            self.damping,
            self.rest_length,
        );
    }

    pub fn get_param(&self, p_param: physics_server_2d::DampedSpringParam) -> f32 {
        match p_param {
            physics_server_2d::DampedSpringParam::DAMPING => self.damping,
            physics_server_2d::DampedSpringParam::STIFFNESS => self.stiffness,
            physics_server_2d::DampedSpringParam::REST_LENGTH => self.rest_length,
            _ => 0.0,
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierJoint for RapierDampedSpringJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::DAMPED_SPRING
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D> {
        Some(self)
    }

    fn get_revolute(&self) -> Option<&RapierRevoluteJoint> {
        None
    }

    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        Some(self)
    }

    fn get_mut_revolute(&mut self) -> Option<&mut RapierRevoluteJoint> {
        None
    }
}
