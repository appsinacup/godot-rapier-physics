use godot::classes::*;
use godot::prelude::*;
use rapier::dynamics::ImpulseJointHandle;
use rapier::math::Real;
use rapier::math::Vector;

use super::rapier_joint::RapierJointBase;
use super::rapier_pin_joint_2d::RapierPinJoint2D;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::bodies_singleton;
pub struct RapierDampedSpringJoint2D {
    rest_length: real,
    stiffness: real,
    damping: real,
    base: RapierJointBase,
}
impl RapierDampedSpringJoint2D {
    pub fn new(p_anchor_a: Vector<Real>, p_anchor_b: Vector<Real>, body_a: Rid, body_b: Rid) -> Self {
        let invalid_joint = Self {
            rest_length: 0.0,
            stiffness: 20.0,
            damping: 1.5,
            base: RapierJointBase::new(invalid_handle(), ImpulseJointHandle::invalid()),
        };
        if body_a == body_b {
            return invalid_joint;
        }
        let bodies_singleton = bodies_singleton();
        if let Some(body_a) = bodies_singleton.collision_objects.get(&body_a) {
            if let Some(body_b) = bodies_singleton.collision_objects.get(&body_b) {
                if !body_a.get_base().is_valid()
                    || !body_b.get_base().is_valid()
                    || body_a.get_base().get_space_handle() != body_b.get_base().get_space_handle()
                {
                    return invalid_joint;
                }
                let rapier_anchor_a = body_a.get_base().get_inv_transform().isometry * p_anchor_a;
                let rapier_anchor_b = body_b.get_base().get_inv_transform().isometry * p_anchor_b;
                let rest_length = (p_anchor_a - p_anchor_b).norm();
                let space_handle = body_a.get_base().get_space_handle();
                let handle = joint_create_spring(
                    space_handle,
                    body_a.get_base().get_body_handle(),
                    body_b.get_base().get_body_handle(),
                    rapier_anchor_a,
                    rapier_anchor_b,
                    20.0,
                    1.5,
                    rest_length,
                    true,
                );
                if handle == ImpulseJointHandle::invalid() {
                    return invalid_joint;
                }
                return Self {
                    rest_length,
                    stiffness: 20.0,
                    damping: 1.5,
                    base: RapierJointBase::new(space_handle, handle),
                };
            }
        }
        invalid_joint
    }

    pub fn set_param(&mut self, p_param: physics_server_2d::DampedSpringParam, p_value: f32) {
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
        let handle = self.get_base().get_handle();
        let space_handle = self.get_base().get_space_handle();
        if handle == ImpulseJointHandle::invalid() || !space_handle.is_valid() {
            return;
        }
        joint_change_spring_params(
            space_handle,
            handle,
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

    fn get_pin(&self) -> Option<&RapierPinJoint2D> {
        None
    }

    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        Some(self)
    }

    fn get_mut_pin(&mut self) -> Option<&mut RapierPinJoint2D> {
        None
    }
}
