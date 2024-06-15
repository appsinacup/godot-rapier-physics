use godot::classes::*;
use godot::prelude::*;
use rapier::dynamics::ImpulseJointHandle;

use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
use super::rapier_joint::RapierJointBase;
use super::rapier_pin_joint_2d::RapierPinJoint2D;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::bodies_singleton;
use crate::Vector;
pub struct RapierGrooveJoint2D {
    base: RapierJointBase,
}
impl RapierGrooveJoint2D {
    pub fn new(
        p_a_groove1: Vector,
        p_a_groove2: Vector,
        p_b_anchor: Vector,
        body_a: Rid,
        body_b: Rid,
    ) -> Self {
        let invalid_joint = Self {
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
                let base_a = body_a.get_base();
                let point_a_1 = base_a.get_inv_transform() * p_a_groove1;
                let point_a_2 = base_a.get_inv_transform() * p_a_groove2;
                let axis = (point_a_2 - point_a_1).normalized();
                let length = (point_a_2 - point_a_1).length();
                let rapier_axis = vector_to_rapier(axis);
                let rapier_limits = vector_to_rapier(Vector2::new(0.0, length));
                let rapier_anchor_a = vector_to_rapier(point_a_1);
                let base_b = body_b.get_base();
                let anchor_b = base_b.get_inv_transform() * p_b_anchor;
                let rapier_anchor_b = vector_to_rapier(anchor_b);
                let space_handle = body_a.get_base().get_space_handle();
                let handle = joint_create_prismatic(
                    space_handle,
                    body_a.get_base().get_body_handle(),
                    body_b.get_base().get_body_handle(),
                    rapier_axis,
                    rapier_anchor_a,
                    rapier_anchor_b,
                    rapier_limits,
                    true,
                );
                return Self {
                    base: RapierJointBase::new(space_handle, handle),
                };
            }
        }
        invalid_joint
    }
}
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

    fn get_pin(&self) -> Option<&RapierPinJoint2D> {
        None
    }

    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }

    fn get_mut_pin(&mut self) -> Option<&mut RapierPinJoint2D> {
        None
    }
}
