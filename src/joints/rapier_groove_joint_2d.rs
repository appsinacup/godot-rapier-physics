use crate::{
    joints::rapier_joint_2d::IRapierJoint2D,
    rapier_wrapper::{handle::invalid_handle, joint::joint_create_prismatic},
    servers2d::rapier_physics_singleton_2d::bodies_singleton,
};
use godot::{
    builtin::{Rid, Vector2},
    engine::physics_server_2d,
};

use super::{
    rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D, rapier_joint_2d::RapierJointBase2D,
    rapier_pin_joint_2d::RapierPinJoint2D,
};
pub struct RapierGrooveJoint2D {
    base: RapierJointBase2D,
}

impl RapierGrooveJoint2D {
    pub fn new(
        rid: Rid,
        p_a_groove1: Vector2,
        p_a_groove2: Vector2,
        p_b_anchor: Vector2,
        body_a: Rid,
        body_b: Rid,
    ) -> Self {
        let invalid_joint = Self {
            base: RapierJointBase2D::new(invalid_handle(), invalid_handle(), rid),
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
                let rapier_axis = rapier2d::na::Vector2::new(axis.x, axis.y);
                let rapier_limits = rapier2d::na::Vector2::new(0.0, length);
                let rapier_anchor_a = rapier2d::na::Vector2::new(point_a_1.x, point_a_1.y);

                let base_b = body_b.get_base();
                let anchor_b = base_b.get_inv_transform() * p_b_anchor;
                let rapier_anchor_b = rapier2d::na::Vector2::new(anchor_b.x, anchor_b.y);
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
                    base: RapierJointBase2D::new(space_handle, handle, rid),
                };
            }
        }
        invalid_joint
    }
}

impl IRapierJoint2D for RapierGrooveJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::GROOVE
    }

    fn get_base(&self) -> &RapierJointBase2D {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase2D {
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
