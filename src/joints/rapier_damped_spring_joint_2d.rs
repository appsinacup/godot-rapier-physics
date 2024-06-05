use crate::{
    joints::rapier_joint_2d::IRapierJoint2D,
    rapier2d::{
        handle::invalid_handle,
        joint::{joint_change_spring_params, joint_create_spring},
    },
    servers::rapier_physics_singleton_2d::bodies_singleton,
};
use godot::{
    builtin::{real, Rid, Vector2},
    engine::physics_server_2d,
};

use super::{rapier_joint_2d::RapierJointBase2D, rapier_pin_joint_2d::RapierPinJoint2D};
pub struct RapierDampedSpringJoint2D {
    rest_length: real,
    stiffness: real,
    damping: real,
    base: RapierJointBase2D,
}

impl RapierDampedSpringJoint2D {
    pub fn new(
        rid: Rid,
        p_anchor_a: Vector2,
        p_anchor_b: Vector2,
        body_a: Rid,
        body_b: Rid,
    ) -> Self {
        let lock = bodies_singleton();
        if let Some(body_a) = lock.collision_objects.get(&body_a) {
            let base_a = body_a.get_base();
            let body_a_handle = base_a.get_body_handle();
            if let Some(body_b) = lock.collision_objects.get(&body_b) {
                let base_b = body_b.get_base();
                let body_b_handle = base_b.get_body_handle();

                let anchor_a = base_a.get_inv_transform().basis_xform(p_anchor_a);
                let anchor_b = base_a.get_inv_transform().basis_xform(p_anchor_b);

                let rest_length = p_anchor_a.distance_to(p_anchor_b);
                let rapier_anchor_a = rapier2d::na::Vector2::new(anchor_a.x, anchor_a.y);
                let rapier_anchor_b = rapier2d::na::Vector2::new(anchor_b.x, anchor_b.y);
                let space_handle = body_a.get_base().get_space_handle();

                let handle = joint_create_spring(
                    space_handle,
                    body_a_handle,
                    body_b_handle,
                    rapier_anchor_a,
                    rapier_anchor_b,
                    20.0,
                    1.5,
                    rest_length,
                    true,
                );
                return Self {
                    rest_length,
                    stiffness: 20.0,
                    damping: 1.5,
                    base: RapierJointBase2D::new(space_handle, handle, rid),
                };
            }
        }
        Self {
            rest_length: 0.0,
            stiffness: 20.0,
            damping: 1.5,
            base: RapierJointBase2D::new(invalid_handle(), invalid_handle(), rid),
        }
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
        if !handle.is_valid() || !space_handle.is_valid() {
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

impl IRapierJoint2D for RapierDampedSpringJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::DAMPED_SPRING
    }
    fn get_mut_base(&mut self) -> &mut RapierJointBase2D {
        &mut self.base
    }
    fn get_base(&self) -> &RapierJointBase2D {
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
