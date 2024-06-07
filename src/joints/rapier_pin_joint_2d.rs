use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::joints::rapier_joint_2d::RapierJointBase2D;
use crate::rapier2d::handle::invalid_handle;
use crate::rapier2d::joint::joint_change_revolute_params;
use crate::rapier2d::joint::joint_create_revolute;
use crate::servers::rapier_physics_singleton_2d::bodies_singleton;
use godot::builtin::Vector2;
use godot::{builtin::Rid, engine::physics_server_2d};

use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
pub struct RapierPinJoint2D {
    angular_limit_lower: f32,
    angular_limit_upper: f32,
    motor_target_velocity: f32,
    motor_enabled: bool,
    angular_limit_enabled: bool,
    base: RapierJointBase2D,
}

impl RapierPinJoint2D {
    pub fn new(rid: Rid, pos: Vector2, body_a: Rid, body_b: Rid) -> Self {
        let invalid_joint = Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
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
                let anchor_a = body_a.get_base().get_inv_transform().basis_xform(pos);
                let anchor_b = body_b.get_base().get_inv_transform().basis_xform(pos);
                let rapier_anchor_a = rapier2d::na::Vector2::new(anchor_a.x, anchor_a.y);
                let rapier_anchor_b = rapier2d::na::Vector2::new(anchor_b.x, anchor_b.y);
                let space_handle = body_a.get_base().get_space_handle();

                let handle = joint_create_revolute(
                    space_handle,
                    body_a.get_base().get_body_handle(),
                    body_b.get_base().get_body_handle(),
                    rapier_anchor_a,
                    rapier_anchor_b,
                    0.0,
                    0.0,
                    false,
                    0.0,
                    false,
                    true,
                );
                return Self {
                    angular_limit_lower: 0.0,
                    angular_limit_upper: 0.0,
                    motor_target_velocity: 0.0,
                    motor_enabled: false,
                    angular_limit_enabled: false,
                    base: RapierJointBase2D::new(space_handle, handle, rid),
                };
            }
        }
        invalid_joint
    }

    pub fn set_param(&mut self, p_param: physics_server_2d::PinJointParam, p_value: f32) {
        match p_param {
            physics_server_2d::PinJointParam::LIMIT_UPPER => {
                self.angular_limit_upper = p_value;
            }
            physics_server_2d::PinJointParam::LIMIT_LOWER => {
                self.angular_limit_lower = p_value;
            }
            physics_server_2d::PinJointParam::MOTOR_TARGET_VELOCITY => {
                self.motor_target_velocity = p_value;
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        joint_change_revolute_params(
            self.base.get_space_handle(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
        );
    }

    pub fn get_param(&self, p_param: physics_server_2d::PinJointParam) -> f32 {
        match p_param {
            physics_server_2d::PinJointParam::LIMIT_UPPER => self.angular_limit_upper,
            physics_server_2d::PinJointParam::LIMIT_LOWER => self.angular_limit_lower,
            physics_server_2d::PinJointParam::MOTOR_TARGET_VELOCITY => self.motor_target_velocity,
            _ => 0.0,
        }
    }

    pub fn set_flag(&mut self, p_flag: physics_server_2d::PinJointFlag, p_enabled: bool) {
        match p_flag {
            physics_server_2d::PinJointFlag::ANGULAR_LIMIT_ENABLED => {
                self.angular_limit_enabled = p_enabled;
            }
            physics_server_2d::PinJointFlag::MOTOR_ENABLED => {
                self.motor_enabled = p_enabled;
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        joint_change_revolute_params(
            self.base.get_space_handle(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
        );
    }

    pub fn get_flag(&self, p_flag: physics_server_2d::PinJointFlag) -> bool {
        match p_flag {
            physics_server_2d::PinJointFlag::ANGULAR_LIMIT_ENABLED => self.angular_limit_enabled,
            physics_server_2d::PinJointFlag::MOTOR_ENABLED => self.motor_enabled,
            _ => false,
        }
    }
}

impl IRapierJoint2D for RapierPinJoint2D {
    fn get_base(&self) -> &RapierJointBase2D {
        &self.base
    }
    fn get_mut_base(&mut self) -> &mut RapierJointBase2D {
        &mut self.base
    }
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::PIN
    }

    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D> {
        None
    }
    fn get_pin(&self) -> Option<&RapierPinJoint2D> {
        Some(self)
    }

    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }
    fn get_mut_pin(&mut self) -> Option<&mut RapierPinJoint2D> {
        Some(self)
    }
}
