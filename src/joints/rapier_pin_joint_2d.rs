use godot::classes::*;
use godot::prelude::*;
use rapier::dynamics::ImpulseJointHandle;
use rapier::math::Vector;

use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
use crate::joints::rapier_joint::IRapierJoint;
use crate::joints::rapier_joint::RapierJointBase;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_server_extra::PhysicsData;
pub struct RapierPinJoint2D {
    angular_limit_lower: f32,
    angular_limit_upper: f32,
    motor_target_velocity: f32,
    motor_enabled: bool,
    angular_limit_enabled: bool,
    base: RapierJointBase,
}
impl RapierPinJoint2D {
    pub fn new(pos: Vector2, body_a: Rid, body_b: Rid, physics_data: &mut PhysicsData) -> Self {
        let invalid_joint = Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
            base: RapierJointBase::new(invalid_handle(), ImpulseJointHandle::invalid()),
        };
        if body_a == body_b {
            return invalid_joint;
        }
        if let Some(body_a) = physics_data.collision_objects.get(&body_a) {
            if let Some(body_b) = physics_data.collision_objects.get(&body_b) {
                if !body_a.get_base().is_valid()
                    || !body_b.get_base().is_valid()
                    || body_a.get_base().get_space_handle() != body_b.get_base().get_space_handle()
                {
                    return invalid_joint;
                }
                let anchor_a = body_a.get_base().get_inv_transform() * pos;
                let anchor_b = body_b.get_base().get_inv_transform() * pos;
                let rapier_anchor_a = Vector::new(anchor_a.x, anchor_a.y);
                let rapier_anchor_b = Vector::new(anchor_b.x, anchor_b.y);
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
                    &mut physics_data.physics_engine,
                );
                return Self {
                    angular_limit_lower: 0.0,
                    angular_limit_upper: 0.0,
                    motor_target_velocity: 0.0,
                    motor_enabled: false,
                    angular_limit_enabled: false,
                    base: RapierJointBase::new(space_handle, handle),
                };
            }
        }
        invalid_joint
    }

    pub fn set_param(
        &mut self,
        p_param: physics_server_2d::PinJointParam,
        p_value: f32,
        physics_engine: &mut PhysicsEngine,
    ) {
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
            physics_engine,
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

    pub fn set_flag(
        &mut self,
        p_flag: physics_server_2d::PinJointFlag,
        p_enabled: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
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
            physics_engine,
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
impl IRapierJoint for RapierPinJoint2D {
    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
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
