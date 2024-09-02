use godot::classes::*;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;

use super::rapier_joint_base::RapierJointBase;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::types::Vector;
pub struct RapierRevoluteJoint {
    angular_limit_lower: f32,
    angular_limit_upper: f32,
    motor_target_velocity: f32,
    motor_enabled: bool,
    angular_limit_enabled: bool,
    base: RapierJointBase,
}
impl RapierRevoluteJoint {
    pub fn new(
        anchor_a: Vector,
        anchor_b: Vector,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
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
        let anchor_a = body_a.get_base().get_inv_transform() * anchor_a;
        let anchor_b = body_b.get_base().get_inv_transform() * anchor_b;
        let rapier_anchor_a = vector_to_rapier(anchor_a);
        let rapier_anchor_b = vector_to_rapier(anchor_b);
        let space_handle = body_a.get_base().get_space_handle();
        let handle = physics_engine.joint_create_revolute(
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
            false,
            false,
            true,
        );
        Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
            base: RapierJointBase::new(space_handle, handle),
        }
    }

    #[cfg(feature = "dim2")]
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
        physics_engine.joint_change_revolute_params(
            self.base.get_space_handle(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
        );
    }

    #[cfg(feature = "dim3")]
    pub fn set_param(
        &mut self,
        p_param: physics_server_3d::HingeJointParam,
        p_value: f32,
        physics_engine: &mut PhysicsEngine,
    ) {
        match p_param {
            physics_server_3d::HingeJointParam::LIMIT_UPPER => {
                self.angular_limit_upper = p_value;
            }
            physics_server_3d::HingeJointParam::LIMIT_LOWER => {
                self.angular_limit_lower = p_value;
            }
            physics_server_3d::HingeJointParam::MOTOR_TARGET_VELOCITY => {
                self.motor_target_velocity = p_value;
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.joint_change_revolute_params(
            self.base.get_space_handle(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
        );
    }

    #[cfg(feature = "dim2")]
    pub fn get_param(&self, p_param: physics_server_2d::PinJointParam) -> f32 {
        match p_param {
            physics_server_2d::PinJointParam::LIMIT_UPPER => self.angular_limit_upper,
            physics_server_2d::PinJointParam::LIMIT_LOWER => self.angular_limit_lower,
            physics_server_2d::PinJointParam::MOTOR_TARGET_VELOCITY => self.motor_target_velocity,
            _ => 0.0,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn get_param(&self, p_param: physics_server_3d::HingeJointParam) -> f32 {
        match p_param {
            physics_server_3d::HingeJointParam::LIMIT_UPPER => self.angular_limit_upper,
            physics_server_3d::HingeJointParam::LIMIT_LOWER => self.angular_limit_lower,
            physics_server_3d::HingeJointParam::MOTOR_TARGET_VELOCITY => self.motor_target_velocity,
            _ => 0.0,
        }
    }

    #[cfg(feature = "dim2")]
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
        physics_engine.joint_change_revolute_params(
            self.base.get_space_handle(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
        );
    }

    #[cfg(feature = "dim3")]
    pub fn set_flag(
        &mut self,
        p_flag: physics_server_3d::HingeJointFlag,
        p_enabled: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        match p_flag {
            physics_server_3d::HingeJointFlag::USE_LIMIT => {
                self.angular_limit_enabled = p_enabled;
            }
            physics_server_3d::HingeJointFlag::ENABLE_MOTOR => {
                self.motor_enabled = p_enabled;
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.joint_change_revolute_params(
            self.base.get_space_handle(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
        );
    }

    #[cfg(feature = "dim2")]
    pub fn get_flag(&self, p_flag: physics_server_2d::PinJointFlag) -> bool {
        match p_flag {
            physics_server_2d::PinJointFlag::ANGULAR_LIMIT_ENABLED => self.angular_limit_enabled,
            physics_server_2d::PinJointFlag::MOTOR_ENABLED => self.motor_enabled,
            _ => false,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn get_flag(&self, p_flag: physics_server_3d::HingeJointFlag) -> bool {
        match p_flag {
            physics_server_3d::HingeJointFlag::USE_LIMIT => self.angular_limit_enabled,
            physics_server_3d::HingeJointFlag::ENABLE_MOTOR => self.motor_enabled,
            _ => false,
        }
    }
}
impl IRapierJoint for RapierRevoluteJoint {
    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    #[cfg(feature = "dim2")]
    fn get_type(&self) -> JointType {
        JointType::PIN
    }

    #[cfg(feature = "dim3")]
    fn get_type(&self) -> JointType {
        JointType::HINGE
    }
}
