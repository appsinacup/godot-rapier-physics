use godot::log::godot_warn;
use godot::obj::EngineEnum;
use godot::{builtin::Rid, engine::physics_server_2d};
use crate::rapier2d::handle::{self, invalid_handle, Handle};
use crate::joints::rapier_joint_2d::IRapierJoint2D;

use super::rapier_joint_2d::RapierJointBase2D;
pub struct RapierPinJoint2D {
    angular_limit_lower: f32,
    angular_limit_upper: f32,
    motor_target_velocity: f32,
    motor_enabled: bool,
    angular_limit_enabled: bool,

    base: RapierJointBase2D,
}

impl RapierPinJoint2D {
    pub fn new(body_a_rid: Rid, body_b_rid: Option<Rid>) -> Self {
        Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,

            base: RapierJointBase2D::new(body_a_rid, body_b_rid),
        }
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
            _ => {
                godot_warn!("Unsupported pin joint param: {}", p_param.ord());
            }
        }
    }

    pub fn get_param(&self, p_param: physics_server_2d::PinJointParam) -> f32 {
        match p_param {
            physics_server_2d::PinJointParam::LIMIT_UPPER => self.angular_limit_upper,
            physics_server_2d::PinJointParam::LIMIT_LOWER => self.angular_limit_lower,
            physics_server_2d::PinJointParam::MOTOR_TARGET_VELOCITY => self.motor_target_velocity,
            _ => {
                godot_warn!("Unsupported pin joint param: {}", p_param.ord());
                0.0
            }
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
            _ => {
                godot_warn!("Unsupported pin joint flag: {}", p_flag.ord());
            }
        }
    }

    pub fn get_flag(&self, p_flag: physics_server_2d::PinJointFlag) -> bool {
        match p_flag {
            physics_server_2d::PinJointFlag::ANGULAR_LIMIT_ENABLED => self.angular_limit_enabled,
            physics_server_2d::PinJointFlag::MOTOR_ENABLED => self.motor_enabled,
            _ => {
                godot_warn!("Unsupported pin joint flag: {}", p_flag.ord());
                false
            }
        }
    }
}

impl IRapierJoint2D for RapierPinJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::PIN
    }
}
