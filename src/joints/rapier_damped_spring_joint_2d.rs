use crate::joints::rapier_joint_2d::IRapierJoint2D;
use godot::log::godot_warn;
use godot::obj::EngineEnum;
use godot::{builtin::Rid, engine::physics_server_2d};

use super::rapier_joint_2d::RapierJointBase2D;
pub struct RapierDampedSpringJoint2D {
    rest_length: f32,
    damping: f32,
    stiffness: f32,
    base: RapierJointBase2D,
}

impl RapierDampedSpringJoint2D {
    pub fn new(rid: Rid, body_a: Rid, body_b: Rid) -> Self {
        Self {
            rest_length: 0.0,
            damping: 0.0,
            stiffness: 0.0,
            base: RapierJointBase2D::new(rid, body_a, body_b),
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
            _ => {
                godot_warn!("Unsupported damped spring joint param: {}", p_param.ord());
            }
        }
    }

    pub fn get_param(&self, p_param: physics_server_2d::DampedSpringParam) -> f32 {
        match p_param {
            physics_server_2d::DampedSpringParam::DAMPING => self.damping,
            physics_server_2d::DampedSpringParam::STIFFNESS => self.stiffness,
            physics_server_2d::DampedSpringParam::REST_LENGTH => self.rest_length,
            _ => {
                godot_warn!("Unsupported damped spring joint param: {}", p_param.ord());
                0.0
            }
        }
    }
}

impl IRapierJoint2D for RapierDampedSpringJoint2D {
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::DAMPED_SPRING
    }
    fn get_base(&self) -> &RapierJointBase2D {
        &self.base
    }
}
