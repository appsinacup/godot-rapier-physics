use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::joints::rapier_joint_2d::RapierJointBase2D;
use crate::rapier2d::joint::joint_change_revolute_params;
use crate::rapier2d::joint::joint_create_revolute;
use crate::rapier2d::vector::Vector;
use crate::servers::rapier_physics_singleton_2d::bodies_singleton;
use crate::servers::rapier_physics_singleton_2d::spaces_singleton;
use godot::builtin::Vector2;
use godot::{builtin::Rid, engine::physics_server_2d};

use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
use super::rapier_groove_joint_2d::RapierGrooveJoint2D;
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
        let body_a_handle;
        let body_b_handle;
        let space_rid;
        let space_handle;
        let rapier_anchor_a;
        let rapier_anchor_b;
        {
            let lock = bodies_singleton().lock().unwrap();
            let body_a = lock.collision_objects.get(&body_a);
            assert!(body_a.is_some());
            let body_a = body_a.unwrap();
            body_a_handle = body_a.get_base().get_body_handle();
            let anchor_a = body_a.get_base().get_inv_transform().basis_xform(pos);
            rapier_anchor_a = Vector::new(anchor_a.x, anchor_a.y);
            let mut anchor_b = pos;
            let body_b = lock.collision_objects.get(&body_b);
            assert!(body_b.is_some());
            let body_b = body_b.unwrap();
            body_b_handle = body_b.get_base().get_body_handle();
            anchor_b = body_b.get_base().get_inv_transform().basis_xform(pos);
            rapier_anchor_b = Vector::new(anchor_b.x, anchor_b.y);
            space_rid = body_a.get_base().get_space();
        }
        {
            let lock = spaces_singleton().lock().unwrap();
            let space_a = lock.spaces.get(&space_rid);
            assert!(space_a.is_some());
            space_handle = space_a.unwrap().get_handle();
        }

        let handle = joint_create_revolute(
            space_handle,
            body_a_handle,
            body_b_handle,
            &rapier_anchor_a,
            &rapier_anchor_b,
            0.0,
            0.0,
            false,
            0.0,
            false,
            true,
        );
        Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
            base: RapierJointBase2D::new(space_handle, handle, rid),
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
            _ => {}
        }
        let handle = self.get_base().get_handle();
        if !handle.is_valid() {
            return;
        }
        let space_handle = self.get_base().get_space_handle();
        joint_change_revolute_params(
            space_handle,
            handle,
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
        let handle = self.get_base().get_handle();
        if !handle.is_valid() {
            return;
        }
        let space_handle = self.get_base().get_space_handle();
        joint_change_revolute_params(
            space_handle,
            handle,
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
    fn get_groove(&self) -> Option<&RapierGrooveJoint2D> {
        None
    }

    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }
    fn get_mut_pin(&mut self) -> Option<&mut RapierPinJoint2D> {
        Some(self)
    }
    fn get_mut_groove(&mut self) -> Option<&mut RapierGrooveJoint2D> {
        None
    }
}
