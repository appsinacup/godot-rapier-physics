use crate::joints::rapier_joint_2d::IRapierJoint2D;
use crate::joints::rapier_joint_2d::RapierJointBase2D;
use crate::rapier2d::handle::invalid_handle;
use crate::rapier2d::joint::joint_change_revolute_params;
use crate::rapier2d::joint::joint_create_revolute;
use crate::rapier2d::vector::Vector;
use crate::servers::rapier_physics_singleton_2d::physics_singleton;
use godot::builtin::Vector2;
use godot::{builtin::Rid, engine::physics_server_2d};
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
        let lock = physics_singleton().lock().unwrap();
        let body_a = lock.collision_objects.get(&body_a);
        assert!(body_a.is_some());
        let body_a = body_a.unwrap();
        let body_a_handle = body_a.get_base().get_body_handle();
        let anchor_a = body_a.get_base().get_inv_transform().basis_xform(pos);
        let space_a = lock.spaces.get(&body_a.get_base().get_space());
        assert!(space_a.is_some());
        let space_handle_a = space_a.unwrap().get_handle();
        let mut space_handle_b = invalid_handle();
        let mut body_b_handle = invalid_handle();
        let mut anchor_b = pos;
        if body_b.is_valid() {
            let body_b = lock.collision_objects.get(&body_b);
            if let Some(body_b) = body_b {
                body_b_handle = body_b.get_base().get_body_handle();
                anchor_b = body_b.get_base().get_inv_transform().basis_xform(pos);
                let space_b = body_b.get_base().get_space();
                if let Some(space_b) = lock.spaces.get(&space_b) {
                    space_handle_b = space_b.get_handle();
                }
            }
        }
        assert!(space_handle_a != space_handle_b);

        let rapier_anchor_a = Vector::new(anchor_a.x, anchor_a.y);
        let rapier_anchor_b = Vector::new(anchor_b.x, anchor_b.y);
        let handle = joint_create_revolute(
            space_handle_a,
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
            base: RapierJointBase2D::new(space_handle_a, handle, rid),
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
    fn get_type(&self) -> physics_server_2d::JointType {
        physics_server_2d::JointType::PIN
    }
}
