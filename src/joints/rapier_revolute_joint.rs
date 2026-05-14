use godot::builtin::Rid;
use godot::classes::*;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;

use super::rapier_joint_base::RapierJointBase;
use super::rapier_joint_base::RapierJointType;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::Vector;
#[cfg(feature = "dim3")]
use crate::types::basis_to_rapier;
#[cfg(feature = "dim2")]
use crate::types::world_to_local_no_scale;
pub struct RapierRevoluteJoint {
    angular_limit_lower: f32,
    angular_limit_upper: f32,
    motor_target_velocity: f32,
    motor_enabled: bool,
    angular_limit_enabled: bool,
    #[cfg(feature = "dim2")]
    softness: f32,
    motor_target_position: f32,
    motor_stiffness: f32,
    motor_damping: f32,
    motor_position_enabled: bool,
    #[cfg(feature = "dim3")]
    motor_max_force: f32,
    base: RapierJointBase,
}
impl RapierRevoluteJoint {
    #[cfg(feature = "dim2")]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: RapierId,
        rid: Rid,
        anchor_a: Vector,
        anchor_b: Vector,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        joint_type: RapierJointType,
    ) -> Self {
        let invalid_joint = Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
            #[cfg(feature = "dim2")]
            softness: 0.0,
            motor_target_position: 0.0,
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_position_enabled: false,
            base: RapierJointBase::default(),
        };
        let body_a_rid = body_a.get_base().get_rid();
        let body_b_rid = body_b.get_base().get_rid();
        if body_a_rid == body_b_rid {
            return invalid_joint;
        }
        if !body_a.get_base().is_valid()
            || !body_b.get_base().is_valid()
            || body_a.get_base().get_space_id() != body_b.get_base().get_space_id()
        {
            return invalid_joint;
        }
        // Convert world positions to local positions without scale for proper joint anchor placement
        let anchor_a = world_to_local_no_scale(&body_a.get_base().get_transform(), anchor_a);
        let anchor_b = world_to_local_no_scale(&body_b.get_base().get_transform(), anchor_b);
        let rapier_anchor_a = vector_to_rapier(anchor_a);
        let rapier_anchor_b = vector_to_rapier(anchor_b);
        let space_handle = body_a.get_base().get_space_id();
        let space_id = body_a.get_base().get_space_id();
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
            joint_type,
            0.0,
            0.0,
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
            softness: 0.0,
            motor_target_position: 0.0,
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_position_enabled: false,
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle, joint_type),
        }
    }

    #[cfg(feature = "dim3")]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: RapierId,
        rid: Rid,
        anchor_a: Vector,
        anchor_b: Vector,
        axis_a: godot::prelude::Basis,
        axis_b: godot::prelude::Basis,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        joint_type: RapierJointType,
    ) -> Self {
        let invalid_joint = Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
            motor_target_position: 0.0,
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_position_enabled: false,
            motor_max_force: 0.0,
            base: RapierJointBase::default(),
        };
        let body_a_rid = body_a.get_base().get_rid();
        let body_b_rid = body_b.get_base().get_rid();
        if body_a_rid == body_b_rid {
            return invalid_joint;
        }
        if !body_a.get_base().is_valid()
            || !body_b.get_base().is_valid()
            || body_a.get_base().get_space_id() != body_b.get_base().get_space_id()
        {
            return invalid_joint;
        }
        let rapier_anchor_a = vector_to_rapier(anchor_a);
        let rapier_anchor_b = vector_to_rapier(anchor_b);
        let rapier_axis_a = basis_to_rapier(axis_a);
        let rapier_axis_b = basis_to_rapier(axis_b);
        let space_handle = body_a.get_base().get_space_id();
        let space_id = body_a.get_base().get_space_id();
        let handle = physics_engine.joint_create_revolute(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            rapier_anchor_a,
            rapier_anchor_b,
            rapier_axis_a,
            rapier_axis_b,
            0.0,
            0.0,
            false,
            0.0,
            false,
            joint_type,
            0.0,
            0.0,
            0.0,
            false,
            0.0,
            true,
        );
        Self {
            angular_limit_lower: 0.0,
            angular_limit_upper: 0.0,
            motor_target_velocity: 0.0,
            motor_enabled: false,
            angular_limit_enabled: false,
            motor_target_position: 0.0,
            motor_stiffness: 0.0,
            motor_damping: 0.0,
            motor_position_enabled: false,
            motor_max_force: 0.0,
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle, joint_type),
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
            physics_server_2d::PinJointParam::SOFTNESS => {
                self.softness = p_value;
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.joint_change_revolute_params(
            self.base.get_space_id(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
            self.softness,
            self.motor_target_position,
            self.motor_stiffness,
            self.motor_damping,
            self.motor_position_enabled,
        );
    }

    pub fn set_motor_position_options(
        &mut self,
        physics_engine: &mut PhysicsEngine,
        motor_target_position: f32,
        motor_stiffness: f32,
        motor_damping: f32,
        motor_position_enabled: bool,
    ) {
        if !self.base.is_valid() {
            return;
        }
        self.motor_target_position = motor_target_position;
        self.motor_stiffness = motor_stiffness;
        self.motor_damping = motor_damping;
        self.motor_position_enabled = motor_position_enabled;
        #[cfg(feature = "dim2")]
        let softness = self.softness;
        #[cfg(feature = "dim3")]
        let softness: f32 = 1.0;
        physics_engine.joint_change_revolute_params(
            self.base.get_space_id(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
            softness,
            motor_target_position,
            motor_stiffness,
            motor_damping,
            motor_position_enabled,
            #[cfg(feature = "dim3")]
            self.motor_max_force,
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
            physics_server_3d::HingeJointParam::MOTOR_MAX_IMPULSE => {
                self.motor_max_force = p_value / Self::estimate_physics_step();
            }
            _ => {}
        }
        if !self.base.is_valid() {
            return;
        }
        physics_engine.joint_change_revolute_params(
            self.base.get_space_id(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
            1.0,
            self.motor_target_position,
            self.motor_stiffness,
            self.motor_damping,
            self.motor_position_enabled,
            self.motor_max_force,
        );
    }

    #[cfg(feature = "dim2")]
    pub fn get_param(&self, p_param: physics_server_2d::PinJointParam) -> f32 {
        match p_param {
            physics_server_2d::PinJointParam::LIMIT_UPPER => self.angular_limit_upper,
            physics_server_2d::PinJointParam::LIMIT_LOWER => self.angular_limit_lower,
            physics_server_2d::PinJointParam::MOTOR_TARGET_VELOCITY => self.motor_target_velocity,
            physics_server_2d::PinJointParam::SOFTNESS => self.softness,
            _ => 0.0,
        }
    }

    // This function is used to convert MOTOR_MAX_IMPULSE into a force which
    // can be passed onto rapier.
    #[cfg(feature = "dim3")]
    pub fn estimate_physics_step() -> f32 {
        /*
        // Use this method if we want to match Jolt's approach
        // as of Godot 4.6.2 (However this approach perpetuates step variant behaviour)
        let mut engine = <godot::classes::Engine as godot::obj::Singleton>::singleton();
        let step = 1.0 / engine.get_physics_ticks_per_second() as f64;
        let step_scaled = step * engine.get_time_scale();
        step_scaled as f32
        */
        // Use this method if we want consistent behaviour
        // during time scaling or changing of physics rate
        // when physics step is 60.0 (default) behaviour will match Jolt
        1.0 / 60.0
    }

    #[cfg(feature = "dim3")]
    pub fn get_param(&self, p_param: physics_server_3d::HingeJointParam) -> f32 {
        match p_param {
            physics_server_3d::HingeJointParam::LIMIT_UPPER => self.angular_limit_upper,
            physics_server_3d::HingeJointParam::LIMIT_LOWER => self.angular_limit_lower,
            physics_server_3d::HingeJointParam::MOTOR_TARGET_VELOCITY => self.motor_target_velocity,
            physics_server_3d::HingeJointParam::MOTOR_MAX_IMPULSE => {
                self.motor_max_force * Self::estimate_physics_step()
            }
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
            self.base.get_space_id(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
            self.softness,
            self.motor_target_position,
            self.motor_stiffness,
            self.motor_damping,
            self.motor_position_enabled,
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
            self.base.get_space_id(),
            self.base.get_handle(),
            self.angular_limit_lower,
            self.angular_limit_upper,
            self.angular_limit_enabled,
            self.motor_target_velocity,
            self.motor_enabled,
            1.0,
            self.motor_target_position,
            self.motor_stiffness,
            self.motor_damping,
            self.motor_position_enabled,
            self.motor_max_force,
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
