use godot::classes::*;
use godot::prelude::*;

use super::rapier_joint_base::RapierJointBase;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::bodies::rapier_collision_object::RapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::rapier_wrapper::prelude::*;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::types::*;
#[derive(Clone, Copy)]
struct AxisParams {
    linear_lower_limit: f32,
    linear_upper_limit: f32,
    linear_limit_softness: f32,
    linear_restitution: f32,
    linear_damping: f32,
    linear_motor_target_velocity: f32,
    linear_motor_force_limit: f32,
    linear_spring_stiffness: f32,
    linear_spring_damping: f32,
    linear_spring_equilibrium_point: f32,
    angular_lower_limit: f32,
    angular_upper_limit: f32,
    angular_limit_softness: f32,
    angular_restitution: f32,
    angular_damping: f32,
    angular_motor_target_velocity: f32,
    angular_motor_force_limit: f32,
    angular_spring_stiffness: f32,
    angular_spring_damping: f32,
    angular_spring_equilibrium_point: f32,
    enable_linear_limit: bool,
    enable_angular_limit: bool,
    enable_linear_motor: bool,
    enable_angular_motor: bool,
    enable_linear_spring: bool,
    enable_angular_spring: bool,
}
impl Default for AxisParams {
    fn default() -> Self {
        Self {
            linear_lower_limit: 0.0,
            linear_upper_limit: 0.0,
            linear_limit_softness: 0.7,
            linear_restitution: 0.5,
            linear_damping: 1.0,
            linear_motor_target_velocity: 0.0,
            linear_motor_force_limit: 0.0,
            linear_spring_stiffness: 0.0,
            linear_spring_damping: 0.0,
            linear_spring_equilibrium_point: 0.0,
            angular_lower_limit: 0.0,
            angular_upper_limit: 0.0,
            angular_limit_softness: 0.7,
            angular_restitution: 0.0,
            angular_damping: 1.0,
            angular_motor_target_velocity: 0.0,
            angular_motor_force_limit: 0.0,
            angular_spring_stiffness: 0.0,
            angular_spring_damping: 0.0,
            angular_spring_equilibrium_point: 0.0,
            enable_linear_limit: false,
            enable_angular_limit: true,
            enable_linear_motor: false,
            enable_angular_motor: false,
            enable_linear_spring: false,
            enable_angular_spring: false,
        }
    }
}
pub struct RapierGeneric6DOFJoint3D {
    base: RapierJointBase,
    axis_data: [AxisParams; 3], // X, Y, Z axes
}
impl RapierGeneric6DOFJoint3D {
    pub fn new(
        id: RapierId,
        rid: Rid,
        anchor_a: Vector3,
        anchor_b: Vector3,
        axis_a: Basis,
        axis_b: Basis,
        body_a: &RapierCollisionObject,
        body_b: &RapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
    ) -> Self {
        let invalid_joint = Self {
            base: RapierJointBase::default(),
            axis_data: [AxisParams::default(); 3],
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
        let handle = physics_engine.joint_create_generic_6dof(
            space_handle,
            body_a.get_base().get_body_handle(),
            body_b.get_base().get_body_handle(),
            rapier_anchor_a,
            rapier_anchor_b,
            rapier_axis_a,
            rapier_axis_b,
            false,
            false,
            true,
        );
        Self {
            base: RapierJointBase::new(id, rid, space_id, space_handle, handle),
            axis_data: [AxisParams::default(); 3],
        }
    }

    pub fn set_param(
        &mut self,
        axis: Vector3Axis,
        param: physics_server_3d::G6dofJointAxisParam,
        value: f32,
    ) {
        let axis_idx = axis.ord() as usize;
        if axis_idx >= 3 {
            return;
        }
        let axis_params = &mut self.axis_data[axis_idx];
        match param {
            physics_server_3d::G6dofJointAxisParam::LINEAR_LOWER_LIMIT => {
                axis_params.linear_lower_limit = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_UPPER_LIMIT => {
                axis_params.linear_upper_limit = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_LIMIT_SOFTNESS => {
                axis_params.linear_limit_softness = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_RESTITUTION => {
                axis_params.linear_restitution = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_DAMPING => {
                axis_params.linear_damping = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_MOTOR_TARGET_VELOCITY => {
                axis_params.linear_motor_target_velocity = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_MOTOR_FORCE_LIMIT => {
                axis_params.linear_motor_force_limit = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_SPRING_STIFFNESS => {
                axis_params.linear_spring_stiffness = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_SPRING_DAMPING => {
                axis_params.linear_spring_damping = value;
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_SPRING_EQUILIBRIUM_POINT => {
                axis_params.linear_spring_equilibrium_point = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_LOWER_LIMIT => {
                axis_params.angular_lower_limit = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_UPPER_LIMIT => {
                axis_params.angular_upper_limit = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_LIMIT_SOFTNESS => {
                axis_params.angular_limit_softness = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_RESTITUTION => {
                axis_params.angular_restitution = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_DAMPING => {
                axis_params.angular_damping = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_MOTOR_TARGET_VELOCITY => {
                axis_params.angular_motor_target_velocity = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_MOTOR_FORCE_LIMIT => {
                axis_params.angular_motor_force_limit = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_STIFFNESS => {
                axis_params.angular_spring_stiffness = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_DAMPING => {
                axis_params.angular_spring_damping = value;
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_EQUILIBRIUM_POINT => {
                axis_params.angular_spring_equilibrium_point = value;
            }
            _ => {}
        }
    }

    pub fn get_param(
        &self,
        axis: Vector3Axis,
        param: physics_server_3d::G6dofJointAxisParam,
    ) -> f32 {
        let axis_idx = axis.ord() as usize;
        if axis_idx >= 3 {
            return 0.0;
        }
        let axis_params = &self.axis_data[axis_idx];
        match param {
            physics_server_3d::G6dofJointAxisParam::LINEAR_LOWER_LIMIT => {
                axis_params.linear_lower_limit
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_UPPER_LIMIT => {
                axis_params.linear_upper_limit
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_LIMIT_SOFTNESS => {
                axis_params.linear_limit_softness
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_RESTITUTION => {
                axis_params.linear_restitution
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_DAMPING => axis_params.linear_damping,
            physics_server_3d::G6dofJointAxisParam::LINEAR_MOTOR_TARGET_VELOCITY => {
                axis_params.linear_motor_target_velocity
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_MOTOR_FORCE_LIMIT => {
                axis_params.linear_motor_force_limit
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_SPRING_STIFFNESS => {
                axis_params.linear_spring_stiffness
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_SPRING_DAMPING => {
                axis_params.linear_spring_damping
            }
            physics_server_3d::G6dofJointAxisParam::LINEAR_SPRING_EQUILIBRIUM_POINT => {
                axis_params.linear_spring_equilibrium_point
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_LOWER_LIMIT => {
                axis_params.angular_lower_limit
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_UPPER_LIMIT => {
                axis_params.angular_upper_limit
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_LIMIT_SOFTNESS => {
                axis_params.angular_limit_softness
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_RESTITUTION => {
                axis_params.angular_restitution
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_DAMPING => axis_params.angular_damping,
            physics_server_3d::G6dofJointAxisParam::ANGULAR_MOTOR_TARGET_VELOCITY => {
                axis_params.angular_motor_target_velocity
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_MOTOR_FORCE_LIMIT => {
                axis_params.angular_motor_force_limit
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_STIFFNESS => {
                axis_params.angular_spring_stiffness
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_DAMPING => {
                axis_params.angular_spring_damping
            }
            physics_server_3d::G6dofJointAxisParam::ANGULAR_SPRING_EQUILIBRIUM_POINT => {
                axis_params.angular_spring_equilibrium_point
            }
            _ => 0.0,
        }
    }

    pub fn set_flag(
        &mut self,
        axis: Vector3Axis,
        flag: physics_server_3d::G6dofJointAxisFlag,
        enable: bool,
    ) {
        let axis_idx = axis.ord() as usize;
        if axis_idx >= 3 {
            return;
        }
        let axis_params = &mut self.axis_data[axis_idx];
        match flag {
            physics_server_3d::G6dofJointAxisFlag::ENABLE_LINEAR_LIMIT => {
                axis_params.enable_linear_limit = enable;
            }
            physics_server_3d::G6dofJointAxisFlag::ENABLE_ANGULAR_LIMIT => {
                axis_params.enable_angular_limit = enable;
            }
            physics_server_3d::G6dofJointAxisFlag::ENABLE_MOTOR => {
                axis_params.enable_linear_motor = enable;
                axis_params.enable_angular_motor = enable;
            }
            physics_server_3d::G6dofJointAxisFlag::ENABLE_LINEAR_SPRING => {
                axis_params.enable_linear_spring = enable;
            }
            physics_server_3d::G6dofJointAxisFlag::ENABLE_ANGULAR_SPRING => {
                axis_params.enable_angular_spring = enable;
            }
            _ => {}
        }
    }

    pub fn get_flag(&self, axis: Vector3Axis, flag: physics_server_3d::G6dofJointAxisFlag) -> bool {
        let axis_idx = axis.ord() as usize;
        if axis_idx >= 3 {
            return false;
        }
        let axis_params = &self.axis_data[axis_idx];
        match flag {
            physics_server_3d::G6dofJointAxisFlag::ENABLE_LINEAR_LIMIT => {
                axis_params.enable_linear_limit
            }
            physics_server_3d::G6dofJointAxisFlag::ENABLE_ANGULAR_LIMIT => {
                axis_params.enable_angular_limit
            }
            physics_server_3d::G6dofJointAxisFlag::ENABLE_MOTOR => axis_params.enable_linear_motor,
            physics_server_3d::G6dofJointAxisFlag::ENABLE_LINEAR_SPRING => {
                axis_params.enable_linear_spring
            }
            physics_server_3d::G6dofJointAxisFlag::ENABLE_ANGULAR_SPRING => {
                axis_params.enable_angular_spring
            }
            _ => false,
        }
    }
}
impl IRapierJoint for RapierGeneric6DOFJoint3D {
    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    fn get_type(&self) -> physics_server_3d::JointType {
        physics_server_3d::JointType::TYPE_6DOF
    }
}
