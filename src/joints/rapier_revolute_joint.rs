use godot::classes::*;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;

#[cfg(feature = "dim2")]
use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
#[cfg(feature = "dim3")]
use super::rapier_spherical_joint_3d::RapierSphericalJoint3D;
use crate::bodies::rapier_collision_object::IRapierCollisionObject;
use crate::joints::rapier_joint::IRapierJoint;
use crate::joints::rapier_joint::RapierJointBase;
use crate::rapier_wrapper::prelude::*;
use crate::types::Vector;
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
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
        body_a: &dyn IRapierCollisionObject,
        body_b: &dyn IRapierCollisionObject,
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
        let space_rid = body_a.get_base().get_space();
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
            base: RapierJointBase::new(space_handle, space_rid, handle),
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
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
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

    #[cfg(feature = "dim2")]
    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D> {
        None
    }

    fn get_revolute(&self) -> Option<&RapierRevoluteJoint> {
        Some(self)
    }

    #[cfg(feature = "dim3")]
    fn get_spherical(&self) -> Option<&RapierSphericalJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_cone_twist(&self) -> Option<&super::rapier_cone_twist_joint_3d::RapierConeTwistJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_generic_6dof(
        &self,
    ) -> Option<&super::rapier_generic_6dof_joint_3d::RapierGeneric6DOFJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_slider(&self) -> Option<&super::rapier_slider_joint_3d::RapierSliderJoint3D> {
        None
    }

    #[cfg(feature = "dim2")]
    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }

    fn get_mut_revolute(&mut self) -> Option<&mut RapierRevoluteJoint> {
        Some(self)
    }

    #[cfg(feature = "dim3")]
    fn get_mut_spherical(&mut self) -> Option<&mut RapierSphericalJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_mut_cone_twist(
        &mut self,
    ) -> Option<&mut super::rapier_cone_twist_joint_3d::RapierConeTwistJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_mut_generic_6dof(
        &mut self,
    ) -> Option<&mut super::rapier_generic_6dof_joint_3d::RapierGeneric6DOFJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_mut_slider(
        &mut self,
    ) -> Option<&mut super::rapier_slider_joint_3d::RapierSliderJoint3D> {
        None
    }
}
