use godot::classes::*;
#[cfg(feature = "dim2")]
use physics_server_2d::JointType;
#[cfg(feature = "dim3")]
use physics_server_3d::JointType;

#[cfg(feature = "dim3")]
use super::rapier_cone_twist_joint_3d::RapierConeTwistJoint3D;
#[cfg(feature = "dim2")]
use super::rapier_damped_spring_joint_2d::RapierDampedSpringJoint2D;
#[cfg(feature = "dim3")]
use super::rapier_generic_6dof_joint_3d::RapierGeneric6DOFJoint3D;
use super::rapier_revolute_joint::RapierRevoluteJoint;
#[cfg(feature = "dim3")]
use super::rapier_slider_joint_3d::RapierSliderJoint3D;
#[cfg(feature = "dim3")]
use super::rapier_spherical_joint_3d::RapierSphericalJoint3D;
use crate::rapier_wrapper::prelude::*;
use crate::types::invalid_rid;
use crate::*;
#[cfg_attr(feature = "serde-serialize", typetag::serde(tag = "type"))]
pub trait IRapierJoint {
    fn get_base(&self) -> &RapierJointBase;
    fn get_mut_base(&mut self) -> &mut RapierJointBase;
    fn get_type(&self) -> JointType;
    #[cfg(feature = "dim2")]
    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D>;
    fn get_revolute(&self) -> Option<&RapierRevoluteJoint>;
    #[cfg(feature = "dim3")]
    fn get_spherical(&self) -> Option<&RapierSphericalJoint3D>;
    #[cfg(feature = "dim3")]
    fn get_cone_twist(&self) -> Option<&RapierConeTwistJoint3D>;
    #[cfg(feature = "dim3")]
    fn get_generic_6dof(&self) -> Option<&RapierGeneric6DOFJoint3D>;
    #[cfg(feature = "dim3")]
    fn get_slider(&self) -> Option<&RapierSliderJoint3D>;
    #[cfg(feature = "dim2")]
    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D>;
    fn get_mut_revolute(&mut self) -> Option<&mut RapierRevoluteJoint>;
    #[cfg(feature = "dim3")]
    fn get_mut_spherical(&mut self) -> Option<&mut RapierSphericalJoint3D>;
    #[cfg(feature = "dim3")]
    fn get_mut_cone_twist(&mut self) -> Option<&mut RapierConeTwistJoint3D>;
    #[cfg(feature = "dim3")]
    fn get_mut_generic_6dof(&mut self) -> Option<&mut RapierGeneric6DOFJoint3D>;
    #[cfg(feature = "dim3")]
    fn get_mut_slider(&mut self) -> Option<&mut RapierSliderJoint3D>;
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierJointBase {
    max_force: f32,
    handle: JointHandle,
    space_handle: WorldHandle,
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "invalid_rid"))]
    space_rid: Rid,
    disabled_collisions_between_bodies: bool,
}
impl Default for RapierJointBase {
    fn default() -> Self {
        Self::new(WorldHandle::default(), Rid::Invalid, JointHandle::default())
    }
}
impl RapierJointBase {
    pub fn new(space_handle: WorldHandle, space_rid: Rid, handle: JointHandle) -> Self {
        Self {
            max_force: f32::MAX,
            handle,
            space_handle,
            space_rid,
            disabled_collisions_between_bodies: true,
        }
    }

    pub fn get_handle(&self) -> JointHandle {
        self.handle
    }

    pub fn get_space_handle(&self) -> WorldHandle {
        self.space_handle
    }

    pub fn get_space(&self) -> Rid {
        self.space_rid
    }

    pub fn set_max_force(&mut self, force: f32) {
        self.max_force = force;
    }

    pub fn get_max_force(&self) -> f32 {
        self.max_force
    }

    pub fn is_valid(&self) -> bool {
        self.space_handle != WorldHandle::default() && self.handle != JointHandle::default()
    }

    pub fn disable_collisions_between_bodies(
        &mut self,
        disabled: bool,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.disabled_collisions_between_bodies = disabled;
        if self.is_valid() {
            physics_engine.joint_change_disable_collision(
                self.space_handle,
                self.handle,
                self.disabled_collisions_between_bodies,
            );
        }
    }

    pub fn is_disabled_collisions_between_bodies(&self) -> bool {
        self.disabled_collisions_between_bodies
    }

    pub fn copy_settings_from(
        &mut self,
        joint: &RapierJointBase,
        physics_engine: &mut PhysicsEngine,
    ) {
        self.set_max_force(joint.get_max_force());
        self.disable_collisions_between_bodies(
            joint.is_disabled_collisions_between_bodies(),
            physics_engine,
        );
    }

    pub fn destroy_joint(&mut self, physics_engine: &mut PhysicsEngine) {
        physics_engine.destroy_joint(self.space_handle, self.handle);
        self.handle = JointHandle::default();
    }
}
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct RapierEmptyJoint {
    base: RapierJointBase,
}
impl RapierEmptyJoint {
    pub fn new() -> Self {
        Self {
            base: RapierJointBase::new(
                WorldHandle::default(),
                Rid::Invalid,
                JointHandle::default(),
            ),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierJoint for RapierEmptyJoint {
    fn get_type(&self) -> JointType {
        JointType::MAX
    }

    fn get_base(&self) -> &RapierJointBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        &mut self.base
    }

    #[cfg(feature = "dim2")]
    fn get_damped_spring(&self) -> Option<&RapierDampedSpringJoint2D> {
        None
    }

    fn get_revolute(&self) -> Option<&RapierRevoluteJoint> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_spherical(&self) -> Option<&RapierSphericalJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_cone_twist(&self) -> Option<&RapierConeTwistJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_generic_6dof(&self) -> Option<&RapierGeneric6DOFJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_slider(&self) -> Option<&RapierSliderJoint3D> {
        None
    }

    #[cfg(feature = "dim2")]
    fn get_mut_damped_spring(&mut self) -> Option<&mut RapierDampedSpringJoint2D> {
        None
    }

    fn get_mut_revolute(&mut self) -> Option<&mut RapierRevoluteJoint> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_mut_spherical(&mut self) -> Option<&mut RapierSphericalJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_mut_cone_twist(&mut self) -> Option<&mut RapierConeTwistJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_mut_generic_6dof(&mut self) -> Option<&mut RapierGeneric6DOFJoint3D> {
        None
    }

    #[cfg(feature = "dim3")]
    fn get_mut_slider(&mut self) -> Option<&mut RapierSliderJoint3D> {
        None
    }
}
impl Drop for RapierJointBase {
    fn drop(&mut self) {
        if self.handle != JointHandle::default() {
            godot_error!("RapierJointBase leaked");
        }
    }
}
