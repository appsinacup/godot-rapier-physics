use godot::classes::*;
#[cfg(feature = "dim2")]
use joints::rapier_groove_joint_2d::RapierGrooveJoint2D;
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
#[cfg_attr(
    feature = "serde-serialize",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum RapierJoint {
    #[cfg(feature = "dim2")]
    RapierDampedSpringJoint2D(RapierDampedSpringJoint2D),
    #[cfg(feature = "dim2")]
    RapierGrooveJoint2D(RapierGrooveJoint2D),
    RapierEmptyJoint(RapierEmptyJoint),
    RapierRevoluteJoint(RapierRevoluteJoint),
    #[cfg(feature = "dim3")]
    RapierSliderJoint3D(RapierSliderJoint3D),
    #[cfg(feature = "dim3")]
    RapierConeTwistJoint3D(RapierConeTwistJoint3D),
    #[cfg(feature = "dim3")]
    RapierSphericalJoint3D(RapierSphericalJoint3D),
    #[cfg(feature = "dim3")]
    RapierGeneric6DOFJoint3D(RapierGeneric6DOFJoint3D),
}
#[cfg_attr(feature = "serde-serialize", typetag::serde)]
impl IRapierJoint for RapierJoint {
    fn get_base(&self) -> &RapierJointBase {
        match self {
            #[cfg(feature = "dim2")]
            RapierJoint::RapierDampedSpringJoint2D(joint) => joint.get_base(),
            #[cfg(feature = "dim2")]
            RapierJoint::RapierGrooveJoint2D(joint) => joint.get_base(),
            RapierJoint::RapierEmptyJoint(joint) => joint.get_base(),
            RapierJoint::RapierRevoluteJoint(joint) => joint.get_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierSliderJoint3D(joint) => joint.get_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierConeTwistJoint3D(joint) => joint.get_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierSphericalJoint3D(joint) => joint.get_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierGeneric6DOFJoint3D(joint) => joint.get_base(),
        }
    }

    fn get_mut_base(&mut self) -> &mut RapierJointBase {
        match self {
            #[cfg(feature = "dim2")]
            RapierJoint::RapierDampedSpringJoint2D(joint) => joint.get_mut_base(),
            #[cfg(feature = "dim2")]
            RapierJoint::RapierGrooveJoint2D(joint) => joint.get_mut_base(),
            RapierJoint::RapierEmptyJoint(joint) => joint.get_mut_base(),
            RapierJoint::RapierRevoluteJoint(joint) => joint.get_mut_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierSliderJoint3D(joint) => joint.get_mut_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierConeTwistJoint3D(joint) => joint.get_mut_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierSphericalJoint3D(joint) => joint.get_mut_base(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierGeneric6DOFJoint3D(joint) => joint.get_mut_base(),
        }
    }

    fn get_type(&self) -> JointType {
        match self {
            #[cfg(feature = "dim2")]
            RapierJoint::RapierDampedSpringJoint2D(joint) => joint.get_type(),
            #[cfg(feature = "dim2")]
            RapierJoint::RapierGrooveJoint2D(joint) => joint.get_type(),
            RapierJoint::RapierEmptyJoint(joint) => joint.get_type(),
            RapierJoint::RapierRevoluteJoint(joint) => joint.get_type(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierSliderJoint3D(joint) => joint.get_type(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierConeTwistJoint3D(joint) => joint.get_type(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierSphericalJoint3D(joint) => joint.get_type(),
            #[cfg(feature = "dim3")]
            RapierJoint::RapierGeneric6DOFJoint3D(joint) => joint.get_type(),
        }
    }
}
#[cfg_attr(feature = "serde-serialize", typetag::serde(tag = "type"))]
pub trait IRapierJoint {
    fn get_base(&self) -> &RapierJointBase;
    fn get_mut_base(&mut self) -> &mut RapierJointBase;
    fn get_type(&self) -> JointType;
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
impl Default for RapierEmptyJoint {
    fn default() -> Self {
        Self::new()
    }
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
}
impl Drop for RapierJointBase {
    fn drop(&mut self) {
        if self.handle != JointHandle::default() {
            godot_error!("RapierJointBase leaked");
        }
    }
}
